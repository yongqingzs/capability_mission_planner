#include <capability_mission_planner/offline_planner_config.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace capability_mission_planner::offline {
namespace {

std::filesystem::path resolve_path(
  const std::filesystem::path& config_path,
  const std::string& value)
{
  std::filesystem::path path(value);
  if (path.is_relative()) path = config_path.parent_path() / path;
  return std::filesystem::absolute(path).lexically_normal();
}

void require_map(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsMap()) throw std::runtime_error(name + " must be a map");
}

CapabilitySet capabilities(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence())
    throw std::runtime_error(name + " must be a sequence of strings");
  CapabilitySet result;
  for (const auto& item : node) {
    const auto value = item.as<std::string>();
    if (value.empty()) throw std::runtime_error(name + " contains an empty capability");
    result.insert(value);
  }
  return result;
}

GridPosition position(
  const YAML::Node& node,
  const MultiMapBundle& bundle,
  const std::string& name)
{
  require_map(node, name);
  if (!node["map_id"]) throw std::runtime_error(name + ".map_id is required");
  const auto map_id = node["map_id"].as<std::string>();
  const auto& map = bundle.map(map_id);
  const int representations = static_cast<int>(!!node["grid"]) +
    static_cast<int>(!!node["local_xy"]) + static_cast<int>(!!node["root_xy"]);
  if (representations != 1)
    throw std::runtime_error(name + " must contain exactly one of grid, local_xy, root_xy");

  GridPosition result;
  if (node["grid"]) {
    const auto value = node["grid"];
    if (!value.IsSequence() || value.size() != 2U)
      throw std::runtime_error(name + ".grid must contain [x, y]");
    result = {map_id, value[0].as<int>(), value[1].as<int>()};
  } else {
    const auto key = node["local_xy"] ? "local_xy" : "root_xy";
    const auto value = node[key];
    if (!value.IsSequence() || value.size() != 2U)
      throw std::runtime_error(name + "." + key + " must contain [x, y] in metres");
    MetricPose pose{map_id, value[0].as<double>(), value[1].as<double>()};
    if (node["root_xy"]) pose = map.root_to_local(pose);
    result = map.local_to_grid(pose.x, pose.y);
  }
  if (!bundle.traversable(result))
    throw std::runtime_error(name + " resolves to a blocked or out-of-map cell");
  return result;
}

void load_traversal(const YAML::Node& node, TraversalOptions& options) {
  if (!node) return;
  require_map(node, "planner.traversal");
  if (node["time_step_seconds"])
    options.time_step_seconds = node["time_step_seconds"].as<double>();
  if (node["nominal_speed_mps"])
    options.nominal_speed_mps = node["nominal_speed_mps"].as<double>();
  if (node["default_transition_seconds"])
    options.default_transition_seconds = node["default_transition_seconds"].as<double>();
  if (node["map_switch_seconds"])
    options.map_switch_seconds = node["map_switch_seconds"].as<double>();
  if (node["transition_seconds"]) {
    require_map(node["transition_seconds"], "planner.traversal.transition_seconds");
    for (const auto& item : node["transition_seconds"])
      options.transition_seconds[item.first.as<std::string>()] = item.second.as<double>();
  }
  if (node["transition_requirements"]) {
    require_map(node["transition_requirements"],
      "planner.traversal.transition_requirements");
    for (const auto& item : node["transition_requirements"]) {
      const auto type = item.first.as<std::string>();
      options.transition_requirements[type] = capabilities(
        item.second, "transition requirement " + type);
    }
  }
  if (!(options.time_step_seconds > 0.0) || !(options.nominal_speed_mps > 0.0) ||
    options.default_transition_seconds < 0.0 || options.map_switch_seconds < 0.0)
  {
    throw std::runtime_error("planner traversal times and speed are invalid");
  }
  for (const auto& [type, seconds] : options.transition_seconds)
    if (seconds < 0.0) throw std::runtime_error("negative transition time for " + type);
}

} // namespace

ConfiguredMission OfflinePlannerConfigLoader::load(
  const std::filesystem::path& raw_config_path)
{
  const auto config_path = std::filesystem::absolute(raw_config_path).lexically_normal();
  const auto root = YAML::LoadFile(config_path.string());
  require_map(root, "configuration root");
  if (root["version"] && root["version"].as<int>() != 1)
    throw std::runtime_error("unsupported configuration version");

  require_map(root["map"], "map");
  if (!root["map"]["directory"])
    throw std::runtime_error("map.directory is required");
  MapLoadOptions map_options;
  if (root["map"]["allow_unknown"])
    map_options.allow_unknown = root["map"]["allow_unknown"].as<bool>();
  if (root["map"]["inflation_radius_m"])
    map_options.inflation_radius = root["map"]["inflation_radius_m"].as<double>();
  if (map_options.inflation_radius < 0.0)
    throw std::runtime_error("map.inflation_radius_m must be non-negative");

  ConfiguredMission result;
  result.bundle = MapBundleLoader::load(
    resolve_path(config_path, root["map"]["directory"].as<std::string>()), map_options);
  if (!root["output_directory"])
    throw std::runtime_error("output_directory is required");
  result.output_directory = resolve_path(
    config_path, root["output_directory"].as<std::string>());

  const auto planner = root["planner"];
  if (planner) require_map(planner, "planner");
  load_traversal(planner ? planner["traversal"] : YAML::Node{}, result.traversal);
  if (planner && planner["objective"]) {
    require_map(planner["objective"], "planner.objective");
    const auto objective = planner["objective"];
    if (objective["maximum_load_weight"])
      result.objective.maximum_load = objective["maximum_load_weight"].as<double>();
    if (objective["total_load_weight"])
      result.objective.total_load = objective["total_load_weight"].as<double>();
    if (result.objective.maximum_load < 0.0 || result.objective.total_load < 0.0)
      throw std::runtime_error("planner objective weights must be non-negative");
  }
  if (planner && planner["coordinate_conflicts"])
    result.coordinate_conflicts = planner["coordinate_conflicts"].as<bool>();

  if (root["export"]) {
    require_map(root["export"], "export");
    if (root["export"]["path_thickness"])
      result.export_options.path_thickness = root["export"]["path_thickness"].as<int>();
    if (root["export"]["draw_grid"])
      result.export_options.draw_grid = root["export"]["draw_grid"].as<bool>();
    if (result.export_options.path_thickness <= 0)
      throw std::runtime_error("export.path_thickness must be positive");
  }

  if (!root["robots"] || !root["robots"].IsSequence())
    throw std::runtime_error("robots must be a sequence");
  for (std::size_t i = 0; i < root["robots"].size(); ++i) {
    const auto node = root["robots"][i];
    require_map(node, "robot");
    if (!node["id"] || !node["start"] || !node["capabilities"])
      throw std::runtime_error("each robot requires id, start, and capabilities");
    result.robots.push_back({node["id"].as<std::string>(),
      position(node["start"], *result.bundle, "robot start"),
      capabilities(node["capabilities"], "robot capabilities"),
      node["return_home"] ? node["return_home"].as<bool>() : true});
  }

  if (!root["tasks"] || !root["tasks"].IsSequence())
    throw std::runtime_error("tasks must be a sequence");
  for (std::size_t i = 0; i < root["tasks"].size(); ++i) {
    const auto node = root["tasks"][i];
    require_map(node, "task");
    if (!node["id"] || !node["location"] || !node["requirements"] ||
      !node["category"] || !node["service_seconds"])
    {
      throw std::runtime_error(
        "each task requires id, location, requirements, category, and service_seconds");
    }
    result.tasks.push_back(make_mapped_task(
      node["id"].as<std::string>(),
      position(node["location"], *result.bundle, "task location"),
      capabilities(node["requirements"], "task requirements"),
      node["category"].as<std::string>(), node["service_seconds"].as<int>(),
      node["high_priority"] ? node["high_priority"].as<bool>() : false));
  }
  return result;
}

} // namespace capability_mission_planner::offline
