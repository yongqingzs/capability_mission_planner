#include <capability_mission_planner/offline_map_planner.hpp>
#include <capability_mission_planner/offline_planner_config.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace capability_mission_planner;
using namespace capability_mission_planner::offline;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

GridPosition different_free_cell(const MultiMapBundle& bundle, GridPosition origin) {
  const auto& map = bundle.map(origin.map_id);
  GridPosition best = origin;
  int best_distance = -1;
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      if (!map.is_traversable(x, y)) continue;
      const int distance = std::abs(x - origin.x) + std::abs(y - origin.y);
      if (distance > best_distance) {
        best = {origin.map_id, x, y};
        best_distance = distance;
      }
    }
  }
  if (best == origin) throw std::runtime_error("could not find a second free cell");
  return best;
}

void check_coordinate_round_trips(const MultiMapBundle& bundle) {
  for (const auto& [id, map] : bundle.maps) {
    bool checked = false;
    for (int y = 0; y < map.height && !checked; ++y) {
      for (int x = 0; x < map.width; ++x) {
        if (!map.is_traversable(x, y)) continue;
        const GridPosition grid{id, x, y};
        const auto local = map.grid_to_local(grid);
        require(map.local_to_grid(local.x, local.y) == grid,
          "grid/local round trip failed on " + id);
        const auto root = map.local_to_root(local);
        const auto restored = map.root_to_local(root);
        require(std::hypot(restored.x - local.x, restored.y - local.y) < 1e-8,
          "local/ROOT round trip failed on " + id);
        checked = true;
        break;
      }
    }
    require(checked, "map has no traversable test cell: " + id);
  }
}

void check_schedule_conflicts(const OfflineMissionPlan& plan) {
  std::size_t horizon = 0;
  for (const auto& schedule : plan.schedules) horizon = std::max(horizon, schedule.size());
  const auto at = [](const std::vector<TimedMapState>& schedule, std::size_t tick) {
    return tick < schedule.size() ? schedule[tick] : schedule.back();
  };
  for (std::size_t tick = 0; tick < horizon; ++tick) {
    for (std::size_t a = 0; a < plan.schedules.size(); ++a) {
      for (std::size_t b = a + 1U; b < plan.schedules.size(); ++b) {
        const auto first = at(plan.schedules[a], tick);
        const auto second = at(plan.schedules[b], tick);
        require(first.position != second.position, "vertex conflict remains in schedule");
        require(first.transition_id.empty() || first.transition_id != second.transition_id,
          "transition resource conflict remains in schedule");
        if (tick + 1U >= horizon) continue;
        const auto first_next = at(plan.schedules[a], tick + 1U);
        const auto second_next = at(plan.schedules[b], tick + 1U);
        require(!(first.position == second_next.position &&
          second.position == first_next.position), "opposing edge conflict remains in schedule");
      }
    }
  }
}

void check_multi_map_planning(const std::shared_ptr<const MultiMapBundle>& bundle) {
  require(bundle->is_multi_map(), "expected a multi-map bundle");
  require(!bundle->transitions.empty(), "multi-map bundle has no transitions");
  for (const auto& transition : bundle->transitions) {
    require(bundle->traversable(transition.from_cell), "blocked transition source");
    require(bundle->traversable(transition.to_cell), "blocked transition destination");
  }

  const auto& transition = bundle->transitions.front();
  MultiMapPathPlanner path_planner(bundle);
  bool rejected_without_capability = false;
  try {
    (void)path_planner.plan(transition.from_cell, transition.to_cell, {});
  } catch (const std::runtime_error&) {
    rejected_without_capability = true;
  }
  if (transition.type == "stairs")
    require(rejected_without_capability, "stairs path accepted without stairs capability");

  const auto path = path_planner.plan(
    transition.from_cell, transition.to_cell, {"stairs"});
  require(std::any_of(path.steps.begin(), path.steps.end(), [&](const auto& step) {
    return step.transition_id == transition.id;
  }), "cross-map path does not include the expected transition");

  const auto second_start = different_free_cell(*bundle, transition.from_cell);
  std::vector<MappedRobot> robots{
    {"mission_robot", transition.from_cell, {"camera", "fire", "stairs"}, true},
    {"idle_robot", second_start, {"thermal"}, true}};
  std::vector<MappedTask> tasks{
    make_mapped_task("photo", transition.to_cell, {"camera"}, "photo", 1),
    make_mapped_task("fire", transition.to_cell, {"fire"}, "fire", 1)};
  OfflineMissionPlanner mission_planner{MultiMapPathPlanner(bundle)};
  const auto plan = mission_planner.plan(robots, tasks, true);

  std::vector<int> assignments(tasks.size(), 0);
  for (const auto& route : plan.routes) {
    for (const auto& stop : route.stops) {
      for (const auto task : stop.task_indices) {
        require(task < tasks.size(), "invalid task index in route");
        ++assignments[task];
        for (const auto& requirement : tasks[task].requirements)
          require(robots[route.robot_index].capabilities.count(requirement) != 0U,
            "task assigned to an incapable robot");
      }
    }
  }
  require(std::all_of(assignments.begin(), assignments.end(),
    [](int count) { return count == 1; }), "task was not assigned exactly once");
  require(plan.schedules.size() == robots.size(), "coordinated schedules are missing");
  check_schedule_conflicts(plan);
}

void check_task_level_navigation_costs(const std::shared_ptr<const MultiMapBundle>& bundle) {
  const auto& map = bundle->maps.begin()->second;
  GridPosition start;
  GridPosition goal;
  for (int y = 0; y < map.height && start.map_id.empty(); ++y) {
    for (int x = 0; x < map.width; ++x) {
      if (map.is_traversable(x, y) && map.clearance(x, y) > 0.5) {
        start = {map.id, x, y};
        break;
      }
    }
  }
  for (int y = map.height - 1; y >= 0 && goal.map_id.empty(); --y) {
    for (int x = map.width - 1; x >= 0; --x) {
      if (map.is_traversable(x, y) && map.clearance(x, y) > 0.5 &&
        std::abs(x - start.x) + std::abs(y - start.y) > 20) {
        goal = {map.id, x, y};
        break;
      }
    }
  }
  require(!start.map_id.empty() && !goal.map_id.empty(),
    "could not find high-clearance task-level test endpoints");
  TraversalOptions options;
  options.obstacle_cost_weight = 2.0;
  options.allow_diagonal = true;
  MultiMapPathPlanner planner(bundle, options);
  const auto path = planner.plan(start, goal, {}, 0.25, 0.5);
  require(!path.steps.empty(), "high-clearance task-level path is empty");
  for (const auto& step : path.steps)
    require(bundle->map(step.position.map_id).clearance(step.position.x, step.position.y) >= 0.25,
      "task-level path violates robot clearance");
}

void check_task_tolerance_projection(
  const std::filesystem::path& map_directory,
  const std::shared_ptr<const MultiMapBundle>& bundle)
{
  const auto& map = bundle->maps.begin()->second;
  GridPosition blocked;
  GridPosition nearby_free;
  bool found = false;
  for (int y = 0; y < map.height && !found; ++y) {
    for (int x = 0; x < map.width && !found; ++x) {
      if (map.is_traversable(x, y)) continue;
      for (int dy = -2; dy <= 2 && !found; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const GridPosition candidate{map.id, x + dx, y + dy};
          if (bundle->traversable(candidate) && std::hypot(dx, dy) <= 2.0) {
            blocked = {map.id, x, y};
            nearby_free = candidate;
            found = true;
            break;
          }
        }
      }
    }
  }
  require(found, "could not find blocked cell adjacent to free cell");

  const auto config_path = std::filesystem::temp_directory_path() /
    "capability_mission_planner_tolerance_test.yaml";
  std::ofstream config(config_path);
  require(config.good(), "could not create tolerance test config");
  config << "version: 1\n"
    << "map:\n  directory: " << map_directory.string() << "\n"
    << "output_directory: tolerance-output\n"
    << "planner:\n  traversal: {}\n"
    << "robots:\n  - id: test\n    start:\n      map_id: " << map.id
    << "\n      grid: [" << nearby_free.x << ", " << nearby_free.y << "]\n"
    << "    capabilities: [camera]\n    return_home: false\n"
    << "tasks:\n  - id: tolerant\n    location:\n      map_id: " << map.id
    << "\n      grid: [" << blocked.x << ", " << blocked.y << "]\n"
    << "    position_tolerance_m: " << (3.0 * map.resolution) << "\n"
    << "    requirements: [camera]\n    category: photo\n    service_seconds: 1\n";
  config.close();

  const auto loaded = OfflinePlannerConfigLoader::load(config_path);
  require(loaded.tasks.size() == 1U, "tolerance test task was not loaded");
  require(loaded.tasks.front().position_tolerance_m > 0.0,
    "task position tolerance was not preserved");
  require(loaded.tasks.front().location != blocked,
    "blocked task location was not projected");
  require(bundle->traversable(loaded.tasks.front().location),
    "projected task location is not traversable");
  std::filesystem::remove(config_path);
}

} // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc != 3) throw std::invalid_argument("expected single-map and multi-map directories");
    const auto single = MapBundleLoader::load(argv[1]);
    const auto multi = MapBundleLoader::load(argv[2]);
    require(!single->is_multi_map(), "expected a single-map bundle");
    check_coordinate_round_trips(*single);
    check_coordinate_round_trips(*multi);
    check_multi_map_planning(multi);
    check_task_level_navigation_costs(single);
    check_task_tolerance_projection(argv[1], single);
    std::cout << "offline adapter tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "offline adapter tests failed: " << error.what() << '\n';
    return 1;
  }
}
