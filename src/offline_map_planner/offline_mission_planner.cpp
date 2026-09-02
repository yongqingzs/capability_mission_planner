#include <capability_mission_planner/offline_map_planner.hpp>

#include "multi_map_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>

namespace capability_mission_planner::offline {
namespace {

bool compatible(const MappedRobot& robot, const MappedTask& task) {
  return std::all_of(task.requirements.begin(), task.requirements.end(),
    [&](const auto& requirement) {
      return robot.capabilities.count(requirement) != 0U;
    });
}

void recompute_route(
  MappedRobotRoute& route,
  const MappedRobot& robot,
  const MultiMapPathPlanner& planner,
  bool exact_paths)
{
  route.travel_ticks = 0;
  route.service_ticks = 0;
  route.segments.clear();
  GridPosition current = robot.start;
  const double clearance = std::max(0.0, robot.clearance_radius_m + robot.safety_margin_m);
  const double speed = robot.nominal_speed_mps > 0.0 ? robot.nominal_speed_mps :
    planner.options().nominal_speed_mps;
  for (const auto& stop : route.stops) {
    auto segment_ticks = exact_paths ?
      planner.plan(current, stop.location, robot.capabilities, clearance, speed).travel_ticks :
      planner.estimate_distance(current, stop.location, robot.capabilities,
        clearance, speed);
    route.travel_ticks += segment_ticks;
    route.service_ticks += stop.service_ticks;
  if (exact_paths)
      route.segments.push_back(planner.plan(current, stop.location,
        robot.capabilities, clearance, speed));
    current = stop.location;
  }
  if (robot.return_home && !route.stops.empty()) {
    auto segment_ticks = exact_paths ?
      planner.plan(current, robot.start, robot.capabilities, clearance, speed).travel_ticks :
      planner.estimate_distance(current, robot.start, robot.capabilities,
        clearance, speed);
    route.travel_ticks += segment_ticks;
    if (exact_paths)
      route.segments.push_back(planner.plan(current, robot.start,
        robot.capabilities, clearance, speed));
  }
}

double objective(
  const std::vector<MappedRobotRoute>& routes,
  const ObjectiveWeights& weights)
{
  int maximum = 0;
  int total = 0;
  for (const auto& route : routes) {
    maximum = std::max(maximum, route.load_ticks());
    total += route.load_ticks();
  }
  return weights.maximum_load * maximum + weights.total_load * total;
}

std::vector<MappedRobotRoute> insertion_options(
  const MappedRobotRoute& source,
  std::size_t task_index,
  const MappedTask& task,
  int service_ticks)
{
  for (std::size_t i = 0; i < source.stops.size(); ++i) {
    if (source.stops[i].location == task.location) {
      auto merged = source;
      merged.stops[i].task_indices.push_back(task_index);
      merged.stops[i].service_ticks += service_ticks;
      merged.stops[i].position_tolerance_m = std::max(
        merged.stops[i].position_tolerance_m, task.position_tolerance_m);
      return {std::move(merged)};
    }
  }
  std::vector<MappedRobotRoute> result;
  for (std::size_t position = 0; position <= source.stops.size(); ++position) {
    auto inserted = source;
    inserted.stops.insert(
      inserted.stops.begin() + static_cast<std::ptrdiff_t>(position),
      MappedRouteStop{task.location, {task_index}, service_ticks,
        task.position_tolerance_m});
    result.push_back(std::move(inserted));
  }
  return result;
}

bool remove_task(
  MappedRobotRoute& route,
  std::size_t task_index,
  int service_ticks)
{
  for (auto stop = route.stops.begin(); stop != route.stops.end(); ++stop) {
    const auto entry = std::find(
      stop->task_indices.begin(), stop->task_indices.end(), task_index);
    if (entry == stop->task_indices.end())
      continue;
    stop->task_indices.erase(entry);
    stop->service_ticks -= service_ticks;
    if (stop->task_indices.empty())
      route.stops.erase(stop);
    return true;
  }
  return false;
}

std::size_t compatible_robot_count(
  const std::vector<MappedRobot>& robots,
  const MappedTask& task)
{
  return static_cast<std::size_t>(std::count_if(
    robots.begin(), robots.end(), [&](const auto& robot) {
      return compatible(robot, task);
    }));
}

const SharedResource* resource_at(const std::vector<SharedResource>& resources,
  const GridPosition& position)
{
  for (const auto& resource : resources)
    if (std::find(resource.cells.begin(), resource.cells.end(), position) != resource.cells.end())
      return &resource;
  return nullptr;
}

int tick_at_frame(const std::vector<TimedMapState>& schedule, std::size_t frame) {
  const auto found = std::find_if(schedule.begin(), schedule.end(), [&](const auto& state) {
    return state.route_frame == frame;
  });
  return found == schedule.end() ? schedule.back().tick : found->tick;
}

void append_path_checkpoints(std::vector<NavigationCheckpoint>& points,
  const MultiMapPath& path, std::size_t frame_offset,
  const std::vector<TimedMapState>& schedule)
{
  struct TurnCandidate {
    std::size_t index;
    GridPosition position;
    int tick;
  };
  std::vector<TurnCandidate> candidates;
  for (std::size_t i = 1; i + 1U < path.steps.size(); ++i) {
    const auto& previous = path.steps[i - 1U].position;
    const auto& current = path.steps[i].position;
    const auto& next = path.steps[i + 1U].position;
    if (previous.map_id != current.map_id || current.map_id != next.map_id) continue;
    const long long dx1 = current.x - previous.x, dy1 = current.y - previous.y;
    const long long dx2 = next.x - current.x, dy2 = next.y - current.y;
    const double norm1 = std::hypot(static_cast<double>(dx1), static_cast<double>(dy1));
    const double norm2 = std::hypot(static_cast<double>(dx2), static_cast<double>(dy2));
    const double cosine = (dx1 * dx2 + dy1 * dy2) / (norm1 * norm2);
    if (cosine > std::cos(45.0 * 3.14159265358979323846 / 180.0)) continue;
    const auto frame = frame_offset + static_cast<std::size_t>(path.steps[i].arrival_tick);
    const int tick = tick_at_frame(schedule, frame);
    candidates.push_back({i, current, tick});
  }
  // A grid path often represents one physical bend with several alternating
  // diagonal/cardinal steps. Collapse candidates that are close along the
  // same bend and retain its middle point as the navigation waypoint.
  constexpr std::size_t max_turn_run = 8U;
  std::size_t begin = 0;
  while (begin < candidates.size()) {
    std::size_t end = begin;
    while (end + 1U < candidates.size() &&
      candidates[end + 1U].index - candidates[end].index <= max_turn_run) ++end;
    const auto& representative = candidates[begin + (end - begin) / 2U];
    points.push_back({NavigationCheckpointType::Turn, representative.position,
      representative.tick, representative.tick, {}, {}, {}, {}});
    begin = end + 1U;
  }
  for (std::size_t i = 1; i < path.steps.size(); ++i) {
    if (path.steps[i].transition_id.empty()) continue;
    const auto& id = path.steps[i].transition_id;
    const auto entry_frame = frame_offset +
      static_cast<std::size_t>(path.steps[i - 1U].arrival_tick);
    const auto exit_frame = frame_offset +
      static_cast<std::size_t>(path.steps[i].arrival_tick);
    const int entry_tick = tick_at_frame(schedule, entry_frame);
    const int exit_tick = tick_at_frame(schedule, exit_frame);
    points.push_back({NavigationCheckpointType::TransitionEntry,
      path.steps[i - 1U].position, entry_tick, entry_tick, {}, id, {}, {}});
    points.push_back({NavigationCheckpointType::TransitionExit,
      path.steps[i].position, exit_tick, exit_tick, {}, id, {}, {}});
  }
}

void extract_navigation_annotations(OfflineMissionPlan& plan,
  const std::vector<MappedTask>& tasks)
{
  plan.navigation_checkpoints.assign(plan.schedules.size(), {});
  plan.traffic_events.assign(plan.schedules.size(), {});
  for (std::size_t robot = 0; robot < plan.schedules.size(); ++robot) {
    const auto& schedule = plan.schedules[robot];
    if (schedule.empty()) continue;
    auto& points = plan.navigation_checkpoints[robot];
    points.push_back({NavigationCheckpointType::Start, schedule.front().position,
      schedule.front().tick, schedule.front().tick, {}, {}, {}, {}});
    const auto& route = plan.routes.at(robot);
    std::size_t frame = 0;
    std::size_t segment = 0;
    for (const auto& stop : route.stops) {
      const auto& path = route.segments.at(segment++);
      append_path_checkpoints(points, path, frame, schedule);
      frame += static_cast<std::size_t>(path.travel_ticks);
      const int arrival_tick = tick_at_frame(schedule, frame);
      const int departure_tick = tick_at_frame(schedule,
        frame + static_cast<std::size_t>(stop.service_ticks));
      for (const auto task_index : stop.task_indices) {
        const auto task_id = stop.task_indices.empty() ? std::string{} :
          tasks.at(task_index).id();
        points.push_back({NavigationCheckpointType::Task, stop.location,
          arrival_tick, departure_tick, {}, {}, task_id, {}});
      }
      frame += static_cast<std::size_t>(stop.service_ticks);
    }
    if (segment < route.segments.size()) {
      append_path_checkpoints(points, route.segments.at(segment), frame, schedule);
    }

    std::size_t wait_index = 0;
    for (std::size_t i = 0; i < schedule.size(); ++i) {
      const auto& state = schedule[i];
      const auto* before = resource_at(plan.shared_resources, state.position);
      const auto* after = i + 1U < schedule.size()
        ? resource_at(plan.shared_resources, schedule[i + 1U].position) : nullptr;
      if (before != after) {
        if (before) points.push_back({NavigationCheckpointType::ResourceExit, state.position,
          state.tick, state.tick, before->id, {}, {}, {}});
        if (after) points.push_back({NavigationCheckpointType::ResourceEntry,
          schedule[i + 1U].position, schedule[i + 1U].tick, schedule[i + 1U].tick,
          after->id, {}, {}, {}});
      }
      std::size_t end = i;
      while (end + 1U < schedule.size() &&
        schedule[end + 1U].route_frame == state.route_frame) ++end;
      if (end > i) {
        const std::string checkpoint_id = "holding-" + std::to_string(wait_index++);
        points.push_back({NavigationCheckpointType::Holding, state.position,
          state.tick, schedule[end].tick, {}, {}, {}, checkpoint_id});
        plan.traffic_events[robot].push_back({"wait", state.tick,
          schedule[end].tick, state.position, {}, "coordination_delay", checkpoint_id});
        i = end;
      }
    }
    points.push_back({NavigationCheckpointType::Finish, schedule.back().position,
      schedule.back().tick, schedule.back().tick, {}, {}, {}, {}});
    std::stable_sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
      return a.arrival_tick < b.arrival_tick;
    });
    points.erase(std::unique(points.begin(), points.end(), [](const auto& a, const auto& b) {
      return a.type == NavigationCheckpointType::Turn &&
        b.type == NavigationCheckpointType::Turn;
    }), points.end());
    for (std::size_t i = 0; i < points.size(); ++i)
      if (points[i].id.empty()) points[i].id = "checkpoint-" + std::to_string(i);
  }
}

} // namespace

int MappedTask::service_duration_seconds() const {
  const auto seconds = std::chrono::duration<double>(
    header.original_duration_estimate()).count();
  return std::max(0, static_cast<int>(std::ceil(seconds)));
}

bool MappedTask::high_priority() const {
  return booking->priority() == TaskPriority::High;
}

MappedTask make_mapped_task(
  std::string id,
  GridPosition location,
  CapabilitySet requirements,
  std::string category,
  int service_seconds,
  bool high_priority,
  int earliest_start_seconds)
{
  const auto priority = high_priority ? TaskPriority::High : TaskPriority::Normal;
  const auto start = TaskTime{} + std::chrono::seconds(earliest_start_seconds);
  auto booking = std::make_shared<TaskBooking>(
    std::move(id), start, priority, false, std::vector<std::string>{category});
  TaskHeader header(
    std::move(category), "offline mapped atomic task",
    std::chrono::seconds(service_seconds));
  return {std::move(booking), std::move(header), std::move(location),
    std::move(requirements), 0.0};
}

OfflineMissionPlanner::OfflineMissionPlanner(
  MultiMapPathPlanner path_planner,
  ObjectiveWeights weights)
: _path_planner(std::move(path_planner)), _weights(weights)
{
  if (weights.maximum_load < 0.0 || weights.total_load < 0.0)
    throw std::invalid_argument("objective weights must be non-negative");
}

OfflineMissionPlan OfflineMissionPlanner::plan(
  const std::vector<MappedRobot>& robots,
  const std::vector<MappedTask>& tasks,
  bool coordinate_conflicts) const
{
  if (robots.empty() && !tasks.empty())
    throw std::invalid_argument("tasks cannot be assigned without robots");
  std::set<std::string> robot_ids;
  std::set<GridPosition> starts;
  for (const auto& robot : robots) {
    if (!robot_ids.insert(robot.id).second)
      throw std::invalid_argument("duplicate robot id: " + robot.id);
    if (!_path_planner.bundle().traversable(robot.start))
      throw std::invalid_argument("robot start is not traversable: " + robot.id);
    if (!starts.insert(robot.start).second)
      throw std::invalid_argument("robots cannot share an initial location");
  }
  std::set<std::string> task_ids;
  for (const auto& task : tasks) {
    if (!task_ids.insert(task.id()).second)
      throw std::invalid_argument("duplicate task id: " + task.id());
    if (!_path_planner.bundle().traversable(task.location))
      throw std::invalid_argument("task location is not traversable: " + task.id());
    if (compatible_robot_count(robots, task) == 0U)
      throw std::invalid_argument("no capable robot for task " + task.id());
  }

  const double time_step = _path_planner.options().time_step_seconds;
  std::vector<int> service_ticks(tasks.size());
  std::transform(tasks.begin(), tasks.end(), service_ticks.begin(), [&](const auto& task) {
    return static_cast<int>(std::ceil(
      task.service_duration_seconds() / time_step - 1e-9));
  });

  std::vector<MappedRobotRoute> routes(robots.size());
  for (std::size_t i = 0; i < routes.size(); ++i)
    routes[i].robot_index = i;

  std::vector<std::size_t> order(tasks.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    const auto candidates_a = compatible_robot_count(robots, tasks[a]);
    const auto candidates_b = compatible_robot_count(robots, tasks[b]);
    if (candidates_a != candidates_b)
      return candidates_a < candidates_b;
    if (tasks[a].high_priority() != tasks[b].high_priority())
      return tasks[a].high_priority();
    return service_ticks[a] > service_ticks[b];
  });

  for (const auto task_index : order) {
    double best_score = std::numeric_limits<double>::infinity();
    std::size_t best_robot = robots.size();
    MappedRobotRoute best_route;
    for (std::size_t robot_index = 0; robot_index < robots.size(); ++robot_index) {
      if (!compatible(robots[robot_index], tasks[task_index]))
        continue;
      for (auto candidate : insertion_options(
          routes[robot_index], task_index, tasks[task_index], service_ticks[task_index]))
      {
        try {
          recompute_route(candidate, robots[robot_index], _path_planner, false);
        } catch (const std::runtime_error&) {
          continue;
        }
        auto trial = routes;
        trial[robot_index] = candidate;
        const double score = objective(trial, _weights);
        if (score < best_score) {
          best_score = score;
          best_robot = robot_index;
          best_route = std::move(candidate);
        }
      }
    }
    if (best_robot == robots.size())
      throw std::runtime_error("no reachable capable robot for task " + tasks[task_index].id());
    routes[best_robot] = std::move(best_route);
  }

  for (std::size_t pass = 0; pass < tasks.size(); ++pass) {
    const double current_score = objective(routes, _weights);
    double best_score = current_score;
    std::vector<MappedRobotRoute> best_routes;
    for (std::size_t source = 0; source < robots.size(); ++source) {
      std::vector<std::size_t> source_tasks;
      for (const auto& stop : routes[source].stops)
        source_tasks.insert(source_tasks.end(), stop.task_indices.begin(), stop.task_indices.end());
      for (const auto task_index : source_tasks) {
        auto without = routes;
        if (!remove_task(without[source], task_index, service_ticks[task_index]))
          continue;
        recompute_route(without[source], robots[source], _path_planner, false);
        for (std::size_t target = 0; target < robots.size(); ++target) {
          if (!compatible(robots[target], tasks[task_index]))
            continue;
          for (auto candidate : insertion_options(
              without[target], task_index, tasks[task_index], service_ticks[task_index]))
          {
            try {
              recompute_route(candidate, robots[target], _path_planner, false);
            } catch (const std::runtime_error&) {
              continue;
            }
            auto trial = without;
            trial[target] = std::move(candidate);
            const double score = objective(trial, _weights);
            if (score + 1e-9 < best_score) {
              best_score = score;
              best_routes = std::move(trial);
            }
          }
        }
      }
    }
    if (best_routes.empty())
      break;
    routes = std::move(best_routes);
  }

  OfflineMissionPlan result;
  result.routes = std::move(routes);
  for (std::size_t i = 0; i < result.routes.size(); ++i)
    recompute_route(result.routes[i], robots[i], _path_planner, true);
  result.time_step_seconds = time_step;
  for (const auto& route : result.routes) {
    result.maximum_load_ticks = std::max(result.maximum_load_ticks, route.load_ticks());
    result.total_load_ticks += route.load_ticks();
  }
  if (coordinate_conflicts && !robots.empty()) {
    result.schedules = coordinate_multi_map_routes(
      _path_planner, robots, result.routes);
    result.shared_resources = _path_planner.options().shared_resources;
    extract_navigation_annotations(result, tasks);
  }
  return result;
}

} // namespace capability_mission_planner::offline
