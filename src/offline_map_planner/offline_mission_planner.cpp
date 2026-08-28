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
  const MultiMapPathPlanner& planner)
{
  route.travel_ticks = 0;
  route.service_ticks = 0;
  route.segments.clear();
  GridPosition current = robot.start;
  const double clearance = std::max(0.0, robot.clearance_radius_m + robot.safety_margin_m);
  const double speed = robot.nominal_speed_mps > 0.0 ? robot.nominal_speed_mps :
    planner.options().nominal_speed_mps;
  for (const auto& stop : route.stops) {
    auto segment = planner.plan(current, stop.location, robot.capabilities, clearance, speed);
    route.travel_ticks += segment.travel_ticks;
    route.service_ticks += stop.service_ticks;
    route.segments.push_back(std::move(segment));
    current = stop.location;
  }
  if (robot.return_home && !route.stops.empty()) {
    auto segment = planner.plan(current, robot.start, robot.capabilities, clearance, speed);
    route.travel_ticks += segment.travel_ticks;
    route.segments.push_back(std::move(segment));
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
          recompute_route(candidate, robots[robot_index], _path_planner);
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
        recompute_route(without[source], robots[source], _path_planner);
        for (std::size_t target = 0; target < robots.size(); ++target) {
          if (!compatible(robots[target], tasks[task_index]))
            continue;
          for (auto candidate : insertion_options(
              without[target], task_index, tasks[task_index], service_ticks[task_index]))
          {
            try {
              recompute_route(candidate, robots[target], _path_planner);
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
  result.time_step_seconds = time_step;
  for (const auto& route : result.routes) {
    result.maximum_load_ticks = std::max(result.maximum_load_ticks, route.load_ticks());
    result.total_load_ticks += route.load_ticks();
  }
  if (coordinate_conflicts && !robots.empty()) {
    result.schedules = coordinate_multi_map_routes(
      _path_planner, robots, result.routes);
  }
  return result;
}

} // namespace capability_mission_planner::offline
