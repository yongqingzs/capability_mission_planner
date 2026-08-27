#include <capability_mission_planner/offline_map_planner.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
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
    std::cout << "offline adapter tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "offline adapter tests failed: " << error.what() << '\n';
    return 1;
  }
}
