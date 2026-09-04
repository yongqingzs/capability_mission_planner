#include <capability_mission_planner/offline_planner_config.hpp>

#include <chrono>
#include <iostream>
#include <stdexcept>

using namespace capability_mission_planner::offline;

int main(int argc, char* argv[]) {
  try {
    if (argc < 2 || argc > 3) {
      std::cerr << "usage: capability_mission_planner_cli CONFIG.yaml [OUTPUT_DIRECTORY]\n";
      return 2;
    }
    const auto total_start = std::chrono::steady_clock::now();
    auto request = OfflinePlannerConfigLoader::load(argv[1]);
    const auto loaded_at = std::chrono::steady_clock::now();
    if (argc == 3) request.output_directory = std::filesystem::absolute(argv[2]);

    MultiMapPathPlanner path_planner(request.bundle, request.traversal);
    OfflineMissionPlanner planner(std::move(path_planner), request.objective);
    const auto plan = planner.plan(
      request.robots, request.tasks, request.coordinate_conflicts);
    const auto planned_at = std::chrono::steady_clock::now();
    PlanExporter::write(request.output_directory, *request.bundle,
      request.robots, request.tasks, plan, request.export_options);
    const auto exported_at = std::chrono::steady_clock::now();
    const auto seconds = [](const auto begin, const auto end) {
      return std::chrono::duration<double>(end - begin).count();
    };

    std::cout << "planning complete\n"
              << "  maps: " << request.bundle->maps.size() << '\n'
              << "  robots: " << request.robots.size() << '\n'
              << "  tasks: " << request.tasks.size() << '\n'
              << "  maximum_load_seconds: "
              << plan.maximum_load_ticks * plan.time_step_seconds << '\n'
              << "  total_load_seconds: "
              << plan.total_load_ticks * plan.time_step_seconds << '\n'
              << "  allocation_estimate_requests: "
              << plan.allocation_path_stats.estimate_requests << '\n'
              << "  allocation_grid_searches: "
              << plan.allocation_path_stats.grid_searches << '\n'
              << "  total_grid_searches: "
              << plan.total_path_stats.grid_searches << '\n'
              << "  timing_load_and_parse_seconds: "
              << seconds(total_start, loaded_at) << '\n'
              << "  timing_planning_seconds: "
              << seconds(loaded_at, planned_at) << '\n'
              << "  timing_export_seconds: "
              << seconds(planned_at, exported_at) << '\n'
              << "  timing_total_seconds: "
              << seconds(total_start, exported_at) << '\n'
              << "  output: " << request.output_directory << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "planning failed: " << error.what() << '\n';
    return 1;
  }
}
