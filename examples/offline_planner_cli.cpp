#include <capability_mission_planner/offline_planner_config.hpp>

#include <iostream>
#include <stdexcept>

using namespace capability_mission_planner::offline;

int main(int argc, char* argv[]) {
  try {
    if (argc < 2 || argc > 3) {
      std::cerr << "usage: capability_mission_planner_cli CONFIG.yaml [OUTPUT_DIRECTORY]\n";
      return 2;
    }
    auto request = OfflinePlannerConfigLoader::load(argv[1]);
    if (argc == 3) request.output_directory = std::filesystem::absolute(argv[2]);

    MultiMapPathPlanner path_planner(request.bundle, request.traversal);
    OfflineMissionPlanner planner(std::move(path_planner), request.objective);
    const auto plan = planner.plan(
      request.robots, request.tasks, request.coordinate_conflicts);
    PlanExporter::write(request.output_directory, *request.bundle,
      request.robots, request.tasks, plan, request.export_options);

    std::cout << "planning complete\n"
              << "  maps: " << request.bundle->maps.size() << '\n'
              << "  robots: " << request.robots.size() << '\n'
              << "  tasks: " << request.tasks.size() << '\n'
              << "  maximum_load_seconds: "
              << plan.maximum_load_ticks * plan.time_step_seconds << '\n'
              << "  total_load_seconds: "
              << plan.total_load_ticks * plan.time_step_seconds << '\n'
              << "  output: " << request.output_directory << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "planning failed: " << error.what() << '\n';
    return 1;
  }
}
