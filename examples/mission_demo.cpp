#include <capability_mission_planner/mission_planner.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace capability_mission_planner;

namespace {

std::string json_string(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  output << '"';
  return output.str();
}

} // namespace

int main() {
  const std::vector<Robot> robots{
    {"a", {0, 0}, {"fire", "camera"}, true},
    {"b", {11, 0}, {"camera", "thermal"}, true},
    {"c", {0, 11}, {"fire", "thermal"}, true},
    {"d", {11, 11}, {"fire", "camera", "thermal"}, true}};

  std::vector<AtomicTask> tasks;
  for (int i = 0; i < 20; ++i) {
    const Location point{1 + i % 10, 1 + 3 * (i / 10)};
    tasks.push_back(make_task(
      "P" + std::to_string(i) + "-photo", point, {"camera"},
      "gimbal_photo", 1, i % 5 == 0));
    const std::string capability = i % 2 == 0 ? "fire" : "thermal";
    tasks.push_back(make_task(
      "P" + std::to_string(i) + "-" + capability, point, {capability},
      capability, 2));
  }

  MissionPlanner planner(GridPathPlanner(GridMap(12, 12)));
  const auto plan = planner.plan(robots, tasks, true);

  std::cout << "{\n  \"maximum_load\": " << plan.maximum_load
            << ",\n  \"total_load\": " << plan.total_load
            << ",\n  \"routes\": [\n";
  for (std::size_t r = 0; r < plan.routes.size(); ++r) {
    const auto& route = plan.routes[r];
    std::cout << "    {\n      \"robot\": "
              << json_string(robots[route.robot_index].id)
              << ",\n      \"travel_cost\": " << route.travel_cost
              << ",\n      \"service_cost\": " << route.service_cost
              << ",\n      \"stops\": [\n";
    for (std::size_t s = 0; s < route.stops.size(); ++s) {
      const auto& stop = route.stops[s];
      std::cout << "        {\"location\": [" << stop.location.x << ", "
                << stop.location.y << "], \"tasks\": [";
      for (std::size_t t = 0; t < stop.task_indices.size(); ++t) {
        if (t != 0U)
          std::cout << ", ";
        std::cout << json_string(tasks[stop.task_indices[t]].id());
      }
      std::cout << "]}" << (s + 1U == route.stops.size() ? "\n" : ",\n");
    }
    std::cout << "      ],\n      \"schedule\": [\n";
    const auto& schedule = plan.schedules[route.robot_index];
    for (std::size_t s = 0; s < schedule.size(); ++s) {
      const auto& state = schedule[s];
      std::cout << "        {\"t\": " << state.time << ", \"x\": "
                << state.location.x << ", \"y\": " << state.location.y << "}"
                << (s + 1U == schedule.size() ? "\n" : ",\n");
    }
    std::cout << "      ]\n    }"
              << (r + 1U == plan.routes.size() ? "\n" : ",\n");
  }
  std::cout << "  ]\n}\n";
}
