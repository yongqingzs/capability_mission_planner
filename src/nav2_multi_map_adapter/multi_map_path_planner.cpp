#include <capability_mission_planner/offline_map_planner.hpp>

#include <capability_mission_planner/search/a_star.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace capability_mission_planner::offline {
namespace {

struct GridPositionHash {
  std::size_t operator()(const GridPosition& position) const noexcept {
    std::size_t value = std::hash<std::string>{}(position.map_id);
    value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.x)) +
      0x9e3779b9U + (value << 6U) + (value >> 2U);
    value ^= static_cast<std::size_t>(static_cast<unsigned int>(position.y)) +
      0x9e3779b9U + (value << 6U) + (value >> 2U);
    return value;
  }
};

struct GridAction {};

bool contains_all(const CapabilitySet& available, const CapabilitySet& required) {
  return std::all_of(required.begin(), required.end(), [&](const auto& capability) {
    return available.count(capability) != 0U;
  });
}

int seconds_to_ticks(double seconds, double time_step) {
  return std::max(1, static_cast<int>(std::ceil(seconds / time_step - 1e-9)));
}

double transition_seconds(const MapTransition& transition, const TraversalOptions& options) {
  const auto configured = options.transition_seconds.find(transition.type);
  return (configured == options.transition_seconds.end()
      ? options.default_transition_seconds
      : configured->second) + options.map_switch_seconds;
}

std::string position_key(const GridPosition& position) {
  return position.map_id + ':' + std::to_string(position.x) + ':' +
    std::to_string(position.y);
}

std::string cache_key(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities)
{
  std::ostringstream output;
  output << position_key(start) << '>' << position_key(goal) << '|';
  for (const auto& capability : capabilities)
    output << capability << ',';
  return output.str();
}

class GridEnvironment {
public:
  GridEnvironment(const MapLayer& map, GridPosition goal, int move_ticks)
  : _map(map), _goal(std::move(goal)), _move_ticks(move_ticks) {}

  int admissibleHeuristic(const GridPosition& state) const {
    return (std::abs(state.x - _goal.x) + std::abs(state.y - _goal.y)) *
      _move_ticks;
  }
  bool isSolution(const GridPosition& state) const { return state == _goal; }
  void getNeighbors(
    const GridPosition& state,
    std::vector<search::Neighbor<GridPosition, GridAction, int>>& neighbors) const
  {
    static constexpr std::array<std::array<int, 2>, 4> offsets{{
      {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}}};
    for (const auto& offset : offsets) {
      GridPosition next{state.map_id, state.x + offset[0], state.y + offset[1]};
      if (_map.is_traversable(next.x, next.y))
        neighbors.emplace_back(next, GridAction{}, _move_ticks);
    }
  }
  void onExpandNode(const GridPosition&, int, int) const {}
  void onDiscover(const GridPosition&, int, int) const {}

private:
  const MapLayer& _map;
  GridPosition _goal;
  int _move_ticks;
};

struct Previous {
  std::size_t node = 0;
  bool transition = false;
  std::string transition_id;
  int transition_ticks = 0;
};

} // namespace

MultiMapPathPlanner::MultiMapPathPlanner(
  std::shared_ptr<const MultiMapBundle> bundle,
  TraversalOptions options)
: _bundle(std::move(bundle)), _options(std::move(options))
{
  if (!_bundle)
    throw std::invalid_argument("multi-map bundle must not be null");
  if (!(_options.time_step_seconds > 0.0) || !(_options.nominal_speed_mps > 0.0))
    throw std::invalid_argument("time step and nominal speed must be positive");
}

MultiMapPath MultiMapPathPlanner::plan(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities) const
{
  if (!_bundle->traversable(start) || !_bundle->traversable(goal))
    throw std::invalid_argument("path endpoints must be traversable");
  const auto full_key = cache_key(start, goal, capabilities);
  const auto full_cached = _cache.find(full_key);
  if (full_cached != _cache.end())
    return full_cached->second;

  const auto same_map_path = [&](const GridPosition& from, const GridPosition& to) {
    const std::string key = position_key(from) + '>' + position_key(to);
    const auto cached = _segment_cache.find(key);
    if (cached != _segment_cache.end())
      return cached->second;
    const auto& map = _bundle->map(from.map_id);
    const int move_ticks = seconds_to_ticks(
      map.resolution / _options.nominal_speed_mps, _options.time_step_seconds);
    GridEnvironment environment(map, to, move_ticks);
    search::AStar<GridPosition, GridAction, int, GridEnvironment, GridPositionHash>
      grid_search(environment);
    search::PlanResult<GridPosition, GridAction, int> result;
    if (!grid_search.search(from, result))
      throw std::runtime_error("no path within " + from.map_id);
    MultiMapPath path;
    path.travel_ticks = result.cost;
    for (const auto& [position, arrival] : result.states)
      path.steps.push_back({position, arrival, {}});
    _segment_cache.emplace(key, path);
    return path;
  };

  if (start.map_id == goal.map_id) {
    auto result = same_map_path(start, goal);
    _cache.emplace(full_key, result);
    return result;
  }

  std::vector<GridPosition> nodes{start, goal};
  const auto add_node = [&](const GridPosition& position) {
    if (std::find(nodes.begin(), nodes.end(), position) == nodes.end())
      nodes.push_back(position);
  };
  for (const auto& transition : _bundle->transitions) {
    const auto requirement = _options.transition_requirements.find(transition.type);
    if (requirement != _options.transition_requirements.end() &&
      !contains_all(capabilities, requirement->second))
    {
      continue;
    }
    add_node(transition.from_cell);
    add_node(transition.to_cell);
  }

  const auto node_index = [&](const GridPosition& position) {
    const auto found = std::find(nodes.begin(), nodes.end(), position);
    return found == nodes.end() ? nodes.size()
      : static_cast<std::size_t>(std::distance(nodes.begin(), found));
  };
  std::vector<int> distance(nodes.size(), std::numeric_limits<int>::max());
  std::vector<Previous> previous(nodes.size());
  std::vector<bool> has_previous(nodes.size(), false);
  std::vector<bool> visited(nodes.size(), false);
  distance[0] = 0;

  for (std::size_t iteration = 0; iteration < nodes.size(); ++iteration) {
    std::size_t current = nodes.size();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (!visited[i] && distance[i] != std::numeric_limits<int>::max() &&
        (current == nodes.size() || distance[i] < distance[current]))
      {
        current = i;
      }
    }
    if (current == nodes.size() || current == 1U)
      break;
    visited[current] = true;

    for (std::size_t next = 0; next < nodes.size(); ++next) {
      if (next == current || nodes[next].map_id != nodes[current].map_id)
        continue;
      try {
        const auto segment = same_map_path(nodes[current], nodes[next]);
        const int candidate = distance[current] + segment.travel_ticks;
        if (candidate < distance[next]) {
          distance[next] = candidate;
          previous[next] = Previous{current, false, {}, 0};
          has_previous[next] = true;
        }
      } catch (const std::runtime_error&) {
      }
    }
    for (const auto& transition : _bundle->transitions) {
      if (transition.from_cell != nodes[current])
        continue;
      const auto requirement = _options.transition_requirements.find(transition.type);
      if (requirement != _options.transition_requirements.end() &&
        !contains_all(capabilities, requirement->second))
      {
        continue;
      }
      const auto next = node_index(transition.to_cell);
      if (next == nodes.size())
        continue;
      const int transition_ticks = seconds_to_ticks(
        transition_seconds(transition, _options), _options.time_step_seconds);
      const int candidate = distance[current] + transition_ticks;
      if (candidate < distance[next]) {
        distance[next] = candidate;
        previous[next] = Previous{current, true, transition.id, transition_ticks};
        has_previous[next] = true;
      }
    }
  }

  if (distance[1] == std::numeric_limits<int>::max())
    throw std::runtime_error("no path from " + start.map_id + " to " + goal.map_id);

  std::vector<std::size_t> chain{1U};
  while (chain.back() != 0U) {
    if (!has_previous[chain.back()])
      throw std::logic_error("broken multi-map path predecessor chain");
    chain.push_back(previous[chain.back()].node);
  }
  std::reverse(chain.begin(), chain.end());

  MultiMapPath output;
  output.steps.push_back({start, 0, {}});
  int elapsed = 0;
  for (std::size_t i = 1; i < chain.size(); ++i) {
    const auto to = chain[i];
    const auto from = chain[i - 1U];
    const auto& edge = previous[to];
    if (edge.transition) {
      elapsed += edge.transition_ticks;
      output.steps.push_back({nodes[to], elapsed, edge.transition_id});
    } else {
      const auto segment = same_map_path(nodes[from], nodes[to]);
      for (std::size_t step = 1; step < segment.steps.size(); ++step) {
        output.steps.push_back({segment.steps[step].position,
          elapsed + segment.steps[step].arrival_tick, {}});
      }
      elapsed += segment.travel_ticks;
    }
  }
  output.travel_ticks = elapsed;
  _cache.emplace(full_key, output);
  return output;
}

int MultiMapPathPlanner::distance(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities) const
{
  return plan(start, goal, capabilities).travel_ticks;
}

} // namespace capability_mission_planner::offline
