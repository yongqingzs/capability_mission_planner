#include <capability_mission_planner/offline_map_planner.hpp>

#include <capability_mission_planner/search/a_star.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

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
  const CapabilitySet& capabilities,
  double required_clearance_m,
  double nominal_speed_mps = 0.0)
{
  std::ostringstream output;
  output << position_key(start) << '>' << position_key(goal) << '|';
  for (const auto& capability : capabilities)
    output << capability << ',';
  output << "|clearance=" << required_clearance_m;
  output << "|speed=" << nominal_speed_mps;
  return output.str();
}

class GridEnvironment {
public:
  GridEnvironment(const MapLayer& map, GridPosition goal, int move_ticks,
    int diagonal_ticks, double required_clearance_m, double obstacle_cost_weight,
    bool allow_diagonal)
  : _map(map), _goal(std::move(goal)), _move_ticks(move_ticks),
    _diagonal_ticks(diagonal_ticks), _required_clearance_m(required_clearance_m),
    _obstacle_cost_weight(obstacle_cost_weight), _allow_diagonal(allow_diagonal) {}

  int admissibleHeuristic(const GridPosition& state) const {
    const int diagonal = std::min(std::abs(state.x - _goal.x), std::abs(state.y - _goal.y));
    const int straight = std::abs(state.x - _goal.x) + std::abs(state.y - _goal.y) - 2 * diagonal;
    return diagonal * _diagonal_ticks + straight * _move_ticks;
  }
  bool isSolution(const GridPosition& state) const { return state == _goal; }
  void getNeighbors(
    const GridPosition& state,
    std::vector<search::Neighbor<GridPosition, GridAction, int>>& neighbors) const
  {
    static constexpr std::array<std::array<int, 2>, 8> offsets{{
      {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}},
      {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
    for (const auto& offset : offsets) {
      const bool diagonal = offset[0] != 0 && offset[1] != 0;
      if (diagonal && !_allow_diagonal) continue;
      GridPosition next{state.map_id, state.x + offset[0], state.y + offset[1]};
      if (!_map.is_traversable(next.x, next.y) ||
        _map.clearance(next.x, next.y) < _required_clearance_m) continue;
      if (diagonal && (!_map.is_traversable(state.x + offset[0], state.y) ||
        !_map.is_traversable(state.x, state.y + offset[1]))) continue;
      const int move_ticks = diagonal ? _diagonal_ticks : _move_ticks;
      const int penalty = static_cast<int>(std::lround(
        _obstacle_cost_weight * move_ticks * _map.cost(next.x, next.y) / 252.0));
      neighbors.emplace_back(next, GridAction{}, std::max(1, move_ticks + penalty));
    }
  }
  void onExpandNode(const GridPosition&, int, int) const {}
  void onDiscover(const GridPosition&, int, int) const {}

private:
  const MapLayer& _map;
  GridPosition _goal;
  int _move_ticks;
  int _diagonal_ticks;
  double _required_clearance_m;
  double _obstacle_cost_weight;
  bool _allow_diagonal;
};

struct Previous {
  std::size_t node = 0;
  bool transition = false;
  std::string transition_id;
  int transition_ticks = 0;
};

} // namespace

namespace {

std::shared_ptr<const MultiMapBundle> make_coarse_bundle(
  const std::shared_ptr<const MultiMapBundle>& source, unsigned int factor)
{
  if (factor <= 1U) return source;
  auto result = std::make_shared<MultiMapBundle>();
  result->directory = source->directory;
  for (const auto& [id, original] : source->maps) {
    MapLayer map = original;
    map.width = (original.width + static_cast<int>(factor) - 1) /
      static_cast<int>(factor);
    map.height = (original.height + static_cast<int>(factor) - 1) /
      static_cast<int>(factor);
    map.resolution = original.resolution * factor;
    map.traversable.assign(static_cast<std::size_t>(map.width * map.height), 0U);
    map.clearance_m.assign(map.traversable.size(), 0.0F);
    map.inflated_cost.assign(map.traversable.size(), 254U);
    for (int cy = 0; cy < map.height; ++cy) {
      for (int cx = 0; cx < map.width; ++cx) {
        bool free = true;
        float clearance = std::numeric_limits<float>::infinity();
        unsigned char cost = 0U;
        for (unsigned int dy = 0; dy < factor; ++dy) {
          for (unsigned int dx = 0; dx < factor; ++dx) {
            const int x = cx * static_cast<int>(factor) + static_cast<int>(dx);
            const int y = cy * static_cast<int>(factor) + static_cast<int>(dy);
            if (x >= original.width || y >= original.height) continue;
            free = free && original.is_traversable(x, y);
            clearance = std::min(clearance, static_cast<float>(original.clearance(x, y)));
            cost = std::max(cost, original.cost(x, y));
          }
        }
        const auto index = static_cast<std::size_t>(cy * map.width + cx);
        map.traversable[index] = free ? 1U : 0U;
        map.clearance_m[index] = std::isfinite(clearance) ? clearance : 0.0F;
        map.inflated_cost[index] = free ? cost : 254U;
      }
    }
    result->maps.emplace(id, std::move(map));
  }
  return result;
}

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
  return plan(start, goal, capabilities, 0.0);
}

MultiMapPath MultiMapPathPlanner::plan(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities,
  double required_clearance_m) const
{
  return plan(start, goal, capabilities, required_clearance_m, 0.0);
}

MultiMapPath MultiMapPathPlanner::plan(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities,
  double required_clearance_m,
  double nominal_speed_mps) const
{
  if (required_clearance_m < 0.0)
    throw std::invalid_argument("required clearance must be non-negative");
  const double speed = nominal_speed_mps > 0.0 ? nominal_speed_mps : _options.nominal_speed_mps;
  if (!(speed > 0.0)) throw std::invalid_argument("nominal speed must be positive");
  if (!_bundle->traversable(start) || !_bundle->traversable(goal))
    throw std::invalid_argument("path endpoints must be traversable");
  const auto full_key = cache_key(start, goal, capabilities, required_clearance_m, speed);
  const auto full_cached = _cache.find(full_key);
  if (full_cached != _cache.end())
    return full_cached->second;

  const auto same_map_path = [&](const GridPosition& from, const GridPosition& to) {
    const std::string key = position_key(from) + '>' + position_key(to) +
      "|clearance=" + std::to_string(required_clearance_m) +
      "|speed=" + std::to_string(speed);
    const auto cached = _segment_cache.find(key);
    if (cached != _segment_cache.end())
      return cached->second;
    const auto& map = _bundle->map(from.map_id);
    const int move_ticks = seconds_to_ticks(
      map.resolution / speed, _options.time_step_seconds);
    const int diagonal_ticks = seconds_to_ticks(
      std::sqrt(2.0) * map.resolution / speed,
      _options.time_step_seconds);
    if (map.clearance(from.x, from.y) < required_clearance_m ||
      map.clearance(to.x, to.y) < required_clearance_m)
      throw std::runtime_error("path endpoint has insufficient clearance");
    GridEnvironment environment(map, to, move_ticks, diagonal_ticks,
      required_clearance_m, _options.obstacle_cost_weight, _options.allow_diagonal);
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
    if (_options.downsample_costmap && _options.coarse_search_factor > 1U) {
      if (!_coarse_bundle)
        _coarse_bundle = make_coarse_bundle(_bundle, _options.coarse_search_factor);
      if (!_coarse_planner) {
        TraversalOptions coarse_options = _options;
        coarse_options.downsample_costmap = false;
        coarse_options.coarse_search_factor = 1U;
        _coarse_planner = std::make_shared<MultiMapPathPlanner>(
          _coarse_bundle, coarse_options);
      }
      const auto coarse_start = GridPosition{start.map_id,
        start.x / static_cast<int>(_options.coarse_search_factor),
        start.y / static_cast<int>(_options.coarse_search_factor)};
      const auto coarse_goal = GridPosition{goal.map_id,
        goal.x / static_cast<int>(_options.coarse_search_factor),
        goal.y / static_cast<int>(_options.coarse_search_factor)};
      const auto coarse_path = _coarse_planner->plan(
        coarse_start, coarse_goal, capabilities, required_clearance_m, speed);
      MultiMapPath output;
      output.travel_ticks = coarse_path.travel_ticks;
      const auto expand = [&](const GridPosition& position) {
        const int factor = static_cast<int>(_options.coarse_search_factor);
        const int x = std::min(position.x * factor + factor / 2, _bundle->map(position.map_id).width - 1);
        const int y = std::min(position.y * factor + factor / 2, _bundle->map(position.map_id).height - 1);
        return GridPosition{position.map_id, x, y};
      };
      for (const auto& step : coarse_path.steps)
        output.steps.push_back({expand(step.position), step.arrival_tick, step.transition_id});
      _cache.emplace(full_key, output);
      return output;
    }
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
    if (_bundle->map(transition.from_cell.map_id).clearance(
        transition.from_cell.x, transition.from_cell.y) < required_clearance_m ||
      _bundle->map(transition.to_cell.map_id).clearance(
        transition.to_cell.x, transition.to_cell.y) < required_clearance_m)
      continue;
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

int MultiMapPathPlanner::estimate_distance(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities,
  double required_clearance_m,
  double nominal_speed_mps) const
{
  if (!_options.downsample_costmap || _options.coarse_search_factor <= 1U ||
    start.map_id != goal.map_id)
    return plan(start, goal, capabilities, required_clearance_m,
      nominal_speed_mps).travel_ticks;
  if (!_coarse_bundle)
    _coarse_bundle = make_coarse_bundle(_bundle, _options.coarse_search_factor);
  const auto coarse_position = [&](const GridPosition& position) {
    return GridPosition{position.map_id,
      position.x / static_cast<int>(_options.coarse_search_factor),
      position.y / static_cast<int>(_options.coarse_search_factor)};
  };
  try {
    if (!_coarse_planner) {
      auto coarse_options = _options;
      coarse_options.downsample_costmap = false;
      coarse_options.coarse_search_factor = 1U;
      _coarse_planner = std::make_shared<MultiMapPathPlanner>(_coarse_bundle, coarse_options);
    }
    return _coarse_planner->plan(coarse_position(start), coarse_position(goal),
      capabilities, required_clearance_m, nominal_speed_mps).travel_ticks;
  } catch (const std::runtime_error&) {
    return distance(start, goal, capabilities, required_clearance_m);
  }
}

int MultiMapPathPlanner::distance(
  const GridPosition& start,
  const GridPosition& goal,
  const CapabilitySet& capabilities,
  double required_clearance_m) const
{
  return plan(start, goal, capabilities, required_clearance_m).travel_ticks;
}

} // namespace capability_mission_planner::offline
