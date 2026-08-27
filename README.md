# Capability Mission Planner

Capability-aware multi-robot, multi-task mission planning with internal modules
for:

- task booking, priority, label, category, and duration metadata;
- A* obstacle-aware travel costs;
- CBS conflict-free timed schedules.

The planner accepts any number of robots, points, and atomic tasks. A robot may
receive zero, one, or many tasks. Tasks at the same point are merged into one
stop when they are assigned to the same robot.

## Build and test

The planner is self-contained. Adapted source is organized by function under
`include/capability_mission_planner/` and `src/`; its upstream licenses are
retained under `third_party/licenses/`. No RMF workspace or separate planning
project is required at configure, build, or runtime.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run the 4-robot, 20-point, 40-atomic-task example with:

```bash
./build/mission_demo > mission_plan.json
```

Validate planning on a PNG occupancy map and render the result with:

```bash
./build/image_map_demo tmp/image.png tmp/capability_mission_plan.png
```

This optional example uses system OpenCV for PNG input/output and raster
preprocessing. The planning library itself does not depend on OpenCV.

## Planning hierarchy

1. Reject robot-task edges that do not satisfy every required capability.
2. Insert constrained/high-priority tasks first, minimizing maximum route load
   plus weighted total load with exact A* travel costs.
3. Merge same-location tasks assigned to the same robot.
4. Convert each route into ordered milestones and mandatory service waits.
5. Use CBS to remove vertex and opposing-edge conflicts while preserving task
   order.
