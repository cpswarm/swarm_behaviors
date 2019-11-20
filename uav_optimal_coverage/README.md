# uav_optimal_coverage

This package performs coverage with a swarm of unmanned aerial vehicles (UAVs). The UAVs optimally divide the area to be covered among each other. It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* position
* velocity

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* area_division
* coverage_path
* target_monitor (only if `single_target=true`)

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
Run the launch file
```
roslaunch uav_optimal_coverage uav_optimal_coverage.launch
```
to launch the `uav_optimal_coverage` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `uav_optimal_coverage.yaml` that allows to configure the behavior of the `uav_optimal_coverage` node.

## Nodes

### uav_optimal_coverage
The `uav_optimal_coverage` performs coverage with a swarm of UAVs. The coverage is optimal in a sense that the area to be covered is divided among the UAVs to avoid overlapping regions to be covered multiple times. Within each region, the assigned UAV sweeps the area using simple back and forth (boustrophedon) motion. Once the region has been sweeped completely, it aborts the coverage. When the parameter `single_target` is set to `true`, it succeeds once a target has been found and returns the target ID and position. The area division and coverage path generation are provided by the [swarm functions library](https://github.com/cpswarm/swarm_functions/).

#### Action Goal
* `uav_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  An empty goal that starts the optimal coverage behavior.

#### Action Result
* `uav_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed when `single_target` is set to `true`.

#### Services Called
* `coverage_path/waypoint` ([cpswarm_msgs/GetWaypoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetWaypoint.html))
  Get the current waypoint to navigate to.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: `true`)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~target_velocity` (real, default: `0.5`)
  Target velocity of the UAV in meter per second.
* `~tolerance` (real, default: `0.5`)
  The distance in meter to the current waypoint below which the next waypoint of the path is selected.

## Code API
[uav_optimal_coverage package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_optimal_coverage/docs/html/files.html)
