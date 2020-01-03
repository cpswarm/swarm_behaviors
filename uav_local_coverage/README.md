# uav_local_coverage

This package performs local coverage with an unmanned aerial vehicle (UAV). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_position

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor (only if `single_target=true`)

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
Run the launch file
```
roslaunch uav_local_coverage uav_local_coverage.launch
```
to launch the `uav_local_coverage` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `uav_local_coverage.yaml` that allows to configure the behavior of the `uav_local_coverage` node.

## Nodes

### uav_local_coverage
The `uav_local_coverage` performs coverage locally around the current position of the UAV. This is achieved by generating a spiral movement pattern according to the [circle involute](http://mathworld.wolfram.com/CircleInvolute.html). The shape of the circle involute is computed based on the characteristics of the camera of the UAV. It is computed in such a way that a downward facing camera completely covers the area around the current position of the UAV. The UAV follows this path for a predefined number of steps and then aborts the coverage. When the parameter `single_target` is set to `true`, it succeeds once a target has been found and returns the target ID and position.

#### Action Goal
* `uav_local_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  An empty goal that starts the local coverage behavior.

#### Action Result
* `uav_local_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed when `single_target` is set to `true`.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: `true`)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~altitude` (real, default: `5.0`)
  The altitude above ground at which the UAV operates. It is used to compute the path of the UAV.
* `~fov_hor` (real, default: `1.236`)
  Horizontal camera field of view in radian. It is used to compute the path of the UAV.
* `~fov_ver` (real, default: `0.970`)
  Vertical camera field of view in radian. It is used to compute the path of the UAV.
* `~local_steps` (integer, default: `20`)
  Number of steps to do in the local coverage behavior.

## Code API
[uav_local_coverage package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_local_coverage/docs/html/files.html)
