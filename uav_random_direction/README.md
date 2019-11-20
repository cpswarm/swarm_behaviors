# uav_random_direction

This package performs random direction coverage with an unmanned aerial vehicle (UAV). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* position

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor (only if `single_target=true`)

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* obstacle_detection

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)
* [random_numbers](https://wiki.ros.org/random_numbers/)

## Execution
Run the launch file
```
roslaunch uav_random_direction uav_random_direction.launch
```
to launch the `uav_random_direction` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `uav_random_direction.yaml` that allows to configure the behavior of the `uav_random_direction` node.

## Nodes

### uav_random_direction
The `uav_random_direction` performs coverage using the random direction algorithm. The random direction is a mathematical movement model, where an agent moves straight forward until it reaches an obstacle or the environment boundary. There, it changes its direction randomly into a direction that is clear of obstacles. When the parameter `single_target` is set to `true`, the UAV succeeds once a target has been found and returns the target ID and position.

#### Action Goal
* `uav_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  An empty goal that starts the random direction coverage behavior.

#### Action Result
* `uav_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed when `single_target` is set to `true`.

#### Services Called
* `obstacle_detection/get_clear_sector` ([cpswarm_msgs/GetSector](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetSector.html))
  Get the circular sector that is clear of obstacles.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: `true`)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~step_size_max` (real, default: `3.0`)
  The maximum distance in meter that a UAV travels in one step.
* `~step_size_min` (real, default: `1.0`)
  The minimum distance in meter that a UAV travels in one step.
* `/rng_seed` (integer, default: `0`)
  The seed used for random number generation. In the default case, a random seed is generated.

## Code API
[uav_random_direction package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_random_direction/docs/html/files.html)
