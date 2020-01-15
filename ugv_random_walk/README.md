# ugv_random_walk
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__ugv_random_walk__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__ugv_random_walk__ubuntu_xenial__source/)

This package performs random walk coverage with an unmanned ground vehicle (UGV). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_position

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor (only if `single_target=true`)

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider
* obstacle_detection

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)
* [random_numbers](https://wiki.ros.org/random_numbers/)

## Execution
Run the launch file
```
roslaunch ugv_random_walk ugv_random_walk.launch
```
to launch the `ugv_random_walk` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `ugv_random_walk.yaml` that allows to configure the behavior of the `ugv_random_walk` node.

## Nodes

### ugv_random_walk
The `ugv_random_walk` performs coverage using the random walk algorithm. The random walk is a mathematical movement model, where an agent moves straight for a specific distance. Then, it changes its direction randomly into a direction that is clear of obstacles and moves straight again. If it arrives at the environment boundary, it reflects to continue its walk. When the parameter `single_target` is set to `true`, the UGV succeeds once a target has been found and returns the target ID and position.

#### Action Goal
* `ugv_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  A goal that starts the random direction coverage behavior.

#### Action Result
* `ugv_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed when `single_target` is set to `true`.

#### Services Called
* `obstacle_detection/get_clear_sector` ([cpswarm_msgs/GetSector](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetSector.html))
  Get the circular sector that is clear of obstacles.
* `area/closest_bound` ([cpswarm_msgs/ClosestBound](https://cpswarm.github.io/cpswarm_msgs/html/srv/ClosestBound.html))
  Get the coordinates of the closest boundary in order to reflect from it.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: `true`)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~step_size` (real, default: `3.0`)
  The distance in meter that a UGV travels in one step.
* `/rng_seed` (integer, default: `0`)
  The seed used for random number generation. In the default case, a random seed is generated.

## Code API
[ugv_random_walk package code API documentation](https://cpswarm.github.io/swarm_behaviors/ugv_random_walk/docs/html/files.html)
