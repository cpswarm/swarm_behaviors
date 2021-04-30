# uav_coverage
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__uav_coverage__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__uav_coverage__ubuntu_xenial__source/)

This package performs different types of coverage with unmanned aerial vehicles (UAVs). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_flocking
* swarm_behaviors_position
* swarm_behaviors_velocity

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor (only if `single_target=true` or `help_range>0`)
* coverage_path (only if `behavior=flocking` or `behavior=systematic`)


The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider (only if `behavior=random`)
* obstacle_detection (only if `behavior=random`)
* *_pos_provider (only if `help_range>0`)

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)
* [random_numbers](https://wiki.ros.org/random_numbers/)

## Execution
Run the launch file
```
roslaunch uav_coverage uav_coverage.launch
```
to launch the `uav_coverage` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).
* `behavior` (string)
  The behavior to use for coverage. See below for more details.

In the `param` subdirectory there is the parameter file `uav_coverage.yaml` that allows to configure the behavior of the `uav_coverage` node.

## Nodes

### uav_coverage
The `uav_coverage` lets a swarm of UAVs cover the environment. It provides an action server that has three outcomes: `succeeded`, `preempted`, or `aborted`. When the parameter `single_target` is set to `true`, the coverage succeeds once a target has been found and returns the target ID and position. When the parameter `help_range` is greater than zero, the coverage is preempted based on a certain probability when help calls from other CPSs are received.

<!-- TODO: probability: -->

The coverage is performed either individually or cooperatively by employing one of the following algorithms:
* **Flocking**: The UAVs move in a flock through the environment following the rules of cohesion, alignment, and repulsion.
The flock sweeps the environment using simple back and forth (boustrophedon) motion. Once the environment has been sweeped completely, the UAVs abort the coverage. The coverage path generation is provided by the [swarm functions library](https://github.com/cpswarm/swarm_functions/).
* **Local**: A single UAV performs coverage locally around its current position. This is achieved by generating a spiral movement pattern according to the [circle involute](http://mathworld.wolfram.com/CircleInvolute.html). The shape of the circle involute is computed based on the characteristics of the camera of the UAV. It is computed in such a way that a downward facing camera completely covers the area around the current position of the UAV. The UAV follows this path for a predefined number of steps and then aborts the coverage.
* **Random**: The UAVs perform coverage using the random direction algorithm. The random direction is a mathematical movement model, where an agent moves straight forward until it reaches an obstacle or the environment boundary. There, it changes its direction randomly into a direction that is clear of obstacles. The random algorithm does not abort automatically.
* **Systematic**: The UAVs perform coverage cooperatively. The environment to be covered is divided among the UAVs to avoid overlapping regions to be covered multiple times. Within each region, the assigned UAV sweeps the environment using simple back and forth (boustrophedon) motion. Once the region has been sweeped completely, it aborts the coverage. The area division and coverage path generation are provided by the [swarm functions library](https://github.com/cpswarm/swarm_functions/).

#### Action Goal
* `uav_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  A goal that starts the random direction coverage behavior. It contains the altitude at which to operate.

#### Action Result
* `uav_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed if `single_target=true`.
* `target_help` ([cpswarm_msgs/TargetHelp](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetHelp.html))
  Position, ID, and required time of a target where help is needed by another CPS. Only subscribed if `help_range>0`.
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html))
  Position of this UAV. Only subscribed if `help_range>0`.

#### Services Called
* `coverage_path/waypoint` ([cpswarm_msgs/GetWaypoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetWaypoint.html))
  Get the current waypoint to navigate to. Only called if `behavior=flocking` or `behavior=systematic`.
* `area/get_area` ([cpswarm_msgs/GetPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoints.html))
  Get the area polygon. Only called if `behavior=random`.
* `obstacle_detection/get_clear_sector` ([cpswarm_msgs/GetSector](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetSector.html))
  Get the circular sector that is clear of obstacles. Only called if `behavior=random`.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: `true`)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~help_range` (real, default: `0.0`)
  The distance in meter within which help calls of other CPSs are considered.
* `~flocking/flock_vel` (real, default: `0.5`)
  Target velocity of the flock in meter per second.
* `~local/fov_hor` (real, default: `1.236`)
  Horizontal camera field of view in radian. It is used to compute the path of the UAV.
* `~local/fov_ver` (real, default: `0.970`)
  Vertical camera field of view in radian. It is used to compute the path of the UAV.
* `~local/steps` (integer, default: `20`)
  Number of steps to do in the local coverage behavior.
* `~random/margin` (real, default: `0.5`)
  The distance in meter to keep to the environment boundary.
* `/rng_seed` (integer, default: `0`)
  The seed used for random number generation. In the default case, a random seed is generated.

## Code API
[uav_coverage package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_coverage/docs/html/files.html)
