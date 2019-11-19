# uav_flocking

This package performs coverage and tracking with a flock of unmanned aerial vehicles (UAVs). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* position
* velocity

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* coverage_path
* target_monitor

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider

Further required packages are:
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
There are two launch files, one for coverage and one for tracking. Run the launch file
```
roslaunch uav_flocking uav_flocking_coverage.launch
```
to launch the `uav_flocking_coverage` node. Run the launch file
```
roslaunch uav_flocking uav_flocking_tracking.launch
```
to launch the `uav_flocking_tracking` node.

The launch files can be configured with following parameters:
* `id` (integer, default: 1)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `uav_flocking.yaml` that allows to configure the behavior of the both nodes.

## Nodes

### uav_flocking_coverage
The `uav_flocking_coverage` performs coverage with a swarm of UAVs. The UAVs stay in a flock while sweeping the area using simple back and forth (boustrophedon) motion. Once the region has been sweeped completely, the UAVs abort the coverage. When the parameter `single_target` is set to `true`, a UAV succeeds once a target has been found and returns the target ID and position. The coverage path generation is provided by the [swarm functions library](https://github.com/cpswarm/swarm_functions/).

#### Action Goal
* `uav_coverage/goal` ([cpswarm_msgs/CoverageGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  An empty goal that starts the flocking coverage behavior.

#### Action Result
* `uav_coverage/result` ([cpswarm_msgs/CoverageResult](https://cpswarm.github.io/cpswarm_msgs/html/action/Coverage.html))
  ID and position of the target that has been found.

#### Subscribed Topics
* `target_found` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position and ID of a target detected by the target monitor. Only subscribed when `single_target` is set to `true`.
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative positions of the other swarm members.
* `swarm_velocity_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative velocities of the other swarm members.

#### Services Called
* `coverage_path/waypoint` ([cpswarm_msgs/GetWaypoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetWaypoint.html))
  Get the current waypoint to navigate to.
* `area/get_area` ([cpswarm_msgs/GetPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoints.html))
  Get the area boundaries.
* `area/get_center` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the center of the area.

#### Parameters
* `~loop_rate` (real, default: 5.0)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 1)
  The size of the message queue used for publishing and subscribing to topics.
* `~single_target` (boolean, default: true)
  Whether the algorithm will succeed / terminate once a target has been found.
* `~equi_dist` (real, default: 10.0)
  Equilibrium distance between CPSs in meter.
* `~flock_vel` (real, default: 0.5)
  Target velocity of the flock in meter per second.
* `~form_vel` (real, default: 0.5)
  Maximum velocity during formation flight in meter per second.
* `~repulse_spring` (real, default: 1.0)
  Repulsion spring constant of half-spring per square second.
* `~repulse_max` (real, default: 1.0)
  Maximum repulsion between CPSs in meter in order to avoid over excitation.
* `~align_frict` (real, default: 20.0)
  Velocity alignment viscous friction coefficient in square meter per second. Higher values result in slower but more stable flocks.
* `~align_slope` (real, default: 1.0)
  Constant slope around equilibrium distance in meter.
* `~align_min` (real, default: 1.0)
  Minimum alignment between CPS in meter in order to avoid over excitation.
* `~wall_frict` (real, default: 20.0)
  Bounding area viscous friction coefficient in square meter per second.
* `~wall_decay` (real, default: 1.0)
  Softness of wall as decay width in meter.
* `~form_shape` (real, default: 1.0)
  Strength of the shape forming velocity component in formation flight.
* `~form_track` (real, default: 1.0)
  Strength of the tracking velocity component in formation flight.
* `~accel_time` (real, default: 1.0)
  Characteristic time needed by the CPS to reach the target velocity in seconds.

### uav_flocking_tracking
The `uav_flocking_tracking` performs tracking with a swarm of UAVs. The UAVs stay in a formation while tracking a target. The position of the target is updated by the target monitor from the [swarm functions library](https://github.com/cpswarm/swarm_functions/). When the target is lost, i.e., the target is not in the camera field of view anymore, the tracking aborts. When the target is done, i.e., handled by another CPS, the tracking succeeds.

#### Action Goal
* `uav_tracking/goal` ([cpswarm_msgs/TrackingGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Tracking.html))
  A goal that starts the tracking behavior. It contains the ID of the target to track.

#### Subscribed Topics
* `target_update` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position updates of the target being tracked.
* `target_lost` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been lost.
* `target_done` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been handled by another CPS.
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative positions of the other swarm members
* `swarm_velocity_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative velocities of the other swarm members.

#### Services Called
* `area/get_area` ([cpswarm_msgs/GetPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoints.html))
  Get the area boundaries.
* `area/get_center` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the center of the area.

#### Parameters
* `~loop_rate` (real, default: 5.0)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 1)
  The size of the message queue used for publishing and subscribing to topics.
* `~equi_dist` (real, default: 10.0)
  Equilibrium distance between CPSs in meter.
* `~flock_vel` (real, default: 0.5)
  Target velocity of the flock in meter per second.
* `~form_vel` (real, default: 0.5)
  Maximum velocity during formation flight in meter per second.
* `~repulse_spring` (real, default: 1.0)
  Repulsion spring constant of half-spring per square second.
* `~repulse_max` (real, default: 1.0)
  Maximum repulsion between CPSs in meter in order to avoid over excitation.
* `~align_frict` (real, default: 20.0)
  Velocity alignment viscous friction coefficient in square meter per second. Higher values result in slower but more stable flocks.
* `~align_slope` (real, default: 1.0)
  Constant slope around equilibrium distance in meter.
* `~align_min` (real, default: 1.0)
  Minimum alignment between CPS in meter in order to avoid over excitation.
* `~wall_frict` (real, default: 20.0)
  Bounding area viscous friction coefficient in square meter per second.
* `~wall_decay` (real, default: 1.0)
  Softness of wall as decay width in meter.
* `~form_shape` (real, default: 1.0)
  Strength of the shape forming velocity component in formation flight.
* `~form_track` (real, default: 1.0)
  Strength of the tracking velocity component in formation flight.
* `~accel_time` (real, default: 1.0)
  Characteristic time needed by the CPS to reach the target velocity in seconds.

## Code API
[uav_flocking package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_flocking/docs/html/files.html)
