# swarm_behaviors_flocking
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__swarm_behaviors_flocking__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__swarm_behaviors_flocking__ubuntu_xenial__source/)

This package provides functionalities for flocking and formation flight. It is a library package of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* position
* velocity

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* kinematics_exchanger

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Libraries

### flocking
The `flocking` library provides functionalities for flocking and formation flight. It calculates the required velocity for a CPS to either stay in a flock or to create a specific formation with a swarm of CPSs.

#### Subscribed Topics
* `swarm_position_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative positions of the other swarm members
* `swarm_position` ([cpswarm_msgs/ArrayOfPositions](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfPositions.html))
  Absolute positions of the other swarm members.
* `swarm_velocity_rel` ([cpswarm_msgs/ArrayOfVectors](https://cpswarm.github.io/cpswarm_msgs/html/msg/ArrayOfVectors.html))
  Relative velocities of the other swarm members.

#### Services Called
* `area/get_center` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the center of the area.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~start_delay` (real, default: `1.0`)
  The Time in seconds to wait during startup for other CPSs.
* `~equi_dist` (real, default: `10.0`)
  Equilibrium distance between CPSs in meter.
* `~repulse_spring` (real, default: `1.0`)
  Repulsion spring constant of half-spring per square second.
* `~repulse_max` (real, default: `1.0`)
  Maximum repulsion between CPSs in meter in order to avoid over excitation.
* `~align_frict` (real, default: `20.0`)
  Velocity alignment viscous friction coefficient in square meter per second. Higher values result in slower but more stable flocks.
* `~align_slope` (real, default: `1.0`)
  Constant slope around equilibrium distance in meter.
* `~align_min` (real, default: `1.0`)
  Minimum alignment between CPS in meter in order to avoid over excitation.
* `~wall_frict` (real, default: `20.0`)
  Bounding area viscous friction coefficient in square meter per second.
* `~wall_decay` (real, default: `1.0`)
  Softness of wall as decay width in meter.
* `~accel_time` (real, default: `1.0`)
  Characteristic time needed by the CPS to reach the target velocity in seconds.
* `~flocking/flock_vel` (real, default: `0.5`)
  Target velocity of the flock in meter per second.
* `~formation/formation` (string)
  The type of formation to create. Can be grid, ring, or line.
* `~formation/form_vel` (real, default: `0.5`)
  Maximum velocity during formation flight in meter per second.
* `~formation/form_shape` (real, default: `1.0`)
  Strength of the shape forming velocity component in formation flight.
* `~formation/form_track` (real, default: `1.0`)
  Strength of the tracking velocity component in formation flight.
* `~formation/form_decay` (real, default: `1.0`)
  Softness of shape in meter.

## Code API
[swarm_behaviors_flocking package code API documentation](https://cpswarm.github.io/swarm_behaviors/lib/swarm_behaviors_flocking/docs/html/files.html)
