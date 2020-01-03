^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swarm_behaviors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2020-01-03)
------------------
* Changed: Rename position library to swarm_behaviors_position
* Changed: Rename velocity library to swarm_behaviors_velocity

1.2.0 (2019-12-29)
------------------
* First public release
* Added: Package position
* Added: Package velocity
* Added: Package uav_optimal_coverage
* Added: Package ugv_random_walk
* Changed: Create stack for swarm_behaviors library
* Changed: Refactor library structure
* Fixed: Force C++11
* Changed behavior algorithms: Targets handled by swarm functions library
* Changed behavior algorithms: Check if movement was successful
* Changed coverage algorithms: Return state aborted once coverage finishes
* Changed random coverage algorithms: Read RNG seed from parameter, use random seed otherwise
* Fixed behavior algorithms: Correctly handle state
* Fixed coverage algorithms: Returning of target
* Changed: Split up navigation library, create position library and move parts to abstraction library
* Added to position: Obstacle detection
* Changed position: Move obstacle avoidance to abstraction library
* Changed position: Always use local coordinates
* Fixed position: Goal computation
* Changed velocity: Limit velocity for small distances
* Changed uav_optimal_coverage: Use position setpoint
* Changed uav_optimal_coverage: Use current position to determine next waypoint
* Fixed uav_optimal_coverage: Stop moving when behavior finishes
* Fixed uav_optimal_coverage: Completely traverse path
* Changed uav_random_direction: UAVs depart in different directions
* Fixed uav_random_direction: Change direction when obstacle detected
* Fixed uav_simple_tracking: Handling of target rescued event
* Contributors: Micha Sende, Omar Morando

1.1.0 (2018-11-12)
------------------
* Changed navigation: Store yaw in pose
* Changed navigation: Keep UAVs at same altitude
* Changed navigation: Restructure navigation library
* Fixed uav_random_direction: Computation of sector occupied by obstacles and other UAVs
* Contributors: Micha Sende

1.0.0 (2018-10-30)
------------------
* Initial release
* Contributors: Micha Sende
