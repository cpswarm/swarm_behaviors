^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uav_random_direction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2020-01-03)
------------------

1.2.0 (2019-12-29)
------------------
* Changed: Refactor library structure
* Changed: UAVs depart in different directions
* Changed: Read RNG seed from parameter, use random seed otherwise
* Changed: Return state aborted once coverage finishes
* Changed: Check if movement was successful
* Changed: Targets handled by swarm functions library
* Fixed: Change direction when obstacle detected
* Fixed: Correctly handle state of behavior algorithm
* Fixed: Force C++11
* Fixed: Returning of target
* Contributors: Micha Sende

1.1.0 (2018-11-12)
------------------
* Fixed: Computation of sector occupied by obstacles and other UAVs
* Contributors: Micha Sende

1.0.0 (2018-10-30)
------------------
* Initial release of uav_random_direction
* Contributors: Micha Sende
