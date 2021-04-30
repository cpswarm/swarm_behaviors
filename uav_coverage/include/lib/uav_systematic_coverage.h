#ifndef UAV_SYSTEMATIC_COVERAGE_H
#define UAV_SYSTEMATIC_COVERAGE_H

#include <geometry_msgs/Pose.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include "uav_coverage_behavior.h"

/**
 * @brief A class that allows to cover a given area optimally with a swarm of cyber physical systems (CPSs). The area is divided among the CPSs and each CPS covers its part by simple back and forth (boustrophedon) motion.
 */
class uav_systematic_coverage : public uav_coverage_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude The altitude at which the CPS operates.
     */
    uav_systematic_coverage (double altitude);

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step ();

private:
    /**
     * @brief Service client to get the current waypoint to navigate to.
     */
    ServiceClient wp_getter;

    /**
     * @brief Current waypoint to navigate to.
     */
    geometry_msgs::Point waypoint;

    /**
     * @brief Service message to get the current waypoint.
     */
    cpswarm_msgs::GetWaypoint get_wp;
};

#endif // UAV_SYSTEMATIC_COVERAGE_H
