#ifndef UAV_OPTIMAL_COVERAGE_H
#define UAV_OPTIMAL_COVERAGE_H

#include "cpswarm_msgs/GetWaypoint.h"
#include "uav_coverage.h"

/**
 * @brief An implementation of the coverage class that allows to cover a given area optimally with a swarm of cyber physical systems (CPSs). The area is divided among the CPSs and each CPS covers its part by simple back and forth (boustrophedon) motion.
 */
class uav_optimal_coverage : public uav_coverage
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    uav_optimal_coverage ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_optimal_coverage ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step();

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

    /**
     * @brief Target velocity of the UAV.
     */
    double target_velocity;

    /**
     * The distance to the current waypoint below which the next waypoint of the path is selected.
     */
    double tolerance;

};

#endif // UAV_OPTIMAL_COVERAGE_H

