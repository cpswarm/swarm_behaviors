#ifndef UAV_FLOCKING_COVERAGE_H
#define UAV_FLOCKING_COVERAGE_H

#include <flocking.h>
#include <velocity.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include "uav_coverage_behavior.h"

/**
 * @brief A class that allows to cover a given area with the flocking algorithm.
 */
class uav_flocking_coverage : public uav_coverage_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_flocking_coverage (double altitude);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_flocking_coverage ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step ();

    /**
     * @brief Stop moving.
     */
    void stop ();

private:
    /**
     * @brief A helper object for velocity related tasks.
     */
    velocity vel;

    /**
     * @brief The flocking behavior library object.
     */
    flocking flock;

    /**
     * @brief Service client to get the current waypoint to navigate to.
     */
    ServiceClient wp_getter;

    /**
     * @brief Service message to get the current waypoint.
     */
    cpswarm_msgs::GetWaypoint get_wp;

    /**
     * @brief Current waypoint to navigate to.
     */
    geometry_msgs::Point waypoint;

    /**
     * @brief Target velocity of the flock.
     */
    double flock_vel;
};

#endif // UAV_FLOCKING_COVERAGE_H

