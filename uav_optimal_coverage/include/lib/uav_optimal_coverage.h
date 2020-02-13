#ifndef UAV_OPTIMAL_COVERAGE_H
#define UAV_OPTIMAL_COVERAGE_H

#include <geometry_msgs/Pose.h>
#include <cpswarm_msgs/GetWaypoint.h>
#include <position.h>

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of the behavior algorithm.
 */
typedef enum {
    STATE_ACTIVE = 0,
    STATE_SUCCEEDED,
    STATE_ABORTED
} behavior_state_t;

/**
 * @brief A class that allows to cover a given area optimally with a swarm of cyber physical systems (CPSs). The area is divided among the CPSs and each CPS covers its part by simple back and forth (boustrophedon) motion.
 */
class uav_optimal_coverage
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude The altitude at which the CPS operates.
     */
    uav_optimal_coverage (double altitude);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_optimal_coverage ();

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
     * @brief A helper object for position related tasks.
     */
    position pos;

};

#endif // UAV_OPTIMAL_COVERAGE_H

