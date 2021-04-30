#ifndef UAV_FLOCKING_TRACKING_H
#define UAV_FLOCKING_TRACKING_H

#include <geometry_msgs/Vector3.h>
#include <flocking.h>
#include <velocity.h>
#include "uav_tracking_behavior.h"

/**
 * @brief A class that allows to track a target by creating a formation with multiple unmanned aerial vehicles (UAVs).
 */
class uav_flocking_tracking : public uav_tracking_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param altitude: The altitude at which the UAV operates.
     */
    uav_flocking_tracking (cpswarm_msgs::TargetPositionEvent target, double altitude);

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @param target Updated target information.
     * @return Return the state of the tracking algorithm.
     */
    behavior_state_t step (cpswarm_msgs::TargetPositionEvent target);

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
};

#endif // UAV_FLOCKING_TRACKING_H
