#ifndef UAV_SIMPLE_TRACKING_H
#define UAV_SIMPLE_TRACKING_H

#include "uav_tracking_behavior.h"

/**
 * @brief A class that allows to track a target by minimizing the offset between the unmanned aerial vehicle (UAV) and the target.
 */
class uav_simple_tracking : public uav_tracking_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param altitude: The altitude at which the UAV operates.
     */
    uav_simple_tracking (cpswarm_msgs::TargetPositionEvent target, double altitude);

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @param target Updated target information.
     * @return Return the state of the tracking algorithm.
     */
    behavior_state_t step (cpswarm_msgs::TargetPositionEvent target);
};

#endif // UAV_SIMPLE_TRACKING_H
