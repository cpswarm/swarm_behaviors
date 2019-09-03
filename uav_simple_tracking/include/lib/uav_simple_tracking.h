#ifndef UAV_SIMPLE_TRACKING_H
#define UAV_SIMPLE_TRACKING_H

#include "uav_tracking.h"

/**
 * @brief An implementation of the tracking class that allows to track a target by minimizing the offset between the cyber physical system (CPS) and the target.
 */
class uav_simple_tracking : public uav_tracking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param ugv The UGV that has been assigned to rescue the tracked target.
     */
    uav_simple_tracking (unsigned int target, string ugv);

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @return Return the state of the tracking algorithm.
     */
    behavior_state_t step ();

private:
    /**
     * @brief Compute goal position in order to continue tracking of the target.
     * @return The selected goal.
     */
    geometry_msgs::Pose select_goal ();
};

#endif // UAV_SIMPLE_TRACKING_H
