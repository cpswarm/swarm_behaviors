#ifndef UAV_TRACKING_H
#define UAV_TRACKING_H

#include "uav_behavior.h"

/**
 * @brief An abstract class that allows to track a target with a cyber physical system (CPS).
 */
class uav_tracking : public uav_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param ugv The UGV that has been assigned to rescue the tracked target.
     */
    uav_tracking(unsigned int target, string ugv);

private:
    /**
     * @brief Callback function for incoming target rescued events.
     * @param msg ID and last position of target.
     */
    void target_rescued_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg);

    /**
     * @brief Callback function for target position.
     * @param msg ID of target and translation between CPS and target as received from the OpenMV camera.
     */
    void tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg);
};

#endif // UAV_TRACKING_H
