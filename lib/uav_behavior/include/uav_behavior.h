#ifndef UAV_BEHAVIOR_H
#define UAV_BEHAVIOR_H

#include "cps_behavior.h"

/**
 * @brief An abstract base class for all behaviors of unmanned aerial vehicle (UAV) cyber physical systems (CPSs).
 */
class uav_behavior : public cps_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    uav_behavior ();

protected:
    /**
     * @brief Subscriber for the tracking information of the CPS's camera.
     */
    Subscriber tracking_sub;

private:
    /**
     * @brief Callback function for target position.
     * @param msg ID of target and translation between CPS and target as received from the OpenMV camera.
     */
    virtual void tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg) = 0;
};

#endif // UAV_BEHAVIOR_H
