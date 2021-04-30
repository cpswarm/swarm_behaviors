#include "lib/uav_simple_tracking.h"

uav_simple_tracking::uav_simple_tracking(cpswarm_msgs::TargetPositionEvent target, double altitude) : uav_tracking_behavior(target, altitude)
{
}

behavior_state_t uav_simple_tracking::step (cpswarm_msgs::TargetPositionEvent target)
{
    // update target information
    this->target = target;

    // check if target pose has been received
    if (target.header.stamp > Time(0)) {
        // target left the area
        if (pos.out_of_bounds(target.pose.pose)) {
            return STATE_SUCCEEDED;
        }

        // target is still inside area
        else {
            // move to target position
            if (pos.move(target.pose.pose) == false)
                return STATE_ABORTED;
        }
    }
    else {
        ROS_WARN_ONCE("No pose received for target %d, holding position!", target.id);
    }

    // return state to action server
    return STATE_ACTIVE;
}
