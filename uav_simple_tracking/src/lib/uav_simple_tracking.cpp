#include "lib/uav_simple_tracking.h"

uav_simple_tracking::uav_simple_tracking(unsigned int target)
{
    this->target.id = target;
    NodeHandle nh;
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    target_sub = nh.subscribe("target_update", queue_size, &uav_simple_tracking::target_callback, this);
}

behavior_state_t uav_simple_tracking::step ()
{
    // update position information
    spinOnce();

    // target left the area
    if (pos.out_of_bounds(target.pose.pose)) {
        return STATE_SUCCEEDED;
    }

    // target is still inside area
    else {
        // move to target position
        pos.move(target.pose.pose);
    }

    // return state to action server
    return STATE_ACTIVE;
}

void uav_simple_tracking::target_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    // update information for this target
    if (target.id == msg->id)
        target = *msg;
}
