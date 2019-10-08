#include "lib/uav_flocking_tracking.h"

uav_flocking_tracking::uav_flocking_tracking(unsigned int target)
{
    this->target.id = target;
    flock = new uav_flocking();
    NodeHandle nh;
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    target_sub = nh.subscribe("target_update", queue_size, &uav_flocking_tracking::target_callback, this);
}

uav_flocking_tracking::~uav_flocking_tracking ()
{
    delete flock;
}

behavior_state_t uav_flocking_tracking::step ()
{
    // update position information
    spinOnce();

    // target left the area
    if (pos.out_of_bounds(target.pose.pose)) {
        return STATE_SUCCEEDED;
    }

    // target is still inside area
    else {
        // compute velocity for flocking of CPSs
        geometry_msgs::Vector3 velocity = flock->tracking(target.pose.pose);

        // move with new velocity
        vel.move(velocity);
    }

    // return state to action server
    return STATE_ACTIVE;
}

void uav_flocking_tracking::target_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    // update information for this target
    if (target.id == msg->id)
        target = *msg;
}
