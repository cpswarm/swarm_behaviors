#include "lib/uav_flocking_tracking.h"

uav_flocking_tracking::uav_flocking_tracking(cpswarm_msgs::TargetPositionEvent target, double altitude) : uav_tracking_behavior(target, altitude), flock(altitude), vel(altitude)
{
}

behavior_state_t uav_flocking_tracking::step (cpswarm_msgs::TargetPositionEvent target)
{
    // update target information
    this->target = target;

    // target left the area
    if (pos.out_of_bounds(target.pose.pose)) {
        return STATE_SUCCEEDED;
    }

    // target is still inside area
    else {
        // compute velocity for flocking of CPSs
        geometry_msgs::Vector3 velocity = flock.formation_velocity(target.pose.pose);

        // move with new velocity
        vel.move(velocity);
    }

    // return state to action server
    return STATE_ACTIVE;
}

void uav_flocking_tracking::stop ()
{
    geometry_msgs::Vector3 velocity;
    vel.move(velocity);
}
