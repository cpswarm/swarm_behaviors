#include "lib/uav_flocking_tracking.h"

uav_flocking_tracking::uav_flocking_tracking(unsigned int target, string ugv) : uav_tracking(target, ugv)
{
    flock = new uav_flocking();
}

uav_flocking_tracking::~uav_flocking_tracking ()
{
    delete flock;
}

behavior_state_t uav_flocking_tracking::step ()
{
    // update position information
    spinOnce();

    // update information about tracking targets
    sar_targets->update();

    // get the target being tracked
    target tracked;

    // uav is tracking a target
    if (sar_targets->tracking(tracked)) {
        // target pose
        geometry_msgs::Pose target_pose = tracked.get_pose();

        // target left the area
        if (pos->out_of_bounds(target_pose)) {
            state = STATE_SUCCEEDED;
        }

        // target is still inside area
        else {
            // compute velocity for flocking of CPSs
            geometry_msgs::Vector3 velocity = flock->tracking(target_pose);

            // move with new velocity
            move(velocity);
        }
    }

    // no target being tracked by this uav
    else{
        // target has been rescued
        if (sar_targets->rescued())
            state = STATE_SUCCEEDED;

        // target has been lost
        else
            state = STATE_ABORTED;
    }

    // return state to action server
    return state;
}
