#include "lib/uav_simple_tracking.h"

uav_simple_tracking::uav_simple_tracking(unsigned int target, string ugv) : uav_tracking(target, ugv)
{

}

behavior_state_t uav_simple_tracking::step ()
{
    // update position information
    spinOnce();

    // update information about tracking targets
    sar_targets->update();

    // get the target being tracked
    target tracked;

    // uav is tracking a target
    if (sar_targets->tracking(tracked)) {
        // target left the area
        if (pos->out_of_bounds(tracked.get_pose())) {
            state = STATE_SUCCEEDED;
        }

        // target is still inside area
        else {
            // compute new goal
            geometry_msgs::Pose goal = select_goal();

            // move to target position
            move(goal);
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

geometry_msgs::Pose uav_simple_tracking::select_goal ()
{
    // get target tracked by this cps
    target tracked;

    // get target position
    sar_targets->tracking(tracked);

    return tracked.get_pose();
}
