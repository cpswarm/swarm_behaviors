#include "uav_coverage.h"

uav_coverage::uav_coverage () : uav_behavior()
{
    // read parameters
    nh.param(this_node::getName() + "/step_size_min", step_size_min, 1.0);
    nh.param(this_node::getName() + "/step_size_max", step_size_max, 3.0);
}

bool uav_coverage::get_target (cpswarm_msgs::CoverageResult& t) const
{
    target found;

    // found a target
    if (sar_targets->tracking(found)) {
        t.target_id = found.get_id();
        t.target_pose = found.get_pose();
        return true;
    }

    // no target found
    else
        return false;
}

void uav_coverage::tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg)
{
    // target already rescued
    if (sar_targets->rescued(msg->id)) {
        return;
    }

    // stop moving
    pose_pub.publish(pos->get_pose());

    // update local variable
    cpswarm_msgs::TargetPositionEvent event;
    event.id = msg->id;
    event.pose.pose = pos->get_pose(msg->tf);
    event.header.stamp = msg->header.stamp;
    sar_targets->update(event, TARGET_TRACKED, "", id);

    // inform others about new target
    sar_targets->publish(msg->id);

    // coverage succeeded, switch to tracking
    state = STATE_SUCCEEDED;

    // unsubscribe
    tracking_sub.shutdown();
}
