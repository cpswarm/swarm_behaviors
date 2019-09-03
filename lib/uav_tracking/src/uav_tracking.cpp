#include "uav_tracking.h"

uav_tracking::uav_tracking (unsigned int target, string ugv) : uav_behavior()
{
    // init position
    spinOnce();

    // store given target
    cpswarm_msgs::TargetPositionEvent msg;
    msg.id = target;
    msg.pose.pose = pos->get_pose();
    msg.header.stamp = Time::now();
    sar_targets->update(msg, TARGET_TRACKED, ugv, id);
}

void uav_tracking::target_rescued_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    if (sar_targets->tracking(msg->id))
        state = STATE_SUCCEEDED;

    uav_behavior::target_rescued_callback(msg);
}

void uav_tracking::tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg)
{
    // the target is tracked by this uav
    if (sar_targets->tracking(msg->id)) {
        // update local variable
        cpswarm_msgs::TargetPositionEvent event;
        event.id = msg->id;
        event.pose.pose = pos->get_pose(msg->tf);
        event.header.stamp = msg->header.stamp;
        sar_targets->update(event, TARGET_TRACKED, "", id);

        // inform selected ugv about target update
        sar_targets->publish(msg->id);
    }
}
