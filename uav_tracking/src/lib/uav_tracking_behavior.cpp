#include "lib/uav_tracking_behavior.h"

uav_tracking_behavior::uav_tracking_behavior(cpswarm_msgs::TargetPositionEvent target, double altitude) : target(target), pos(altitude)
{
}

void uav_tracking_behavior::stop ()
{
    pos.stop();
}
