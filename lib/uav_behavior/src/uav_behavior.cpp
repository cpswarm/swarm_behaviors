#include "uav_behavior.h"

uav_behavior::uav_behavior () : cps_behavior()
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init publishers and subscribers
    tracking_sub = nh.subscribe("tracking", queue_size, &uav_behavior::tracking_callback, this);
}
