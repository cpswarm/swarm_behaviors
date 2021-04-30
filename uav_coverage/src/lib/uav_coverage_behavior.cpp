#include "lib/uav_coverage_behavior.h"

uav_coverage_behavior::uav_coverage_behavior(double altitude) : pos(altitude), altitude(altitude)
{
}

void uav_coverage_behavior::stop ()
{
    pos.stop();
}
