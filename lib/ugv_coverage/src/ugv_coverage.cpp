#include "ugv_coverage.h"

ugv_coverage::ugv_coverage () : ugv_behavior()
{
    // read parameters
    nh.param(this_node::getName() + "/step_size_min", step_size_min, 1.0);
    nh.param(this_node::getName() + "/step_size_max", step_size_max, 3.0);
}
