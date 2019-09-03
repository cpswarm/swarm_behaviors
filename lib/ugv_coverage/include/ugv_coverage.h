#ifndef UGV_COVERAGE_H
#define UGV_COVERAGE_H

#include <cpswarm_msgs/CoverageAction.h>
#include "ugv_behavior.h"

/**
 * @brief An abstract class that allows to cover a given area with a cyber physical system (CPS).
 */
class ugv_coverage : public ugv_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    ugv_coverage ();

protected:
    /**
     * @brief The maximum distance that a UGV travels in one step.
     */
    double step_size_max;

    /**
     * @brief The minimum distance that a UGV travels in one step.
     */
    double step_size_min;

private:
};

#endif // UGV_COVERAGE_H

