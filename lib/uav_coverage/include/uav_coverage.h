#ifndef UAV_COVERAGE_H
#define UAV_COVERAGE_H

#include "cpswarm_msgs/CoverageAction.h"
#include "uav_behavior.h"

/**
 * @brief An abstract class that allows to cover a given area with a cyber physical system (CPS).
 */
class uav_coverage : public uav_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    uav_coverage ();

    /**
     * @brief Get information about a target that has been found by this UAV.
     * @param t Target ID and pose.
     * @return True if a target has been found, false otherwise.
     */
    bool get_target (cpswarm_msgs::CoverageResult& t) const;

protected:
    /**
     * @brief The maximum distance that a UAV travels in one step.
     */
    double step_size_max;

    /**
     * @brief The minimum distance that a UAV travels in one step.
     */
    double step_size_min;

private:
    /**
     * @brief Callback function for target position.
     * @param msg ID of target and translation between CPS and target as received from the OpenMV camera.
     */
    void tracking_callback (const cpswarm_msgs::TargetTracking::ConstPtr& msg);
};

#endif // UAV_COVERAGE_H

