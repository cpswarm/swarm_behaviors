#ifndef UAV_TRACKING_BEHAVIOR_H
#define UAV_TRACKING_BEHAVIOR_H

#include <position.h>
#include <cpswarm_msgs/TargetPositionEvent.h>

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of the behavior algorithm.
 */
typedef enum {
    STATE_ACTIVE = 0,
    STATE_SUCCEEDED,
    STATE_PREEMPTED,
    STATE_ABORTED
} behavior_state_t;

/**
 * @brief An abstract base class that allows to track a given target using different algorithms.
 */
class uav_tracking_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_tracking_behavior (cpswarm_msgs::TargetPositionEvent target, double altitude);

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @param target Updated target information.
     * @return Return the state of the tracking algorithm.
     */
    virtual behavior_state_t step (cpswarm_msgs::TargetPositionEvent target) = 0;

    /**
     * @brief Stop moving.
     */
    void stop ();

protected:
    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief The target being tracked by this UAV.
     */
    cpswarm_msgs::TargetPositionEvent target;
};

#endif // UAV_TRACKING_BEHAVIOR_H
