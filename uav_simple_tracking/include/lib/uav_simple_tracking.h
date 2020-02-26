#ifndef UAV_SIMPLE_TRACKING_H
#define UAV_SIMPLE_TRACKING_H

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
    STATE_ABORTED
} behavior_state_t;

/**
 * @brief An implementation that allows to track a target by minimizing the offset between the cyber physical system (CPS) and the target.
 */
class uav_simple_tracking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_simple_tracking (cpswarm_msgs::TargetPositionEvent target, double altitude);

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @param target Updated target information.
     * @return Return the state of the tracking algorithm.
     */
    behavior_state_t step (cpswarm_msgs::TargetPositionEvent target);

    /**
     * @brief Stop moving.
     */
    void stop ();

private:
    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief The target being tracked by this UAV.
     */
    cpswarm_msgs::TargetPositionEvent target;
};

#endif // UAV_SIMPLE_TRACKING_H
