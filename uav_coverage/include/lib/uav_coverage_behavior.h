#ifndef UAV_COVERAGE_BEHAVIOR_H
#define UAV_COVERAGE_BEHAVIOR_H

#include <position.h>

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
 * @brief An abstract base class that allows to cover a given area using different algorithms.
 */
class uav_coverage_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_coverage_behavior (double altitude);

    /**
     * @brief Move the CPS to a new position.
     * @return Return the state of the coverage algorithm.
     */
    virtual behavior_state_t step () = 0;

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
     * @brief The altitude of the CPS above ground.
     */
    double altitude;
};

#endif // UAV_COVERAGE_BEHAVIOR_H
