#ifndef UAV_LOCAL_COVERAGE_H
#define UAV_LOCAL_COVERAGE_H

#include <position.h>

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
 * @brief A class that allows to cover a given area using a local search algorithm. The local search performs a spiral movement pattern according to the circle involute (http://mathworld.wolfram.com/CircleInvolute.html).
 */
class uav_local_coverage
{
public:
  /**
   * @brief Constructor that initializes the private member variables.
   */
    uav_local_coverage();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step();

private:
    /**
     * @brief Compute local coordinates on circle involute for current step.
     * @param distance Returns distance from local origin.
     * @param direction Returns direction from local origin.
     */
    void compute_involute (double &distance, double &direction);

    /**
     * @brief Compute goal position.
     * @return The selected goal.
     */
    geometry_msgs::Pose select_goal ();

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief The altitude of the UAV above ground.
     */
    double altitude;

    /**
     * @brief Horizontal camera field of view in radian.
     */
    double fov_hor;

    /**
     * @brief Vertical camera field of view in radian.
     */
    double fov_ver;

    /**
     * @brief Center of the spiral movement search pattern.
     */
    geometry_msgs::Pose origin;

    /**
     * @brief Number of steps performed in the local search.
     */
    unsigned int steps;

    /**
     * @brief Maximum number of steps to do in the local search.
     */
    int max_steps;


};

#endif // UAV_LOCAL_COVERAGE_H
