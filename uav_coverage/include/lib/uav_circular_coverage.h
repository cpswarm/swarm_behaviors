#ifndef UAV_CIRCULAR_COVERAGE_H
#define UAV_CIRCULAR_COVERAGE_H

#include "uav_coverage_behavior.h"

/**
 * @brief A class that allows to cover a given area using a circular search algorithm.
 */
class uav_circular_coverage : public uav_coverage_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_circular_coverage (double altitude);

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step ();

private:
    /**
     * @brief Compute circular coordinates for current step.
     * @param distance Returns distance from circular origin.
     * @param direction Returns direction from circular origin.
     */
    void compute_circle (double &distance, double &direction);

    /**
     * @brief Compute goal position.
     * @return The selected goal.
     */
    geometry_msgs::Pose select_goal ();

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
     * @brief Number of steps performed in the circular search.
     */
    unsigned int steps;

    /**
     * @brief Maximum number of steps to do in the circular search.
     */
    int max_steps;
};

#endif // UAV_CIRCULAR_COVERAGE_H
