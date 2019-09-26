#ifndef UAV_RANDOM_DIRECTION_H
#define UAV_RANDOM_DIRECTION_H

#include <random_numbers/random_numbers.h>
#include "cpswarm_msgs/GetSector.h"
#include "uav_coverage.h"

/**
 * @brief An implementation of the coverage class that allows to cover a given area with the random direction algorithm. The random direction is a mathematical movement model, where an agent moves straight forward until it reaches an obstacle. There, it changes its direction randomly.
 */
class uav_random_direction : public uav_coverage
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param seed Seed for random number generation. If not set, a random seed will be used.
     */
    uav_random_direction (int seed=0);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_random_direction ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step();

private:
    /**
     * @brief Obstacle avoidance procedure: change direction.
     */

    void obstacle_avoidance ();

    /**
     * @brief Compute goal position from direction.
     * @return The selected goal.
     */
    geometry_msgs::Pose select_goal ();

    /**
     * @brief Compute new direction using rng.
     */
    void new_direction ();

    /**
     * @brief Service client for determining the sector clear of obstacles.
     */
    ServiceClient clear_sector_client;

    /**
     * @brief Whether the drone still needs to turn before moving.
     */
    bool turn;

    /**
     * @brief The direction in which the drone is travling. It is measured in radian, clockwise starting from north.
     */
    double direction;

    /**
     * @brief The distance which the drone is traveling.
     */
    double distance;

    /**
     * @brief The random number generator used for selecting a random direction.
     */
    random_numbers::RandomNumberGenerator* rng;
};

#endif // UAV_RANDOM_DIRECTION_H

