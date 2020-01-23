#ifndef UAV_RANDOM_DIRECTION_H
#define UAV_RANDOM_DIRECTION_H

#include <random_numbers/random_numbers.h>
#include <cpswarm_msgs/GetSector.h>
#include <cpswarm_msgs/GetPoints.h>
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
 * @brief A class that allows to cover a given area with the random direction algorithm. The random direction is a mathematical movement model, where an agent moves straight forward until it reaches an obstacle. There, it changes its direction randomly.
 */
class uav_random_direction
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_random_direction (double altitude);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_random_direction ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step();

    /**
     * @brief Stop moving.
     */
    void stop ();

private:
    /**
     * @brief Compute goal position from direction.
     * @return Whether the goal could be computed successfully.
     */
    bool select_goal ();

    /**
     * @brief Compute new direction using rng.
     * @return Whether a a new direction could be set successfully.
     */
    bool new_direction ();

    /**
     * @brief Service client to get the area polygon.
     */
    ServiceClient area_client;

    /**
     * @brief Service client for determining the sector clear of obstacles.
     */
    ServiceClient clear_sector_client;

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief The direction in which the drone is travling. It is measured in radian, clockwise starting from north.
     */
    double direction;

    /**
     * @brief The current goal.
     */
    geometry_msgs::Pose goal;

    /**
     * @brief The random number generator used for selecting a random direction.
     */
    random_numbers::RandomNumberGenerator* rng;

    /**
     * @brief The distance in meter to keep to the environment boundary.
     */
    double margin;
};

#endif // UAV_RANDOM_DIRECTION_H

