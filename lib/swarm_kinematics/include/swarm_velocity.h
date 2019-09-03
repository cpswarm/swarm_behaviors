#ifndef SWARM_VELOCITY_H
#define SWARM_VELOCITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/VectorStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief A class that holds velocity information of the other CPSs in the swarm.
 */
class swarm_velocity
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    swarm_velocity ();

    /**
     * @brief Get the relative velocities of the other CPSs.
     * @return A vector containing magnitude and direction of relative velocities.
     */
    const vector<cpswarm_msgs::VectorStamped>& get_velocities_rel () const;

    /**
     * @brief Get the relative velocity of a specific CPS.
     * @param uuid The UUID of the CPS to get the velocity for.
     * @return The magnitude and direction of relative velocity of given CPS. An empty vector in case the UUID is unknown.
     */
    cpswarm_msgs::Vector get_velocity_rel (string uuid) const;

private:
    /**
     * @brief Callback function to receive the absolute velocities of the other CPSs.
     * @param msg GPS coordinates of the other CPSs.
     */
    void swarm_vel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg);

    /**
     * @brief Callback function to receive the relative velocities of the other CPSs.
     * @param msg Distance and bearing of the other CPSs.
     */
    void swarm_vel_rel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the absolute velocities of the other CPSs.
     */
    Subscriber swarm_vel_sub;

    /**
     * @brief Subscriber for the relative velocities of the other CPSs.
     */
    Subscriber swarm_vel_rel_sub;

    /**
     * @brief The absolute velocities of the other CPSs.
     */
    vector<vector<cpswarm_msgs::VectorStamped>> velocities;

    /**
     * @brief The relative velocities of the other swarm members (distance and bearing).
     */
    vector<vector<cpswarm_msgs::VectorStamped>> velocities_rel;

    /**
     * @brief The index of the latest velocity in the velocities vectors.
     */
    unsigned int t;
};

#endif // SWARM_VELOCITY_H
