#ifndef VELOCITY_H
#define VELOCITY_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "position.h"
#include "cpswarm_msgs/Vector.h"

using namespace std;
using namespace ros;

/**
 * @brief A class to provide velocity related functionalities.
 */
class velocity
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    velocity ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~velocity ();

    /**
     * @brief Compute the velocity vector required to reach a goal point from the current position while traveling with a given maximum velocity.
     * @param goal The goal coordinates.
     * @param velocity The velocity magnitude.
     * @return The linear velocity vector that moves the CPS in the direction of the goal point.
     */
    geometry_msgs::Vector3 compute_velocity (geometry_msgs::Point goal, double velocity);

    /**
     * @brief Get the current velocity of the CPS.
     * @return The current linear velocity of the CPS.
     */
    geometry_msgs::Vector3 get_velocity () const;

    /**
    /**@brief Move the CPS with a given velocity.
     * @param velocity The velocity at which the CPS shall move.
     */
    void move (geometry_msgs::Vector3 velocity);

    /**
     * @brief Compute the velocity difference of the CPS to a given velocity.
     * @param v The velocity to compare.
     * @return The velocity relative to the current velocity of the CPS as magnitude and direction.
     */
    cpswarm_msgs::Vector rel_velocity (geometry_msgs::Vector3 v) const;

private:
    /**
     * @brief Callback function for velocity updates.
     * @param msg Velocity received from the CPS FCU.
     */
    void vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the velocity of the CPS.
     */
    Subscriber vel_sub;

    /**
     * @brief Publisher for sending the target velocity of the CPS to the velocity controller in the abstraction library.
     */
    Publisher vel_pub;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief Current velocity of the CPS.
     */
    geometry_msgs::Twist current_vel;

    /**
     * @brief Whether a valid velocity has been received.
     */
    bool vel_valid;

    /**
     * @brief The velocity that the CPS can be slower or faster than the target velocity while still being considered to have reached that target velocity.
     */
    double vel_tolerance;
};

#endif // VELOCITY_H
