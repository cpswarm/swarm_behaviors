#ifndef POSITION_H
#define POSITION_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cpswarm_msgs/OutOfBounds.h>

using namespace std;
using namespace ros;
using namespace actionlib;
using namespace move_base_msgs;

typedef SimpleActionClient<MoveBaseAction> move_base_client;

/**
 * @brief A class to provide position related functionalities.
 */
class position
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    position ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~position ();

    /**
     * @brief Compute the bearing from the current pose of the CPS to a given pose.
     * @param p The pose to compute bearing of.
     * @return The bearing of the given pose relative to the current yaw of the CPS, counterclockwise.
     */
    double bearing (geometry_msgs::Pose p) const;

    /**
     * @brief Compute the goal coordinates relative to the current position.
     * @param distance The distance of the goal from the current position.
     * @param direction The direction of the goal relative to the current position. It is in radian, counterclockwise starting from east / x-axis.
     * @return The goal pose.
     */
    geometry_msgs::Pose compute_goal (double distance, double direction) const;

    /**
     * @brief Compute the goal coordinates relative to a given start position.
     * @param start The starting position.
     * @param distance The distance of the goal from start.
     * @param direction The direction of the goal relative to start. It is in radian, counterclockwise starting from east / x-axis.
     * @return The goal pose.
     */
    geometry_msgs::Pose compute_goal (geometry_msgs::Pose start, double distance, double direction) const;

    /**
     * @brief Compute the straight-line distance from the current position to the given position.
     * @param p The pose to compute distance to.
     * @return The distance in meters.
     */
    double dist (geometry_msgs::Pose p) const;

    /**
     * @brief Compute the straight-line distance between two positions.
     * @param p1 First pose.
     * @param p2 Second pose.
     * @return The distance in meters.
     */
    double dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const;

    /**
     * @brief Get the current pose of the CPS.
     * @return The current pose of the CPS in local coordinates.
     */
    geometry_msgs::Pose get_pose () const;

    /**
     * @brief Get the current yaw orientation of the CPS.
     * @return The current yaw angle of the CPS counterclockwise starting from x-axis/east.
     */
    double get_yaw () const;

    /**
     * @brief Move the CPS to the given pose.
     * @param pose The position to move to.
     */
    void move (geometry_msgs::Pose pose);

    /**
     * @brief Check whether a given pose is out of the mission area boundaries.
     * @param pose The pose to check.
     * @return True if the given pose is outside the mission area or it could not be checked, false otherwise.
     */
    bool out_of_bounds (geometry_msgs::Pose pose);

private:
    /**
     * @brief Get the yaw orientation from a pose.
     * @param pose The pose that contains the orientation.
     * @return The yaw angle of the given pose counterclockwise starting from x-axis/east.
     */
    double get_yaw (geometry_msgs::Pose pose) const;

    /**
     * @brief Callback function for position updates.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Subscriber for the position of the CPS.
     */
    Subscriber pose_sub;

    /**
     * @brief Service client for determining whether the goal is out of the area bounds.
     */
    ServiceClient out_of_bounds_client;

    /**
     * @brief Action client to move the CPS.
     */
    move_base_client moveto_client;

    /**
     * @brief Publisher for sending the goal position of the CPS to the position controller in the abstraction library.
     */
    Publisher pose_pub;

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

    /**
     * @brief Current position of the CPS.
     */
    geometry_msgs::Pose pose;

    /**
     * @brief Whether a valid position has been received.
     */
    bool pose_valid;

    /**
     * @brief The time in seconds that the CPS is given time to reach a destination before giving up.
     */
    double goal_timeout;
};

#endif // POSITION_H
