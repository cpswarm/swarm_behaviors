#include "position.h"

position::position (double altitude) : altitude(altitude)
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/goal_timeout", goal_timeout, 30.0);
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);

    // no pose received yet
    pose_valid = false;

    // init ros communication
    out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    out_of_bounds_client.waitForExistence();
    occupied_sector_client = nh.serviceClient<cpswarm_msgs::GetSector>("obstacle_detection/get_occupied_sector");
    occupied_sector_client.waitForExistence();
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &position::pose_callback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size, true);

    // init position and yaw
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate->sleep();
        spinOnce();
    }
}

position::~position ()
{
    delete rate;
}

double position::bearing (geometry_msgs::Pose p) const
{
    double b = remainder(atan2(p.position.y - pose.position.y, p.position.x - pose.position.x) - get_yaw(), 2*M_PI);
    if (b < 0)
        b += 2*M_PI;
    return b;
}

geometry_msgs::Pose position::compute_goal (double distance, double direction) const
{
    return compute_goal(pose, distance, direction);
}

geometry_msgs::Pose position::compute_goal (geometry_msgs::Pose start, double distance, double direction) const
{
    geometry_msgs::Pose goal;

    // calculate position
    goal.position.x = start.position.x + distance * cos(direction);
    goal.position.y = start.position.y + distance * sin(direction);
    goal.position.z = altitude;

    // calculate orientation
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, direction);
    goal.orientation = tf2::toMsg(orientation);

    return goal;
}

double position::dist (geometry_msgs::Pose p) const
{
    return dist(pose, p);
}

double position::dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const
{
    return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}

geometry_msgs::Pose position::get_pose () const
{
    return pose;
}

double position::get_yaw () const
{
    return get_yaw(pose);
}

bool position::move (geometry_msgs::Pose goal)
{
    // goal is out of bounds
    if (out_of_bounds(goal)) {
        ROS_ERROR("Cannot move to (%.2f,%.2f) because it is out of bounds!", goal.position.x, goal.position.y);
        return false;
    }

    // create goal pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = Time::now();
    goal_pose.pose = goal;
    goal_pose.pose.position.z = altitude;

    // send goal pose to cps controller
    pose_pub.publish(goal_pose);

    ROS_INFO("Move to (%.2f,%.2f)", goal.position.x, goal.position.y);

    // wait until cps reached goal
    Time start = Time::now();
    while (ok() && reached(goal) == false && Time::now() <= start + Duration(goal_timeout)) {
        // wait
        rate->sleep();

        // check if reached goal
        spinOnce();
    }

    // could not reach goal within time
    if (Time::now() > start + Duration(goal_timeout)) {
        ROS_WARN("Failed to reach goal (%.2f,%.2f) in timeout %.2fs!", goal.position.x, goal.position.y, goal_timeout);
    }

    // successfully moved to goal
    return true;
}

bool position::occupied (geometry_msgs::Pose pose)
{
    // get occupied sector
    cpswarm_msgs::GetSector ocs;
    if (occupied_sector_client.call(ocs)) {
        // no obstacles
        if (ocs.response.min == ocs.response.max)
            return false;

        ROS_DEBUG("Goal %.2f in occupied sector [%.2f,%.2f]?", bearing(pose), ocs.response.min, ocs.response.max);

        // check if pose is in this sector
        if (ocs.response.max > 2*M_PI && bearing(pose) < M_PI)
            return ocs.response.min <= bearing(pose) + 2*M_PI && bearing(pose) + 2*M_PI <= ocs.response.max; // always max > min
        else
            return ocs.response.min <= bearing(pose) && bearing(pose) <= ocs.response.max; // always max > min
    }
    else {
        ROS_ERROR("Failed to check if goal is occupied");
        return true;
    }
}

bool position::out_of_bounds (geometry_msgs::Pose pose)
{
    cpswarm_msgs::OutOfBounds oob;
    oob.request.pose = pose;
    if (out_of_bounds_client.call(oob)) {
        return oob.response.out;
    }
    else {
        ROS_ERROR("Failed to check if goal is out of bounds");
        return true;
    }
}

double position::get_yaw (geometry_msgs::Pose pose) const
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

bool position::reached (geometry_msgs::Pose goal)
{
    ROS_DEBUG("Yaw %.2f --> %.2f", get_yaw(pose), get_yaw(goal));
    ROS_DEBUG("Pose (%.2f,%.2f) --> (%.2f,%.2f)", pose.position.x, pose.position.y, goal.position.x, goal.position.y);
    ROS_DEBUG("%.2f > %.2f", dist(pose, goal), goal_tolerance);

    // whether cps reached goal position, ignoring yaw
    return dist(pose, goal) <= goal_tolerance;
}

void position::pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Yaw %.2f", get_yaw());
    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}
