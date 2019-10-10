#include "position.h"

position::position () : moveto_client("cmd/moveto", true)
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/goal_timeout", goal_timeout, 30.0);

    // no pose received yet
    pose_valid = false;

    // init ros communication
    out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &position::pose_callback, this);
    moveto_client.waitForServer();
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
    return remainder(atan2(p.position.y - pose.position.y, p.position.x - pose.position.x) - get_yaw(), 2*M_PI);
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
    goal.position.z = start.position.z;

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

void position::move (geometry_msgs::Pose goal)
{
    // position to move to
    MoveBaseGoal moveto_goal;
    moveto_goal.target_pose.pose = goal;

    // send goal pose to moveto action server
    moveto_client.sendGoal(moveto_goal);

    // wait until goal is reached
    bool reached = moveto_client.waitForResult(Duration(goal_timeout));

    // failed to reach goal within time
    if (reached == false) {
        ROS_ERROR("Failed to move to (%.2f,%.2f)!", goal.position.x, goal.position.x);
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
