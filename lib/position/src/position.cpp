#include "position.h"

position::position ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.02);

    // no pose received yet
    pose_valid = false;

    // init ros communication
    out_of_bounds_client = nh.serviceClient<cpswarm_msgs::out_of_bounds>("area/out_of_bounds");
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &position::pose_callback, this);

    // init position and yaw
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate.sleep();
        spinOnce();
    }
}

position::~position ()
{
}

angle position::bearing (geometry_msgs::Pose p) const
{
    return angle(atan2(p.position.y - pose.position.y, p.position.x - pose.position.x) - get_yaw().rad());
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

geometry_msgs::Pose position::get_pose (geometry_msgs::Transform tf) const
{
    // compute distance and direction of target
    double distance = hypot(tf.translation.x, tf.translation.y);
    double direction = (get_yaw() + atan2(tf.translation.y, -tf.translation.x) - M_PI / 2).rad_pos(); // x is inverted in tracking camera tf

    // compute target pose in local obstaclerdinates
    return compute_goal(pose, distance, direction);
}

geometry_msgs::Transform position::get_transform (geometry_msgs::Pose p) const
{
    // relative obstaclerdinates of pose
    double dx = p.position.x - pose.position.x;
    double dy = p.position.y - pose.position.y;
    double distance = hypot(dx, dy);
    angle direction = angle(M_PI / 2.0) - get_yaw() + angle(atan2(dy, dx));

    // compute transform
    geometry_msgs::Transform tf;
    tf.translation.x = -distance * cos(direction.rad()); // x is inverted in tracking camera tf
    tf.translation.y = distance * sin(direction.rad());

    return tf;
}

angle position::get_yaw () const
{
    return get_yaw(pose);
}

bool position::out_of_bounds (geometry_msgs::Pose pose)
{
    cpswarm_msgs::out_of_bounds oob;
    oob.request.pose = pose;
    if (out_of_bounds_client.call(oob)){
        return oob.response.out;
    }
    else{
        ROS_ERROR("Failed to check if goal is out of bounds");
        return true;
    }
}

bool position::reached (geometry_msgs::Pose goal)
{
    return dist(pose, goal) < goal_tolerance;
}

bool position::reached_yaw (geometry_msgs::Pose goal)
{
    return (get_yaw() - get_yaw(goal)).rad_pos() < yaw_tolerance;
}

angle position::get_yaw (geometry_msgs::Pose pose) const
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return angle(tf2::getYaw(orientation));
}

void position::pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Yaw %.2f", get_yaw().rad_pos());
    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}
