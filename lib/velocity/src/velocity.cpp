#include "velocity.h"

velocity::velocity ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/vel_tolerance", vel_tolerance, 0.1);

    // no velocity received yet
    vel_valid = false;

    // init publishers and subscribers
    vel_sub = nh.subscribe("vel_provider/velocity", queue_size, &velocity::vel_callback, this);

    // init loop rate
    Rate rate(loop_rate);

    // init velocity
    while (ok() && vel_valid == false) {
        rate.sleep();
        spinOnce();
    }
}

velocity::~velocity ()
{
}

geometry_msgs::Vector3 velocity::compute_velocity (geometry_msgs::Point goal, double velocity)
{
    // relative bearing of goal
    geometry_msgs::Pose goal_pose;
    goal_pose.position = goal;
    angle goal_bear = pos.bearing(goal_pose);

    ROS_DEBUG("Goal bearing %.2f", goal_bear.rad());

    // limit velocity for small distances
    if (pos.dist(goal_pose) < M_PI / 2.0)
        velocity *= sin(pos.dist(goal_pose));

    // velocity components in goal direction
    geometry_msgs::Vector3 vel;
    vel.x = velocity * -sin(goal_bear.rad()); // bearing relative to cps heading
    vel.y = velocity * cos(goal_bear.rad());

    ROS_DEBUG("Velocity (%.2f,%.2f)", vel.x, vel.y);

    return vel;
}

geometry_msgs::Vector3 velocity::get_velocity () const
{
    return current_vel.linear;
}

cpswarm_msgs::Vector velocity::rel_velocity (geometry_msgs::Vector3 v) const
{
    // compute relative velocity
    double dx = v.x - current_vel.linear.x;
    double dy = v.y - current_vel.linear.y;
    double mag = hypot(dx, dy);
    double dir = atan2(dy, dx);

    // return relative velocity
    cpswarm_msgs::Vector rel_vel;
    rel_vel.magnitude = mag;
    rel_vel.direction = dir;
    return rel_vel;
}

void velocity::vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        vel_valid = true;

    current_vel = msg->twist;

    ROS_DEBUG_THROTTLE(1, "Velocity [%.2f, %.2f]", current_vel.linear.x, current_vel.linear.y);
}
