#include "cps_behavior.h"

cps_behavior::cps_behavior ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init publishers and subscribers
    target_found_sub = nh.subscribe("bridge/events/target_found", queue_size, &cps_behavior::target_found_callback, this);
    target_lost_sub = nh.subscribe("bridge/events/target_lost", queue_size, &cps_behavior::target_lost_callback, this);
    target_rescued_sub = nh.subscribe("bridge/events/target_rescued", queue_size, &cps_behavior::target_rescued_callback, this);
    uuid_sub = nh.subscribe("bridge/uuid", queue_size, &cps_behavior::uuid_callback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_controller/goal_position", queue_size);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("vel_controller/target_velocity", queue_size);

    // init state of algorithm
    state = STATE_ACTIVE;

    // init uuid
    id = "";

    // init loop rate
    rate = new Rate(loop_rate);

    // init uuid
    while (ok() && id == "") {
        rate->sleep();
        spinOnce();
    }

    // initialize helper libraries
    pos = new position();
    vel = new velocity();

    // initialize collection of targets
    sar_targets = new targets(id);
}

cps_behavior::~cps_behavior ()
{
    delete rate;
    delete pos;
    delete vel;
    delete sar_targets;
}

void cps_behavior::move (geometry_msgs::Pose goal)
{
    // create goal pose message
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = goal;

    // send goal pose to cps controller
    pose_pub.publish(goal_pose);

    // wait until ugv reached goal
    while (ok() && pos->reached(goal) == false) {
        // wait
        rate->sleep();

        // check if reached goal
        spinOnce();
    }
}

void cps_behavior::move (geometry_msgs::Vector3 velocity)
{
    // create target velocity message
    geometry_msgs::Twist target_velocity;
    target_velocity.linear = velocity;

    // send target velocity to cps controller
    velocity_pub.publish(target_velocity);
}

void cps_behavior::target_rescued_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    sar_targets->update(*msg, TARGET_RESCUED, msg->swarmio.node);
}

void cps_behavior::target_found_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    sar_targets->update(*msg, TARGET_TRACKED, "", msg->swarmio.node);
}

void cps_behavior::target_lost_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg) const
{
    sar_targets->update(*msg, TARGET_LOST, "", msg->swarmio.node);
}

void cps_behavior::uuid_callback (const swarmros::String::ConstPtr& msg)
{
    id = msg->value;
}
