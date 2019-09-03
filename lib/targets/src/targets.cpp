#include "targets.h"

targets::targets (string cps) : cps(cps)
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/fov", fov, 0.64);

    // read rescued targets from parameter server
    vector<int> rescued;
    nh.getParam(this_node::getNamespace() + "/rescued", rescued);
    for (int t : rescued) {
        target new_target = target(t, TARGET_RESCUED);
        target_map.emplace(t, new_target);
    }

    // read all potential targets from parameter file
    vector<double> targets_x;
    vector<double> targets_y;
    nh.getParam(this_node::getName() + "/targets_x", targets_x);
    nh.getParam(this_node::getName() + "/targets_y", targets_y);
    if (targets_x.size() != targets_y.size()) {
        ROS_FATAL("Invalid targets specified! Exiting...");
        shutdown();
    }
    else if (targets_x.size() < 1)
        ROS_INFO("There are no targets!");
    for (int i = 0; i < targets_x.size(); ++i) {
        geometry_msgs::Pose new_target_pose;
        new_target_pose.position.x = targets_x[i];
        new_target_pose.position.y = targets_y[i];
        target new_target = target(i, TARGET_UNKNOWN, new_target_pose);
        target_map_all.emplace(i, new_target);
    }

    // tracking publisher
    tracking_pub = nh.advertise<cpswarm_msgs::TargetTracking>("tracking", queue_size);
}

void targets::publish (unsigned int id)
{
    // target is known
    if (target_map.count(id) > 0) {
        // publish target position event
        target_map[id].publish();
    }
}

bool targets::rescued () const
{
    // look through all known targets
    for (auto t : target_map) {
        // target is rescued and has been tracked this cps
        if (t.second.rescued(cps)) {
            return true;
        }
    }

    // this cps has not been tracking a target that is now rescued
    return false;
}

bool targets::rescued (unsigned int id)
{
    // target is known
    if (target_map.count(id) > 0) {
        // target is rescued
        return target_map[id].rescued();
    }

    // the given target is not known
    return false;
}

bool targets::tracking (target& target) const
{
    // look through all known targets
    for (auto t : target_map) {
        // target is tracked by this cps
        if (t.second.tracked(cps)) {
            target = t.second;
            return true;
        }
    }

    // this cps is not tracking
    return false;
}

bool targets::tracking (unsigned int id)
{
    // target is known
    if (target_map.count(id) > 0) {
        // target is being tracked by this cps
        return target_map[id].tracked(cps);
    }

    // the given target is not tracked by this cps
    return false;
}

void targets::update () const
{
    // check if a target is lost
    for (auto t : target_map) {
        // target is being tracked by this cps
        if (t.second.tracked(cps)) {
            // target lost, no tracking information received within timeout
            if (t.second.lost(cps)) {
                // inform others
                t.second.publish();
            }
        }
    }

    // check if a new target is found
    for (auto t : target_map_all) {
        // target pose
        geometry_msgs::Pose pose = t.second.get_pose();

        // visible distance from drone with fov at current altitude
        double dist = pos.get_pose().position.z * tan(fov / 2.0);

        ROS_DEBUG("Target %d distance %.2f < %.2f", t.first, pos.dist(pose), dist);

        // target is within camera fov
        if (pos.dist(pose) <= dist) {
            // publish tracking information
            cpswarm_msgs::TargetTracking track;
            track.header.stamp = Time::now();
            track.id = t.first;
            track.tf = pos.get_transform(pose);
            tracking_pub.publish(track);
        }
    }
}

void targets::update (cpswarm_msgs::TargetPositionEvent msg, target_state_t state, string guided_by, string tracked_by)
{
    // determine target pose
    geometry_msgs::Pose pose;
    pose = msg.pose.pose;

    ROS_DEBUG("Target %d at [%.2f, %.2f]", msg.id, pose.position.x, pose.position.y);

    // update existing target
    if (target_map.count(msg.id) > 0) {
        target_map[msg.id].update(msg.id, state, pose, msg.header.stamp, guided_by, tracked_by);
    }

    // add new target
    else {
        target new_target = target(msg.id, state, pose, msg.header.stamp, guided_by, tracked_by);
        target_map.emplace(msg.id, new_target);
    }
}
