#include "target.h"

target::target () : target(-1, TARGET_UNKNOWN)
{
}

target::target (unsigned int id, target_state_t state) : target(id, state, geometry_msgs::Pose())
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose) : target(id, state, pose, Time::now())
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp) : target(id, state, pose, stamp, "", "")
{
}

target::target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string guided_by, string tracked_by) : id(id), state(state), pose(pose), stamp(stamp), guided_by(guided_by)
{
    // initialize tracking information
    if (tracked_by != "")
        this->tracked_by.insert(tracked_by);

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    double tracking_timeout;
    nh.param(this_node::getName() + "/tracking_timeout", tracking_timeout, 1.0);
    this->tracking_timeout = Duration(tracking_timeout);
    nh.param(this_node::getName() + "/target_tolerance", target_tolerance, 0.1);
    nh.getParam(this_node::getName() + "/target_found_event", target_found_event);
    nh.getParam(this_node::getName() + "/target_lost_event", target_lost_event);
    nh.getParam(this_node::getName() + "/target_update_event", target_update_event);

    // initialize publishers
    target_found_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_found", queue_size, true);
    target_update_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_update", queue_size, true);
    target_lost_pub = nh.advertise<cpswarm_msgs::TargetPositionEvent>("target_lost", queue_size, true);

    // init loop rate
    rate = new Rate(loop_rate);
}

int target::get_id () const
{
    return id;
}

geometry_msgs::Pose target::get_pose () const
{
    return pose;
}

bool target::lost (string cps)
{
    bool lost;

    // target is already marked as lost
    if (state == TARGET_LOST)
        lost = true;

    // target is being tracked
    else if (state == TARGET_TRACKED) {
        // check if tracking timeout expired
        lost = stamp + tracking_timeout < Time::now();

        // update target information
        if (lost) {
            tracked_by.erase(cps);
            if (tracked_by.size() < 1)
                state = TARGET_LOST;
        }
    }

    // in other states the target cannot get lost
    else
        lost = false;

    return lost;
}

void target::publish ()
{
    // initialize target position event
    cpswarm_msgs::TargetPositionEvent target;
    geometry_msgs::PoseStamped ps;
    ps.pose = pose;
    ps.header.frame_id = "map";
    target.pose = ps;
    target.header.stamp = Time::now();
    target.id = id;

    // target is being tracked
    if (state == TARGET_TRACKED) {
        // new target found
        if (last_pose.position.x == 0 && last_pose.position.y == 0 && last_pose.position.z == 0) {
            // wait until subscriber is connected
            while (ok() && target_found_pub.getNumSubscribers() <= 0)
                rate->sleep();

            // set event name
            target.swarmio.name = target_found_event;

            // publish event
            target_found_pub.publish(target);
        }

        // update for already known target
        else {
            // compute distance that target moved
            double moved = hypot(last_pose.position.x - pose.position.x, last_pose.position.y - pose.position.y);

            if (moved > target_tolerance) {
                // wait until subscriber is connected
                while (ok() && target_update_pub.getNumSubscribers() <= 0)
                    rate->sleep();

                // set event name
                target.swarmio.name = target_update_event;

                // address only cps that is guiding this target
                if (guided_by != "")
                    target.swarmio.node = guided_by;

                // publish event
                target_update_pub.publish(target);
            }
        }

        // store current pose
        last_pose = pose;
    }

    // target has been lost
    else if (state == TARGET_LOST) {
        // wait until subscriber is connected
        while (ok() && target_lost_pub.getNumSubscribers() <= 0)
            rate->sleep();

        // set event name
        target.swarmio.name = target_lost_event;

        // address only cps that is guiding this target
        if (guided_by != "")
            target.swarmio.node = guided_by;

        // publish event
        target_lost_pub.publish(target);
    }
}

bool target::rescued () const
{
    return state == TARGET_RESCUED;
}

bool target::rescued (string cps) const
{
    return state == TARGET_RESCUED && tracked_by.count(cps) > 0;
}

bool target::tracked (string cps) const
{
    return state == TARGET_TRACKED && tracked_by.count(cps) > 0;
}

void target::update (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string guided_by, string tracked_by)
{
    // update target information
    this->pose = pose;
    this->state = state;
    this->stamp = stamp;

    // update guiding information
    if (guided_by != "") {
        this->guided_by = guided_by;
    }

    // update tracking information
    if (state == TARGET_TRACKED) {
        this->tracked_by.insert(tracked_by);
    }
    else if (state == TARGET_RESCUED) {
        // store target id in parameter server
        vector<int> rescued;
        nh.getParam(this_node::getNamespace() + "/rescued", rescued);
        rescued.push_back(id);
        nh.setParam(this_node::getNamespace() + "/rescued", rescued);
    }
    else if (state == TARGET_LOST) {
        this->tracked_by.erase(tracked_by);

        if (this->tracked_by.size() < 1) {
            state = TARGET_LOST;
        }
        else {
            state = TARGET_TRACKED;
        }
    }
}
