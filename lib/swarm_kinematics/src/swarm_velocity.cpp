#include "swarm_velocity.h"

swarm_velocity::swarm_velocity ()
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    int hist;
    nh.param(this_node::getName() + "/sample_hist", hist, 1);

    // initialize parameters
    t = 0;

    // initialize swarm velocity vectors
    for (int i = 0; i < hist; ++i) {
        // absolute
        vector<cpswarm_msgs::VectorStamped> vec;
        velocities.push_back(vec);

        // relative
        vector<cpswarm_msgs::VectorStamped> vec_rel;
        velocities_rel.push_back(vec_rel);
    }

    // publishers and subscribers
    swarm_vel_sub = nh.subscribe("swarm_velocity", queue_size, &swarm_velocity::swarm_vel_rel_callback, this);
}

const vector<cpswarm_msgs::VectorStamped>& swarm_velocity::get_velocities_rel () const
{
    return velocities_rel[t];
}

cpswarm_msgs::Vector swarm_velocity::get_velocity_rel (string uuid) const
{
    // search for cps with given uuid
    for (auto vel : velocities_rel[t]) {
        if (vel.swarmio.node == uuid)
            return vel.vector;
    }

    // return empty vector if uuid is unknown
    return cpswarm_msgs::Vector();
}

void swarm_velocity::swarm_vel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    // TODO
}

void swarm_velocity::swarm_vel_rel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    // increase current index of velocities vector
    t++;
    t %= velocities_rel.size();

    // save velocities
    velocities_rel[t] = msg->vectors;
}
