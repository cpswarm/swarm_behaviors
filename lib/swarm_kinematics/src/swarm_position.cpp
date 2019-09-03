#include "swarm_position.h"

swarm_position::swarm_position ()
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/pose_variation", pose_variation, 0.1);
    int hist;
    nh.param(this_node::getName() + "/sample_hist", hist, 1);
    nh.param(this_node::getName() + "/avoidance_dist", avoidance_dist, 2.0);
    nh.param(this_node::getName() + "/critical_dist", critical_dist, 1.0);
    nh.param(this_node::getName() + "/clear_ang", clear_ang, 0.785);

    // initialize parameters
    t = 0;
    t_rel = 0;

    // initialize swarm position vectors
    for (int i = 0; i < hist; ++i) {
        // absolute
        vector<cpswarm_msgs::Position> vop;
        poses.push_back(vop);

        // relative
        vector<cpswarm_msgs::VectorStamped> vec;
        poses_rel.push_back(vec);
    }

    // publishers and subscribers
    swarm_pose_sub = nh.subscribe("swarm_position", queue_size, &swarm_position::swarm_pose_callback, this);
    swarm_pose_rel_sub = nh.subscribe("swarm_position_rel", queue_size, &swarm_position::swarm_pose_rel_callback, this);
}

geometry_msgs::Point swarm_position::center () const
{
    // initialize mean
    geometry_msgs::Point mean;
    mean.x = 0;
    mean.y = 0;

    // accumulate latest poses
    for (auto pose : poses[t]) {
        mean.x += pose.pose.position.x;
        mean.y += pose.pose.position.y;
    }

    // normalize
    mean.x /= poses[t].size();
    mean.y /= poses[t].size();

    return mean;
}

bool swarm_position::clear () const
{
    // check current cps positions
    for (auto pose : poses_rel[t_rel]) {
        // cps is close enough to start avoidance procedure
        if (pose.vector.magnitude < avoidance_dist) {
            return false;
        }
    }

    // no cps near by
    return true;
}

bool swarm_position::clear_ahead (double heading) const
{
    // the sector that must be clear of cpss
    sector sec(heading-clear_ang, heading+clear_ang);

    ROS_DEBUG("Sector to be clear [%.2f, %.2f]", sec.min(), sec.max());

    // check current cps positions
    for (auto pose : poses_rel[t_rel]) {
        ROS_DEBUG("Checking pose [%.2f, %.2f]", pose.vector.magnitude, pose.vector.direction);
        // cps is close enough to start avoidance procedure
        if (pose.vector.magnitude < avoidance_dist) {
            // cps is in critical sector
            if (sec.contains(pose.vector.direction))
                return false;
        }
    }

    // no cps near by
    return true;
}

double swarm_position::danger () const
{
    // check current cps positions
    for (auto pose : poses_rel[t_rel]) {
        // cps is dangerously close
        if (pose.vector.magnitude < critical_dist) {
            return critical_dist - pose.vector.magnitude ;
        }
    }

    // no cps near by
    return 0.0;
}

const vector<cpswarm_msgs::Position>& swarm_position::get_poses () const
{
    return poses[t];
}

const vector<cpswarm_msgs::VectorStamped>& swarm_position::get_poses_rel () const
{
    return poses_rel[t_rel];
}

sector swarm_position::inflated_region () const
{
    // occupied sector
    sector occ = occupied_region();

    // inflate by clear_ang
    if (occ.size() > 0)
        occ.inflate(clear_ang);

    // return inflated sector
    return occ;
}

sector swarm_position::inflated_region (double heading) const
{
    // occupied sector
    sector occ = occupied_region(heading);

    // inflate by clear_ang
    if (occ.size() > 0)
        occ.inflate(clear_ang);

    // return inflated sector
    return occ;
}

sector swarm_position::occupied_region () const
{
    // minimum and maximum bearing occupied by other cpss
    double swarm_min = 0;
    double swarm_max = 0;

    // take latest poses
    for (auto pose : poses_rel[t_rel]) {
        ROS_DEBUG("Checking pose [%.2f, %.2f]", pose.vector.magnitude, pose.vector.direction);
        if (pose.vector.magnitude <= avoidance_dist) {
            double swarm_min_temp = pose.vector.direction - bearing_tolerance(pose);
            double swarm_max_temp = pose.vector.direction + bearing_tolerance(pose);
            if (swarm_min_temp < swarm_min || swarm_min == 0)
                swarm_min = swarm_min_temp;
            if (swarm_max_temp > swarm_max || swarm_max == 0)
                swarm_max = swarm_max_temp;
        }
    }

    // return sector
    return sector(swarm_min, swarm_max);
}

sector swarm_position::occupied_region (double heading) const
{
    // minimum and maximum bearing occupied by other cpss
    double swarm_min = 0;
    double swarm_max = 0;

    // take latest poses
    for (auto pose : poses_rel[t_rel]) {
        ROS_DEBUG("Checking pose [%.2f, %.2f]", pose.vector.magnitude, pose.vector.direction);
        if (pose.vector.magnitude <= avoidance_dist) {
            double swarm_min_temp = pose.vector.direction - bearing_tolerance(pose);
            double swarm_max_temp = pose.vector.direction + bearing_tolerance(pose);
            if (swarm_min_temp < swarm_min || swarm_min == 0)
                swarm_min = swarm_min_temp;
            if (swarm_max_temp > swarm_max || swarm_max == 0)
                swarm_max = swarm_max_temp;
        }
    }

    // return sector
    return sector(swarm_min-heading, swarm_max-heading);
}

double swarm_position::bearing_tolerance (cpswarm_msgs::VectorStamped pose) const
{
    return asin(pose_variation / pose.vector.magnitude);
}

void swarm_position::swarm_pose_callback (const cpswarm_msgs::ArrayOfPositions::ConstPtr& msg)
{
    // increase current index of positions vector
    t++;
    t %= poses.size();

    // save positions
    poses[t] = msg->positions;
}

void swarm_position::swarm_pose_rel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    // increase current index of positions vector
    t_rel++;
    t_rel %= poses_rel.size();

    // save positions
    poses_rel[t_rel] = msg->vectors;
}
