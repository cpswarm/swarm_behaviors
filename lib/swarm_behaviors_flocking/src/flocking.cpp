#include "flocking.h"

flocking::flocking (double altitude) : pos(altitude), vel(altitude)
{
    // read parameterss
    NodeHandle nh;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    dt = 1 / loop_rate;
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    double start_delay;
    nh.param(this_node::getName() + "/start_delay", start_delay, 1.0);
    nh.param(this_node::getName() + "/equi_dist", equi_dist, 10.0);
    nh.param(this_node::getName() + "/repulse_spring", repulse_spring, 1.0);
    nh.param(this_node::getName() + "/repulse_max", repulse_max, 1.0);
    nh.param(this_node::getName() + "/align_frict", align_frict, 20.0);
    nh.param(this_node::getName() + "/align_slope", align_slope, 1.0);
    nh.param(this_node::getName() + "/align_min", align_min, 1.0);
    nh.param(this_node::getName() + "/wall_frict", wall_frict, 20.0);
    nh.param(this_node::getName() + "/wall_decay", wall_decay, 1.0);
    nh.param(this_node::getName() + "/accel_time", accel_time, 1.0);
    nh.param(this_node::getName() + "/flocking/flock_vel", flock_vel, 0.5);
    nh.getParam(this_node::getName() + "/formation/formation", form);
    nh.param(this_node::getName() + "/formation/form_vel", form_vel, 0.5);
    nh.param(this_node::getName() + "/formation/form_shape", form_shape, 1.0);
    nh.param(this_node::getName() + "/formation/form_track", form_track, 1.0);
    nh.param(this_node::getName() + "/formation/form_decay", form_decay, 1.0);

    // init subscribers
    swarm_pose_sub = nh.subscribe("swarm_position_rel", queue_size, &flocking::swarm_pose_callback, this);
    swarm_pose_abs_sub = nh.subscribe("swarm_position", queue_size, &flocking::swarm_pose_abs_callback, this);
    swarm_vel_sub = nh.subscribe("swarm_velocity_rel", queue_size, &flocking::swarm_vel_callback, this);

    // init service clients
    area_client = nh.serviceClient<cpswarm_msgs::GetPoints>("area/get_area");
    area_client.waitForExistence();

    // init velocities and acceleration
    a_repulsion.x = 0;
    a_repulsion.y = 0;
    a_repulsion.z = 0;
    a_alignment.x = 0;
    a_alignment.y = 0;
    a_alignment.z = 0;
    v_flock.x = 0;
    v_flock.y = 0;
    v_flock.z = 0;
    a_wall.x = 0;
    a_wall.y = 0;
    a_wall.z = 0;
    v_formation.x = 0;
    v_formation.y = 0;
    v_formation.z = 0;

    // wait for initial swarm information
    Duration(start_delay).sleep();
    spinOnce();
}

geometry_msgs::Point flocking::center ()
{
    // get coordinates of other cpss
    geometry_msgs::Point center;
    for (auto pos : swarm_pos_abs) {
        center.x += pos.pose.position.x;
        center.y += pos.pose.position.y;
    }

    // add this cps
    center.x += pos.get_pose().position.x;
    center.y += pos.get_pose().position.y;

    // normalize
    center.x /= swarm_pos.size() + 1;
    center.y /= swarm_pos.size() + 1;

    return center;
}

geometry_msgs::Vector3 flocking::flocking_velocity (geometry_msgs::Vector3 velocity)
{
    // compute acceleration from repulsive forces between CPSs
    repulsion();

    // compute acceleration to align velocities between CPSs
    alignment();

    // compute velocity to move as flock
    flock(velocity);

    // compute acceleration to stay within area
    wall();

    // add up individual velocities and accelerations
    geometry_msgs::Vector3 vel_cur = vel.get_velocity();
    geometry_msgs::Vector3 vel_new;
    vel_new.x = vel_cur.x + 1 / accel_time * (v_flock.x - vel_cur.x) * dt + (a_repulsion.x + a_alignment.x + a_wall.x) * dt;
    vel_new.y = vel_cur.y + 1 / accel_time * (v_flock.y - vel_cur.y) * dt + (a_repulsion.y + a_alignment.y + a_wall.y) * dt;

    ROS_DEBUG_THROTTLE(2, "%.2f = cur %.2f + flock %.2f + repulse %.2f + align %.2f + wall %.2f", hypot(vel_new.x, vel_new.y), hypot(vel_cur.x, vel_cur.x), hypot(1 / accel_time * (v_flock.x - vel_cur.x) * dt, 1 / accel_time * (v_flock.y - vel_cur.y)), hypot(a_repulsion.x * dt, a_repulsion.y*dt), hypot(a_alignment.x * dt, a_alignment.y * dt), hypot(a_wall.x * dt, a_wall.y * dt));

    return vel_new;

}

geometry_msgs::Vector3 flocking::formation_velocity (geometry_msgs::Pose target)
{
    // compute acceleration from repulsive forces between CPSs
    repulsion();

    // compute acceleration to align velocities between CPSs
    alignment();

    // compute velocity required to achieve formation
    formation(target.position);

    // add up individual velocities and accelerations
    geometry_msgs::Vector3 vel_cur = vel.get_velocity();
    geometry_msgs::Vector3 vel_new;
    vel_new.x = vel_cur.x + 1 / accel_time * (v_formation.x - vel_cur.x) * dt + (a_repulsion.x + a_alignment.x + a_wall.x) * dt;
    vel_new.y = vel_cur.y + 1 / accel_time * (v_formation.y - vel_cur.y) * dt + (a_repulsion.y + a_alignment.y + a_wall.y) * dt;

    return vel_new;
}

void flocking::alignment ()
{
    // init alignment acceleration
    a_alignment.x = 0;
    a_alignment.y = 0;

    // compute damped velocity differences for all neighbors
    for (auto pose : swarm_pos) {
        // compute damping
        double damp = max(pose.vector.magnitude - (equi_dist - align_slope), align_min);
        damp *= damp;

        // get velocity of this neighbor
        cpswarm_msgs::Vector v = get_velocity(pose.swarmio.node);
        if (v.magnitude == 0.0 && v.direction == 0.0) // no velocity data found
            continue;

        // compute velocity difference
        double vx = v.magnitude * cos(v.direction);
        double vy = v.magnitude * sin(v.direction);

        // sum up damped velocity difference
        a_alignment.x += vx / damp;
        a_alignment.y += vy / damp;
    }

    // apply friction coefficient
    a_alignment.x *= align_frict;
    a_alignment.y *= align_frict;
}

double flocking::dist_bound()
{
    // get pose
    geometry_msgs::Pose pose = pos.get_pose();

    // get area coordinates
    cpswarm_msgs::GetPoints area;
    if (area_client.call(area) == false){
        ROS_ERROR("Failed to get area boundary");
        return 0.0;
    }

    // point where line from origin through pose intersects boundary
    geometry_msgs::Point p;

    // distance of point from origin
    double dist = 0.0;

    // coordinates of line segment from origin to pose
    double x1 = 0;
    double y1 = 0;
    double x2 = pose.position.x;
    double y2 = pose.position.y;

    // find boundary that yields closest intersection point (i.e. the correct boundary)
    for (int i=0; i<area.response.points.size(); ++i) {
        // coordinates of boundary
        double x3 = area.response.points[i].x;
        double y3 = area.response.points[i].y;
        double x4 = area.response.points[(i+1) % area.response.points.size()].x;
        double y4 = area.response.points[(i+1) % area.response.points.size()].y;

        // compute point based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
        p.x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
        p.y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

        // found closer point
        if (hypot(p.x, p.y) < dist || dist == 0.0)
            dist = hypot(p.x, p.y);
    }

    // return distance
    return dist;
}

void flocking::flock (geometry_msgs::Vector3 velocity)
{
    // current velocity
    double vel_mag = hypot(velocity.x, velocity.y);

    // get pose
    geometry_msgs::Pose pose = pos.get_pose();
    double pose_mag = hypot(pose.position.x, pose.position.y);

    // compute flocking velocity
    v_flock.x = flock_vel / vel_mag * velocity.x;
    v_flock.y = flock_vel / vel_mag * velocity.y;
}

void flocking::formation (geometry_msgs::Point target)
{
    // get pose
    geometry_msgs::Pose pose = pos.get_pose();

    // define shape position depending on formation
    geometry_msgs::Vector3 x_shp;
    double dist;

    // grid formation
    if (form == "grid") {
        x_shp.x = target.x;
        x_shp.y = target.y;
        // circle packing, use function fitted from data available at http://hydra.nat.uni-magdeburg.de/packing/cci/cci.html
        dist = equi_dist / 2 * 0.8135 * pow(swarm_pos.size(), -0.4775) - equi_dist / 2;
    }

    // ring formation
    else if (form == "ring") {
        // radius of ring
        double rad = equi_dist / 2.0 / sin(M_PI / swarm_pos.size());

        // two neighbors on ring
        cpswarm_msgs::VectorStamped n1,n2;
        // find closest neighbors
        for (auto n : swarm_pos) {
            if (n.vector.magnitude < n1.vector.magnitude || n1.vector.magnitude == 0) {
                n2 = n1;
                n1 = n;
            }
            else if (n.vector.magnitude < n2.vector.magnitude || n2.vector.magnitude == 0) {
                n2 = n;
            }
        }
        // get absolute positions
        geometry_msgs::Point n1_p, n2_p;
        for (auto n : swarm_pos_abs) {
            if (n.swarmio.node == n1.swarmio.node) {
                n1_p = n.pose.position;
            }
            if (n.swarmio.node == n2.swarmio.node) {
                n2_p = n.pose.position;
            }
            if ((n1_p.x != 0 || n1_p.y != 0) && (n2_p.x != 0 || n2_p.y != 0)) {
                break;
            }
        }

        // angle bisector: https://stackoverflow.com/questions/43435055/numerically-stable-angle-bisector-algorithm
        geometry_msgs::Vector3 bis;
        // vectors from center to neighbors
        geometry_msgs::Vector3 n1_v, n2_v;
        n1_v.x = n1_p.x - target.x;
        n1_v.y = n1_p.y - target.y;
        n2_v.x = n2_p.x - target.x;
        n2_v.y = n2_p.y - target.y;
        // normalize
        double n1_n = hypot(n1_v.x, n1_v.y);
        n1_v.x /= n1_n;
        n1_v.y /= n1_n;
        double n2_n = hypot(n2_v.x, n2_v.y);
        n2_v.x /= n2_n;
        n2_v.y /= n2_n;
        // dot product of vectors
        double dot = n1_v.x * n2_v.x + n1_v.y * n2_v.y;
        // obtuse angle needs special handling for numeric precision
        if (dot < 0) {
            // difference of neighbor vectors
            geometry_msgs::Vector3 bis_t;
            bis_t.x = n1_v.x - n2_v.x;
            bis_t.y = n1_v.y - n2_v.y;
            // normalize
            double bis_n = hypot(bis_t.x, bis_t.y);
            bis_t.x /= bis_n;
            bis_t.y /= bis_n;
            // determine direction
            double dir = n1_v.x * n2_v.y - n1_v.y * n2_v.x; // rotated n2_v by 90°
            // rotate by 90°
            if (dir < 0) {
                bis.x = bis_t.y;
                bis.y = -bis_t.x;
            }
            // rotate by 270°
            else {
                bis.x = -bis_t.y;
                bis.y = bis_t.x;
            }
        }
        // acute angle
        else {
            // sum of neighbor vectors
            bis.x = n1_v.x + n2_v.x;
            bis.y = n1_v.y + n2_v.y;
            // normalize
            double bis_n = hypot(bis.x, bis.y);
            bis.x /= bis_n;
            bis.y /= bis_n;
        }

        // intersect angle bisector and ring
        x_shp.x = target.x + bis.x * rad;
        x_shp.y = target.y + bis.y * rad;

        // no distance required
        dist = 0;
    }
    else if (form == "line") {
        x_shp.x = 0; // TODO
        x_shp.y = 0; // TODO

        ROS_FATAL("Flocking formation not yet implemented");

        // 1. do a linear regression through the target: https://stats.stackexchange.com/questions/12484/constrained-linear-regression-through-a-specified-point
        // 2. project each cps position on that line
        // 3. move cps positions on that line to have equilibrium spacing
        //    a. for odd number of cpss, one is directly above target
        //    b. for even number of cpss, two are 1/2 eq. dist. away
        //    c. cpss at the edge only have one neighbor

        // no distance required
        dist = 0;
    }
    else {
        ROS_FATAL("Unknown flocking formation");
        return;
    }
    double shp_mag = hypot(x_shp.x - pose.position.x, x_shp.y - pose.position.y);

    // compute shape velocity
    double tf_shp = transfer(shp_mag, dist, form_decay);
    geometry_msgs::Vector3 v_shp;
    if (shp_mag > 0.01) {
        v_shp.x = form_shape * form_vel * tf_shp * (x_shp.x - pose.position.x) / shp_mag;
        v_shp.y = form_shape * form_vel * tf_shp * (x_shp.y - pose.position.y) / shp_mag;
    }

    // compute distance between center of mass and target
    geometry_msgs::Vector3 x_com;
    x_com.x = target.x - center().x;
    x_com.y = target.y - center().y;
    double com_mag = hypot(x_com.x, x_com.y);

    // compute target tracking velocity
    double tf_track = transfer(com_mag, dist, form_decay);
    geometry_msgs::Vector3 v_trg;
    if (com_mag > 0.01) {
        v_trg.x = form_track * form_vel * tf_track * x_com.x / com_mag;
        v_trg.y = form_track * form_vel * tf_track * x_com.y / com_mag;
    }

    // combine velocities
    v_formation.x = v_shp.x + v_trg.x;
    v_formation.y = v_shp.y + v_trg.y;
    double vel_mag = hypot(v_formation.x, v_formation.y);
    if (vel_mag > form_vel) {
        v_formation.x *= form_vel / vel_mag;
        v_formation.y *= form_vel / vel_mag;
    }
}

cpswarm_msgs::Vector flocking::get_velocity (string uuid) const
{
    // search for cps with given uuid
    for (auto vel : swarm_vel) {
        if (vel.swarmio.node == uuid)
            return vel.vector;
    }

    // return empty vector if uuid is unknown
    return cpswarm_msgs::Vector();
}

void flocking::repulsion ()
{
    // init repulsive force acceleration
    a_repulsion.x = 0;
    a_repulsion.y = 0;

    // compute pair potentials for all neighbors
    for (auto pose : swarm_pos) {
        // repulsion only from close neighbors
        if (pose.vector.magnitude < equi_dist) {
            // compute pair potential
            double pot = min(repulse_max, equi_dist - pose.vector.magnitude) / pose.vector.magnitude;

            // sum up potentials
            a_repulsion.x += pot * pose.vector.magnitude * cos(pose.vector.direction);
            a_repulsion.y += pot * pose.vector.magnitude * sin(pose.vector.direction);
        }
    }

    // apply spring constant
    a_repulsion.x *= -repulse_spring;
    a_repulsion.y *= -repulse_spring;
}

double flocking::transfer (double x, double r, double d)
{
    if (x <= r) {
        return 0;
    }
    else if (r + d <= x) {
        return 1;
    }
    else {
        return 0.5 * sin(M_PI / d * (x - r) - M_PI / 2) + 0.5;
    }
}

void flocking::wall ()
{
    // get current velocity
    geometry_msgs::Vector3 velocity = vel.get_velocity();

    // get pose
    geometry_msgs::Pose pose = pos.get_pose();
    double pose_mag = hypot(pose.position.x, pose.position.y);

    // compute velocity requirement due to bounding area
    geometry_msgs::Vector3 v_wall;
    v_wall.x = - flock_vel * pose.position.x / pose_mag - velocity.x;
    v_wall.y = - flock_vel * pose.position.y / pose_mag - velocity.y;

    // compute transfer function for smooth movement
    double tf = transfer(pose_mag, dist_bound()-wall_decay, wall_decay); // subtract decay in order to stay within area bounds

    // total acceleration due to bounding area
    a_wall.x = wall_frict * tf * v_wall.x;
    a_wall.y = wall_frict * tf * v_wall.y;
}

void flocking::swarm_pose_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    swarm_pos = msg->vectors;
}

void flocking::swarm_pose_abs_callback (const cpswarm_msgs::ArrayOfPositions::ConstPtr& msg)
{
    swarm_pos_abs = msg->positions;
}

void flocking::swarm_vel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg)
{
    swarm_vel = msg->vectors;
}
