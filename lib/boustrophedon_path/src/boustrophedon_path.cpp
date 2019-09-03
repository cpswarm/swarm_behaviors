#include "boustrophedon_path.h"

boustrophedon_path::boustrophedon_path ()
{
    // read parameters
    double altitude;
    nh.param(this_node::getName() + "/altitude", altitude, 1.5);
    double fov;
    nh.param(this_node::getName() + "/fov", fov, 1.2);
    double overlap;
    nh.param(this_node::getName() + "/overlap", overlap, 0.0);

    // optimal distance between paths according to covered fov
    this->fov = altitude * tan(fov / 2) * 2 - overlap;

    // area to cover
    ServiceClient area_client = nh.serviceClient<cpswarm_msgs::get_area>("area/get_area");
    area_client.waitForExistence();
    cpswarm_msgs::get_area ga;
    if (area_client.call(ga)) {
        area = ga.response.area;
    }
    else {
        ROS_ERROR("Failed to get area coordinates");
        return;
    }

    // one intermediate waypoint between parallel paths
    wp_perp = 2;

    // find bounding area bound closest to current position
    ServiceClient bound_client = nh.serviceClient<cpswarm_msgs::closest_bound>("area/closest_bound");
    cpswarm_msgs::closest_bound cb;
    if (bound_client.call(cb)) {
        closest_bound.first = cb.response.index[0];
        closest_bound.second = cb.response.index[1];
    }
    else {
        ROS_ERROR("Failed to get area bound");
        return;
    }

    // find spacing between neighboring paths
    find_spacing();

    // find directions parallel and perpendicular to closest bound
    find_directions();

    // find starting point
    find_start();

    // generate path
    generate_path();

    // start at waypoint zero
    wp = 0;
}

boustrophedon_path::~boustrophedon_path ()
{
}

geometry_msgs::Point boustrophedon_path::current_wp ()
{
    return get_wp(wp);
}

geometry_msgs::Point boustrophedon_path::next_wp ()
{
    return get_wp(++wp);
}

geometry_msgs::Point boustrophedon_path::previous_wp ()
{
    return get_wp(wp-1);
}

void boustrophedon_path::step ()
{
    ++wp;
}

void boustrophedon_path::find_directions ()
{
    // parallel directions
    int i = closest_bound.first;
    int j = closest_bound.second;
    dir_par_right = angle(atan2(area[j].y - area[i].y, area[j].x - area[i].x));
    dir_par_left = dir_par_right + M_PI;

    // outward and inward directions (perpendicular to closest bound)
    i = closest_bound.second;
    j = (closest_bound.second + 1) % area.size();
    dir_perp_out = angle(atan2(area[j].y - area[i].y, area[j].x - area[i].x));
    dir_perp_in = dir_perp_out + M_PI;

    ROS_DEBUG("Directions: right %.2f, left %.2f, up %.2f, down %.2f", dir_par_right, dir_par_left, dir_perp_out, dir_perp_in);
}

void boustrophedon_path::find_spacing ()
{
    // TODO: spacing according to swarm size
    // width of area (length of parallel bound)
    int i = closest_bound.first;
    int j = closest_bound.second;
    double width = hypot(area[j].x - area[i].x, area[j].y - area[i].y);

    ROS_DEBUG("Width of area: %.2f", width);

    // height of area (length of perpendicular bound)
    i = closest_bound.second;
    j = (closest_bound.second + 1) % area.size();
    double height = hypot(area[j].x - area[i].x, area[j].y - area[i].y);

    ROS_DEBUG("Height of area: %.2f", height);

    // compute number of turns to be even
    turns = int(ceil(height / fov));
    if (turns % 2 != 0)
        turns += 1;

    ROS_DEBUG("Number of turns: %d", turns);

    // actual distance between paths
    path_spacing = height / double(turns);

    ROS_DEBUG("Path spacing: %.2f", path_spacing);

    // waypoint spacing on perpendicular paths
    wp_spacing_perp = path_spacing / double(wp_perp);

    ROS_ERROR("Perpendicular waypoint spacing: %.2f", wp_spacing_perp);

    // waypoint spacing on parallel paths (at most the spacing as on perpendicular paths)
    wp_par = int(ceil((width / 2.0 - path_spacing) / wp_spacing_perp));
    wp_spacing_par = (width / 2.0 - path_spacing) / double(wp_par);

    ROS_ERROR("Parallel waypoint spacing: %.2f", wp_spacing_par);
}

void boustrophedon_path::find_start ()
{
    geometry_msgs::Point start;

    // center of closest bound
    start.x = area[closest_bound.first].x + (area[closest_bound.second].x - area[closest_bound.first].x) / 2.0;
    start.y = area[closest_bound.first].y + (area[closest_bound.second].y - area[closest_bound.first].y) / 2.0;

    // back off from bound
    start.x += path_spacing / 2.0 * cos(dir_perp_out.rad());
    start.y += path_spacing / 2.0 * sin(dir_perp_out.rad());

    path.push_back(start);
}

void boustrophedon_path::generate_path ()
{
    // starting waypoint
    geometry_msgs::Point wp = path.back();
    wp.x += wp_spacing_par * cos(dir_par_right.rad());
    wp.y += wp_spacing_par * sin(dir_par_right.rad());
    path.push_back(wp);

    // outward path
    for (int turn = 0; turn < turns; ++turn) {
        // perpendicular path
        if (turn > 0) {
            for (int i = 0; i < wp_perp; ++i) {
                wp.x += wp_spacing_perp * cos(dir_perp_out.rad());
                wp.y += wp_spacing_perp * sin(dir_perp_out.rad());
                path.push_back(wp);
            }
        }

        // select direction
        angle dir = dir_par_right;
        if (turn % 2)
            dir = dir_par_left;

        // parallel path
        for (int i = 0; i < wp_par; ++i) {
            // compute next waypoint
            wp.x += wp_spacing_par * cos(dir.rad());
            wp.y += wp_spacing_par * sin(dir.rad());
            path.push_back(wp);
        }
    }

    // inward path
    for (int turn = 0; turn < turns; ++turn) {
        // perpendicular path
        if (turn > 0) {
            for (int i = 0; i < wp_perp; ++i) {
                wp.x += wp_spacing_perp * cos(dir_perp_in.rad());
                wp.y += wp_spacing_perp * sin(dir_perp_in.rad());
                path.push_back(wp);
            }
        }

        // select direction
        angle dir = dir_par_left;
        if (turn % 2)
            dir = dir_par_right;

        // parallel path
        for (int i = 0; i < wp_par; ++i) {
            // compute next waypoint
            wp.x += wp_spacing_par * cos(dir.rad());
            wp.y += wp_spacing_par * sin(dir.rad());
            path.push_back(wp);
        }
    }

    // final waypoint
    wp = path.front();
    path.push_back(wp);
}

geometry_msgs::Point boustrophedon_path::get_wp (int idx)
{
    geometry_msgs::Point waypoint;

    if (0 <= idx && idx < path.size()) {
        waypoint = path[idx];
    }

    return waypoint;
}
