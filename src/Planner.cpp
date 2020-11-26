//
// Created by shurtado on 11/26/2020.
//

#include "Planner.h"
void Planner::waypoint_planner(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                      const vector<double> &map_waypoints_s, double car_x, double car_y, double car_s, double car_yaw,
                               const vector<double> &previous_path_x, const vector<double> &previous_path_y, vector<double> &next_x_vals,
                      vector<double> &next_y_vals) {// Widely space (x,y) waypoints evenly
    tk::spline spline;
    int lane = 1;
    auto ref_velocity  =49.5;//mph

    vector<double> pts_x;
    vector<double> pts_y;

    double ref_x = car_x;
    double ref_y = car_y;

    double ref_yaw = deg2rad(car_yaw);

    if(previous_path_x.size() < 2){
        double last_x = car_x-cos(ref_yaw);
        double last_y = car_y-sin(ref_yaw);

        pts_x.push_back(last_x);
        pts_x.push_back(car_x);

        pts_y.push_back(last_y);
        pts_y.push_back(car_y);
    }else{
        ref_x = previous_path_x[previous_path_x.size() - 1];
        ref_y = previous_path_y[previous_path_x.size() - 1];

        double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
        double ref_y_prev = previous_path_y[previous_path_x.size() - 2];

        ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
        pts_x.push_back(ref_x_prev);
        pts_x.push_back(ref_x);

        pts_y.push_back(ref_y_prev);
        pts_y.push_back(ref_y);
    }

    auto next_wp_0 = getXY(car_s + 30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    auto next_wp_1 = getXY(car_s + 60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    auto next_wp_2 = getXY(car_s + 90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

    pts_x.push_back(next_wp_0[0]);
    pts_x.push_back(next_wp_1[0]);
    pts_x.push_back(next_wp_2[0]);

    pts_y.push_back(next_wp_0[1]);
    pts_y.push_back(next_wp_1[1]);
    pts_y.push_back(next_wp_2[1]);

    // transformation to local car coordinates
    for(size_t i = 0;i<pts_x.size();i++){

        auto shift_x = pts_x[i]-ref_x;
        auto shift_y = pts_y[i]-ref_y;

        pts_x[i] = shift_x *cos(-ref_yaw) - shift_y*sin(-ref_yaw);
        pts_y[i] = shift_x *sin(-ref_yaw) + shift_y*cos(-ref_yaw);
    }

    spline.set_points(pts_x,pts_y);
    for(size_t i =0 ; i < previous_path_x.size(); i++){
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    auto target_x = 30.0;
    auto target_y = spline(target_x);
    auto target_dist = sqrt(target_x*target_x+target_y*target_y);

    double x_add_on=0;

    for(size_t i =1;i<=50-previous_path_x.size();i++){
        // from miles to mts/s
        auto N = target_dist/(0.02*ref_velocity/2.24);
        auto x_point = x_add_on+(target_x)/N;
        auto y_point = spline(x_point);
        x_add_on=x_point;
        double x_ref = x_point;
        double y_ref = y_point;
        // Rotate back

        x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

        x_point+=ref_x;
        y_point+=ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}