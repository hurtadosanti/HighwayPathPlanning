//
// Created by shurtado on 11/26/2020.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "spline.h"
#include "math.h"
#include <iostream>

using std::vector;

/**
 * Plans vehicle motion on a highway given a mat and sensor fusion data
 */
class planner {
public:
    /**
    * Plans a path from Map coordinates to Frenet
     */
    void static waypoint_planner(double &ref_velocity, int &lane, const vector<double> &map_waypoints_x,
                                 const vector<double> &map_waypoints_y,
                                 const vector<double> &map_waypoints_s, double car_x, double car_y, double car_s,
                                 double car_yaw,
                                 const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                 vector<double> &next_x_vals,
                                 vector<double> &next_y_vals, double end_path_s, vector<vector<double>> sensor_fusion);

private:
    /**
     * Calculates if there is a vehicle on the way and plan if is possible to change lanes
     */
    static void
    change_lanes(double car_s, const vector<double> &previous_path_x, const vector<vector<double>> &sensor_fusion,
                 double &ref_velocity, int &lane);

    /**
     * Convert from degrees to radians
     */
    double static deg2rad(double x) { return x * M_PI / 180; }

    /**
     * Convert from Frenet to Map coordinates
     * */
    vector<double> static getXY(double s, double d, const vector<double> &maps_s,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y) {
        int prev_wp = -1;

        while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
            ++prev_wp;
        }

        int wp2 = (prev_wp + 1) % maps_x.size();

        double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                               (maps_x[wp2] - maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s - maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
        double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

        double perp_heading = heading - M_PI / 2;

        double x = seg_x + d * cos(perp_heading);
        double y = seg_y + d * sin(perp_heading);

        return {x, y};
    }


};


#endif //PATH_PLANNING_PLANNER_H
