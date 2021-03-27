#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include <string>
#include <vector>
#include "json.hpp"
// #include "spline.h"
#include <map>
// #include "helpers.h"
#include "vehicle.h"

using namespace std;

class Planner{


    public:

        enum FSM {CS,KL,PLCL,LCL,PLCR,LCR};
        FSM state;
        FSM prev_state;

        vector<double> previous_path_x;
        vector<double> previous_path_y;
        vector<double> ptsx;
        vector<double> ptsy;

        vector<double> maps_x;
        vector<double> maps_y;
        vector<double> maps_s;

        Vehicle ego_car;
        int prev_path_size;

        int prev_target_lane;
        int target_lane;


        double end_path_s;
        double end_path_d;

        double ref_x;
        double ref_y;
        double ref_yaw;

        struct path
        {
        vector<vector<double>> trajectory; 
        double real_speed; double target_speed; 
        int target_lane;
        };

        Planner(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
        virtual ~Planner();
        void update(vector<double> previous_path_x_, vector<double> previous_path_y_, double end_path_s_, double end_path_d_,double s_, double v_, double d_, double x_, double y_, double yaw_, nlohmann::json sensor_fusion_);
        void init_path();
        vector<vector<double>> create_spoints(int lane,double dist_add);
        vector<vector<double>> create_trajectory(int lane, double ref_vel,double dist_add);
        void realize_next_state(FSM new_state);
        vector<Planner::FSM> successor_states();

        
        // vector<vector<double>> prep_lane_change_trajectory(FSM new_state);
        Planner::path keep_lane_path();
        Planner::path prep_lane_change_path(Planner::FSM new_state);
        Planner::path lane_change_path(Planner::FSM new_state);
        Planner::path generate_candidate_path(Planner::FSM new_state);

        Planner::path find_next_path();
        bool lane_change_safe(Planner::FSM new_state);
        double efficiency_cost(double real_speed, double target_speed);
        double traffic_cost(Planner::FSM new_state);
        void get_keep_lane_duration();
        int keep_lane_duration = 0; 
        int lane_change_on_duration =0;
        void get_lane_change_on_delai();
    // private:

};

#endif  // VEHICLE_H