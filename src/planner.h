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
            double real_speed; 
            double target_speed; 
            int target_lane;
        };

        Planner(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
        virtual ~Planner();
        void update(const vector<double>& previous_path_x_, const vector<double>& previous_path_y_, const double& end_path_s_, const double& end_path_d_, const double& s_, const double& v_, const double& d_, const double& x_, const double& y_, const double& yaw_, const nlohmann::json& sensor_fusion_);
        Planner::path find_next_path();

    private:
        void init_path();
        vector<vector<double>> create_spoints(const int& lane, const double& dist_add);
        vector<vector<double>> create_trajectory(const int& lane, const double& ref_vel, const double& dist_add);
        void realize_next_state(FSM new_state);
        vector<Planner::FSM> successor_states();
        Planner::path keep_lane_path();
        Planner::path prep_lane_change_path(const Planner::FSM& new_state);
        Planner::path lane_change_path(const Planner::FSM& new_state);
        Planner::path generate_candidate_path(const Planner::FSM& new_state);
        
        bool lane_change_safe(const Planner::FSM& new_state);
        double efficiency_cost(const double& real_speed, const double& target_speed);
        double traffic_cost(const Planner::FSM& new_state);
        double maneuver_cost(const Planner::FSM& new_state);
        double reach_goal_cost(const Planner::FSM& new_state);
        void get_keep_lane_duration();
        int keep_lane_duration = 0; 
        int lane_change_on_duration =0;
        void get_lane_change_on_delai();

};

#endif