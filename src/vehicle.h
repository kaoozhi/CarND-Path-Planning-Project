#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <string>
#include <vector>
#include "json.hpp"
// #include "spline.h"
#include <map>
// #include "helpers.h"
// #include "FSM.h"

class Vehicle{
    public:

        // enum FSM {CS,KL,PLCL,LCL,PLCR,LCR};
        // FSM state;
        // FSM prev_state;
        // };
        
        double s;
        double v;
        double d;
        double x;
        double y;
        double yaw;
        int prev_path_size;
        int lane;
        double prev_d;
        
        nlohmann::json sensor_fusion;

        struct prediction{bool found; double vel; double gap;};


        // minimum duration to keep in current lane before lane change
        // Vehicle(double s_, double v_, double d_, double x_, double y_, double yaw_);
        Vehicle();
        virtual ~Vehicle();
        double get_kinematics(int lane);
        void update(double s_, double v_, double d_, double x_, double y_, double yaw_, int prev_path_size_, nlohmann::json sensor_fusion_);
        double get_ahead_speed(int lane);
        Vehicle::prediction get_vehicle_ahead(int lane);
        Vehicle::prediction get_vehicle_behind(int lane);
        bool vehicle_close_around(int lane);

        int get_best_lane();
        // int get_current_lane();
        // void successor_states();
        // void next_state(FSM new_state);
        void cost_function();
        
        // void get_keep_lane_duration();
        // int keep_lane_duration = 0; 
    // private:
        // FSM state;
        
        


};

#endif  // VEHICLE_H