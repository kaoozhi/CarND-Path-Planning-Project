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

        struct prediction{bool found; double vel; double gap; double s; double d;};


        Vehicle();
        virtual ~Vehicle();
        double get_kinematics(const int& lane);
        void update(const double& s_, const double& v_, const double& d_, const double& x_, const double& y_, const double& yaw_, const int& prev_path_size_, const nlohmann::json& sensor_fusion_);
        Vehicle::prediction get_vehicle_ahead(const int& lane);
        Vehicle::prediction get_vehicle_behind(const int& lane);
        int get_best_lane();
        void cost_function();
        
        
};

#endif  // VEHICLE_H