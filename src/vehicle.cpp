#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>

const double SPEED_LIMIT = 49.5; //mph
const double SPEED_INCREMENT = .224*0.9; //mph/0.02s->5m/s2
const double SPEED_INCREMENT_LIMIT = .224; //mph/0.02s->10m/s2
const double BUFFER_DISTANCE_AHEAD = 35; //m
const double BUFFER_DISTANCE_BEHIND = 10; //m
const double SECURITE_DISTANCE = 15; //m
const double SECURITE_D_MARGIN = 1.25;

Vehicle::Vehicle(){

    this->lane = 1.0;
    this->s = 0.0;
    this->v = 0.0;
    this->d = 1.0;
    this->x = 0.0;
    this->y = 0.0;
    this->yaw =0.0;
    this->prev_path_size = 0.0;
    // this->state = Vehicle::FSM::CS;
};

Vehicle::~Vehicle(){};

void Vehicle::update(const double& s_, const double& v_, const double& d_, const double& x_, const double& y_, const double& yaw_, const int& prev_path_size_, const nlohmann::json& sensor_fusion_){
    // this->lane = 1;
    this->s = s_;
    // this->v = v_;
    // this->prev_d = this->d;
    this->d = d_;
    this->x = x_;
    this->y = y_;
    this->yaw = yaw_;
    // this->prev_lane = this->lane;
    
    this->lane = floor(d_/4);
    this->prev_path_size = prev_path_size_;
    this->sensor_fusion = sensor_fusion_;
    // this->get_keep_lane_duration();
};

 
Vehicle::prediction Vehicle::get_vehicle_ahead(const int& lane){
    double min_gap = std::numeric_limits<double>::max();
    double vehicle_ahead_speed, vehicle_ahead_s, vehicle_ahead_d;
    bool vehicle_found =false;
    for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
    {
        float d_ = this->sensor_fusion[jj][6];
        double check_car_s_= this->sensor_fusion[jj][5];
        // vehicle ahead entering into the current lane is considered in the current lane for security resason 
        if (d_>0 && this->s <= check_car_s_ && (floor(d_/4) == lane || fabs(d_-lane*4) <= SECURITE_D_MARGIN || fabs(d_-(lane*4+4)) <= SECURITE_D_MARGIN)) 
        {
            // check if vechicle is too close behind in the current lane
            double vx_ = this->sensor_fusion[jj][3];
            double vy_ = this->sensor_fusion[jj][4];
            double x_ = this->sensor_fusion[jj][0];
            double y_ = this->sensor_fusion[jj][1];

            double check_speed_= sqrt(vx_*vx_+vy_*vy_); // m/s
            
            if ((check_car_s_-this->s) <min_gap)
            {
                min_gap = check_car_s_ - this->s;
                vehicle_ahead_speed = check_speed_*2.24; //mph
                vehicle_ahead_s = check_car_s_;
                vehicle_ahead_d = d_;
                vehicle_found =true;
            }
        }
    }

    Vehicle::prediction vehicle_ahead{vehicle_found,vehicle_ahead_speed, min_gap, vehicle_ahead_s, vehicle_ahead_d};
    return vehicle_ahead;

};


Vehicle::prediction Vehicle::get_vehicle_behind(const int& lane){
    // struct vehicle_behind{bool found; double vel; double gap;};
    double min_gap = std::numeric_limits<double>::max();
    double vehicle_behind_speed, vehicle_behind_s, vehicle_behind_d;
    bool vehicle_found =false;
    for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
    {
        float d_ = this->sensor_fusion[jj][6];
        double check_car_s_= this->sensor_fusion[jj][5];
        // d_
        if (d_>0 && this->s > check_car_s_ && (floor(d_/4) == lane || fabs(d_-lane*4)<SECURITE_D_MARGIN || fabs(d_-(lane*4+4)) <SECURITE_D_MARGIN)) 
        {
            // check if vechicle is too close behind in the current lane
            double vx_ = this->sensor_fusion[jj][3];
            double vy_ = this->sensor_fusion[jj][4];
            double x_ = this->sensor_fusion[jj][0];
            double y_ = this->sensor_fusion[jj][1];

            double check_speed_= sqrt(vx_*vx_+vy_*vy_); //m/s
            

            // check_car_s_ += double(this->prev_path_size) *.02*check_speed_;
            if ((this->s-check_car_s_) <min_gap)
            {
                min_gap = this->s-check_car_s_;
                vehicle_behind_speed = check_speed_*2.24; //mph
                vehicle_behind_s = check_car_s_;
                vehicle_behind_d = d_;
                vehicle_found =true;
            }
        }
    }

    // if (min_gap < BUFFER_DISTANCE)
    // {
        // return vehicle_behind{vehicle_found,vehicle_behind_speed, min_gap};
        Vehicle::prediction vehicle_behind{vehicle_found,vehicle_behind_speed, min_gap, vehicle_behind_s, vehicle_behind_d};
        return vehicle_behind;
};

double Vehicle::get_kinematics(const int& lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    // std::cout<<"current_speed"<<this->v<<std::endl;
    double vel_accel = this->v + SPEED_INCREMENT;
    double vel_decel = this->v - SPEED_INCREMENT;
    double max_vel_accel_limit = this->v + SPEED_INCREMENT_LIMIT;
    double max_vel_decel_limit = this->v - SPEED_INCREMENT_LIMIT*1.2;
    double new_vel;
    auto vehicle_ahead = get_vehicle_ahead(lane);
    auto vehicle_behind = get_vehicle_behind(lane);
    // double gap_threshold = 20;
    // double security_gap_threshold =10;

    // Consider closet vehicle ahead or behind within the buffer distance
    if (vehicle_ahead.gap<BUFFER_DISTANCE_AHEAD) 
    {
        // if vehicle behind found

        if (vehicle_ahead.gap<SECURITE_DISTANCE)
            {
                // new_vel = std::max(std::min(vel_decel, vehicle_ahead.vel), max_vel_decel_limit);
                new_vel = max_vel_decel_limit;
                // new_vel = std::max(max_vel_decel_limit,vehicle_ahead.vel -SPEED_INCREMENT);
            }
        else if (vehicle_behind.gap < BUFFER_DISTANCE_BEHIND) 
            {

                // else
                // {
                if (this->v < vehicle_behind.vel)
                {
                    new_vel = std::min(vel_accel, vehicle_behind.vel);
                }
                else
                {
                    new_vel = this->v;
                }
                    // new_vel = std::min(vehicle_ahead.vel, vel_accel); //must travel at the speed of traffic, regardless of buffer with respect to minimum security distance
                // }
                
            } 

            else
            {
                if (this->v < vehicle_ahead.vel)
                {
                    new_vel = std::min(vel_accel, vehicle_ahead.vel);
                }
                else
                {
                    new_vel = std::max(vel_decel, vehicle_ahead.vel);
                }
                
                // new_vel = std::max(vel_decel, vehicle_ahead.vel);
            }
                // }
                // else
                // {
                // new_vel = std::max(std::min(vel_decel, vehicle_ahead.vel), max_vel_decel_limit);
                // new_vel = std::max(vel_decel, vehicle_ahead.vel);
                // }
                
                // std::cout<<"Vehicle ahead, keep distance speed at:"<<new_vel<<", gap:"<<vehicle_ahead.gap<<std::endl;

    }
    else 
    {
        new_vel = std::min(vel_accel, SPEED_LIMIT);
        // std::cout<<"No vehicle ahead, speed at:"<<new_vel<<", gap:"<<vehicle_ahead.gap<<std::endl;
    }

    // std::cout<<"Vehicle behind, at"<<vehicle_ahead.gap<<"m"<<std::endl;

    return std::max(new_vel,3.0);
    // return new_vel;
    
}

// bool Vehicle::vehicle_close_around(int lane)
// {
//     // double min_gap = std::numeric_limits<double>::max();

//     // for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
//     // {
//     // float d_ = this->sensor_fusion[jj][6];

//     //     if (floor(d_/4) == lane)
//     //     {
//     //         // check if vechicle is too close behind in the current lane
//     //         double vx_ = this->sensor_fusion[jj][3];
//     //         double vy_ = this->sensor_fusion[jj][4];
//     //         double x_ = this->sensor_fusion[jj][0];
//     //         double y_ = this->sensor_fusion[jj][1];

//     //         double check_speed_= sqrt(vx_*vx_+vy_*vy_);
//     //         double check_car_s_= this->sensor_fusion[jj][5];

//     //         // check_car_s_ += double(this->prev_path_size) *.02*check_speed_;
//     //         if (abs(check_car_s_-this->s) <min_gap)
//     //         {
//     //         min_gap = abs(check_car_s_ - this->s);
//     //         }
//     //     }
//     // }
//     Vehicle::prediction vehicle_behind = this->get_vehicle_behind(lane);
//     Vehicle::prediction vehicle_ahead = this->get_vehicle_behind(lane);

//     if (vehicle_ahead.found) return (vehicle_ahead.gap < 5);
//     else {return false;}
    

// }

// bool Vehicle::check_collision(int lane)
// {


//     double min_gap = 5;
//     bool collision = false;
//     Vehicle::prediction vehicle_behind = this->get_vehicle_behind(lane);
//     if (vehicle_behind.found)
//     {
//             double vehicle_behind_s_pred = double(this->prev_path_size) *.02*vehicle_behind.vel/2.24 + vehicle_behind.s;
//             double ego_car_s_pred = this->s + double(this->prev_path_size) *.02*this->v/2.24;
//             if ((ego_car_s_pred - vehicle_behind_s_pred) < min_gap)
//             {
//                 collision = true;
//             }

//     }

//     return collision;

// }




// Vehicle::prediction Vehicle::get_vehicle_ahead_debug(const int& lane)
// {
//     double min_gap = std::numeric_limits<double>::max();
//     double vehicle_ahead_speed, vehicle_ahead_s, vehicle_ahead_d;
//     bool vehicle_found =false;
//     for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
//     {
//         float d_ = this->sensor_fusion[jj][6];
//         double check_car_s_= this->sensor_fusion[jj][5];
//         // vehicle ahead entering into the current lane is considered in the current lane for security resason 
//         if (d_>0 && this->s < check_car_s_ && (floor(d_/4) == lane || fabs(d_-lane*4)<SECURITE_D_MARGIN || fabs(d_-(lane*4+4)) <SECURITE_D_MARGIN)) 
//         {
//             // check if vechicle is too close behind in the current lane
//             std::cout<<"vehicle ahead sensed by fusion: "<<d_<<std::endl;
//             double vx_ = this->sensor_fusion[jj][3];
//             double vy_ = this->sensor_fusion[jj][4];
//             double x_ = this->sensor_fusion[jj][0];
//             double y_ = this->sensor_fusion[jj][1];

//             double check_speed_= sqrt(vx_*vx_+vy_*vy_); // m/s
            
//             if ((check_car_s_-this->s) <min_gap)
//             {
//                 min_gap = check_car_s_ - this->s;
//                 vehicle_ahead_speed = check_speed_*2.24; //mph
//                 vehicle_ahead_s = check_car_s_;
//                 vehicle_ahead_d = d_;
//                 vehicle_found =true;
//             }
//         }
//     }

//     Vehicle::prediction vehicle_ahead{vehicle_found,vehicle_ahead_speed, min_gap, vehicle_ahead_s, vehicle_ahead_d};
//     return vehicle_ahead;

// };



