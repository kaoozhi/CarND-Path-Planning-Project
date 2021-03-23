#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
// #include "helpers.h"
// #include "cost.h"
const double SPEED_LIMIT = 49.5; //mph
const double SPEED_INCREMENT = .224; //mph/0.02s->5m/s2
const double SPEED_INCREMENT_LIMIT = .224*1.2; //mph/0.02s->10m/s2
const double BUFFER_DISTANCE_AHEAD = 35; //m
const double BUFFER_DISTANCE_BEHIND = 20; //m
const double SECURITE_DISTANCE = 10; //m
// const double EFFICIENCY = 0.7;

Vehicle::Vehicle(){

    this->lane = 1;
    this->s = 0;
    this->v = 0;
    this->d = 1;
    this->x = 0;
    this->y = 0;
    this->yaw =0;
    this->prev_path_size = 0;
    // this->state = Vehicle::FSM::CS;
};

Vehicle::~Vehicle(){};

void Vehicle::update(double s_, double v_, double d_, double x_, double y_, double yaw_, int prev_path_size_, nlohmann::json sensor_fusion_){
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

// void Vehicle::next_state(FSM new_state)
// {
//     this->prev_state = state;
//     this->state = new_state;
// };

// int Vehicle::get_keep_lane_duration()
// {
//     if (this->state==Vehicle::FSM::KL && this->prev_state==Vehicle::FSM::KL)
//     {
//         return (this->keep_lane_duration +=1);
//     }
//     else
//     {
//         return  (this->keep_lane_duration =0);
//     }
    
// };


 
Vehicle::prediction Vehicle::get_vehicle_ahead(int lane){
    // struct vehicle_ahead{bool found; double vel; double gap;};
    double min_gap = std::numeric_limits<double>::max();
    double vehicle_ahead_speed;
    bool vehicle_found =false;
    for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
    {
    float d_ = this->sensor_fusion[jj][6];

    if (floor(d_/4) == lane)
    {
        // check if vechicle is too close behind in the current lane
        double vx_ = this->sensor_fusion[jj][3];
        double vy_ = this->sensor_fusion[jj][4];
        double x_ = this->sensor_fusion[jj][0];
        double y_ = this->sensor_fusion[jj][1];

        double check_speed_= sqrt(vx_*vx_+vy_*vy_);
        double check_car_s_= this->sensor_fusion[jj][5];

        // check_car_s_ += double(this->prev_path_size) *.02*check_speed_;
        if ((check_car_s_-this->s) <min_gap && this->s < check_car_s_)
        {
        min_gap = check_car_s_ - this->s;
        vehicle_ahead_speed = check_speed_*2.24;
        vehicle_found =true;
        }
    }
    }

    // if (min_gap < BUFFER_DISTANCE)
    Vehicle::prediction vehicle_ahead{vehicle_found,vehicle_ahead_speed, min_gap};
    // {
        // return {vehicle_found,vehicle_ahead_speed, min_gap};
    return vehicle_ahead;
    // }
    // else
    // {
    //     return vehicle_ahead{false,SPEED_LIMIT};
    // }
};


Vehicle::prediction Vehicle::get_vehicle_behind(int lane){
    // struct vehicle_behind{bool found; double vel; double gap;};
    double min_gap = std::numeric_limits<double>::max();
    double vehicle_behind_speed;
    bool vehicle_found =false;
    for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
    {
    float d_ = this->sensor_fusion[jj][6];

    if (floor(d_/4) == lane)
    {
        // check if vechicle is too close behind in the current lane
        double vx_ = this->sensor_fusion[jj][3];
        double vy_ = this->sensor_fusion[jj][4];
        double x_ = this->sensor_fusion[jj][0];
        double y_ = this->sensor_fusion[jj][1];

        double check_speed_= sqrt(vx_*vx_+vy_*vy_);
        double check_car_s_= this->sensor_fusion[jj][5];

        // check_car_s_ += double(this->prev_path_size) *.02*check_speed_;
        if ((this->s-check_car_s_) <min_gap && this->s > check_car_s_)
        {
        min_gap = this->s-check_car_s_;
        vehicle_behind_speed = check_speed_*2.24;
        vehicle_found =true;
        }
    }
    }

    // if (min_gap < BUFFER_DISTANCE)
    // {
        // return vehicle_behind{vehicle_found,vehicle_behind_speed, min_gap};
        Vehicle::prediction vehicle_behind{vehicle_found,vehicle_behind_speed, min_gap};
        return vehicle_behind;
};

double Vehicle::get_kinematics(int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    // std::cout<<"current_speed"<<this->v<<std::endl;
    double vel_accel = this->v + SPEED_INCREMENT;
    double vel_decel = this->v - SPEED_INCREMENT;
    double max_vel_accel_limit = this->v + SPEED_INCREMENT_LIMIT;
    double max_vel_decel_limit = this->v - SPEED_INCREMENT_LIMIT;
    double new_vel;
    auto vehicle_ahead = get_vehicle_ahead(lane);
    auto vehicle_behind = get_vehicle_behind(lane);
    // double gap_threshold = 20;
    // double security_gap_threshold =10;

    // Consider closet vehicle ahead or behind within the buffer distance
    if (vehicle_ahead.found && vehicle_ahead.gap<BUFFER_DISTANCE_AHEAD) 
    {
        // if vehicle behind found

        if (vehicle_ahead.gap<SECURITE_DISTANCE)
            {
                // new_vel = std::max(std::min(vel_decel, vehicle_ahead.vel), max_vel_decel_limit); 
                // new_vel = std::max(std::min(vel_decel, vehicle_ahead.vel), max_vel_decel_limit);
                new_vel = max_vel_decel_limit;
                // new_vel = std::max(max_vel_decel_limit,vehicle_ahead.vel -SPEED_INCREMENT);
            }
        else if (vehicle_behind.found && vehicle_behind.gap < BUFFER_DISTANCE_BEHIND) 
            {

                // else
                // {
                    new_vel = std::min(vehicle_ahead.vel, vel_accel); //must travel at the speed of traffic, regardless of buffer with respect to minimum security distance
                // }
                
            } 
            // else if (vehicle_ahead.gap<SECURITE_DISTANCE)
            // {
            //     // if (this->v > vehicle_ahead.vel) 
            //     new_vel = max_vel_decel_limit;
            // }
            //     // {
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

    return std::max(new_vel,7.0);
    
}

bool Vehicle::vehicle_close_around(int lane)
{
    double min_gap = std::numeric_limits<double>::max();

    for (auto jj=0; jj<this->sensor_fusion.size(); jj++)
    {
    float d_ = this->sensor_fusion[jj][6];

        if (floor(d_/4) == lane)
        {
            // check if vechicle is too close behind in the current lane
            double vx_ = this->sensor_fusion[jj][3];
            double vy_ = this->sensor_fusion[jj][4];
            double x_ = this->sensor_fusion[jj][0];
            double y_ = this->sensor_fusion[jj][1];

            double check_speed_= sqrt(vx_*vx_+vy_*vy_);
            double check_car_s_= this->sensor_fusion[jj][5];

            // check_car_s_ += double(this->prev_path_size) *.02*check_speed_;
            if (abs(check_car_s_-this->s) <min_gap)
            {
            min_gap = abs(check_car_s_ - this->s);
            }
        }
    }

    return (min_gap >10);

}








