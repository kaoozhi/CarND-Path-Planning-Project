#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "helpers.h"
#include "planner.h"
#include "spline.h"


const map<Planner::FSM, int> lane_direction = {{Planner::FSM::PLCL, -1},{Planner::FSM::LCL, -1},{Planner::FSM::PLCR, 1},{Planner::FSM::LCR, 1}};

Planner::Planner(const vector<double> &maps_s_, const vector<double> &maps_x_, const vector<double> &maps_y_)
{

    // this->lane = 0;
    this->previous_path_x = {};
    this->previous_path_y = {};
    this->end_path_d =2+4*0;
    this->end_path_s =0;
    this->prev_path_size = this->previous_path_x.size();

    this->maps_x = maps_x_;
    this->maps_y = maps_y_;
    this->maps_s = maps_s_;
    this->state = Planner::FSM::KL;
    this->ptsx = {};
    this->ptsy = {};
    // this-> ref_x = 0;
    // this-> ref_y = 0;
    // this-> ref_yaw =0;
        // vector<double> previous_path_y;
        // vector<double> ptsx;
        // vector<double> ptsy;
        // int prev_path_size;

        // double end_path_s;
        // double end_path_d;

        // double ref_x;
        // double ref_y;
        // double ref_yaw;

};

Planner::~Planner(){};

void Planner::update(vector<double> previous_path_x_, vector<double> previous_path_y_, double end_path_s_, double end_path_d_,double s_, double v_, double d_, double x_, double y_, double yaw_, nlohmann::json sensor_fusion_){


    this->previous_path_x = previous_path_x_;
    this->previous_path_y = previous_path_y_;
    this->end_path_d =end_path_d_;
    this->end_path_s =end_path_s_;
    this->prev_path_size = this->previous_path_x.size();
    
    // this->ego_car = ego_car_;
    this->ego_car.update(s_,v_,d_,x_,y_,yaw_,this->prev_path_size,sensor_fusion_);
    
    // this->prev_state = this->state;

    this->get_keep_lane_duration();
    this->get_lane_change_on_delai();

};


void Planner::init_path()
{
    this->ptsx = {};
    this->ptsy = {};

          // reference x,y, yaw states
    this->ref_x = this->ego_car.x;
    this->ref_y = this->ego_car.y;
    this->ref_yaw = deg2rad(this->ego_car.yaw);   

    if(this->prev_path_size < 2)
    {
        double prev_car_x = this->ego_car.x - cos(this->ref_yaw);
        double prev_car_y = this->ego_car.y - sin(this->ref_yaw);

        this->ptsx.push_back(prev_car_x);
        this->ptsx.push_back(this->ego_car.x);
        this->ptsy.push_back(prev_car_y);
        this->ptsy.push_back(this->ego_car.y);
    }
    
    else
    {
        this->ref_x = this->previous_path_x[this->prev_path_size-1];
        this->ref_y = this->previous_path_y[this->prev_path_size-1];

        double ref_x_prev = this->previous_path_x[this->prev_path_size-2];
        double ref_y_prev = this->previous_path_y[this->prev_path_size-2];

        this->ref_yaw = atan2(this->ref_y-ref_y_prev, this->ref_x-ref_x_prev);

        this->ptsx.push_back(ref_x_prev);
        this->ptsx.push_back(this->ref_x);
        this->ptsy.push_back(ref_y_prev);
        this->ptsy.push_back(this->ref_y);
    }
        // std::cout<<"prev path size:"<<ptsx.size()<<std::endl;

};


vector<vector<double>> Planner::create_spoints(int lane, double dist_add)
{
    vector<double> ptsx_ = this->ptsx;
    vector<double> ptsy_ = this->ptsy;
    

    // std::cout<<"ptsx points:"<<ptsx_.size()<<std::endl;
    // In Frenet coordinate add even spaced points aheat of the starting reference
    // double dist_add=20;
    vector<double> next_wp0 = getXY(this->ego_car.s+30 + dist_add, (2+4*lane), maps_s, maps_x, maps_y);
    vector<double> next_wp1 = getXY(this->ego_car.s+30*2 + dist_add, (2+4*lane), maps_s, maps_x, maps_y);
    vector<double> next_wp2 = getXY(this->ego_car.s+30*2.5 + dist_add, (2+4*lane), maps_s, maps_x, maps_y);

    ptsx_.push_back(next_wp0[0]);
    ptsx_.push_back(next_wp1[0]);
    ptsx_.push_back(next_wp2[0]);

    ptsy_.push_back(next_wp0[1]);
    ptsy_.push_back(next_wp1[1]);
    ptsy_.push_back(next_wp2[1]);

    // Create a smooth path with spline function in local frame of ego vehicle
    // shift from global frame to local frame
    for (auto i=0; i<ptsx_.size(); i++)
    {
      // shift from global to local so that ego car position is at originate
      double local_x = ptsx_[i]-this->ref_x;
      double local_y = ptsy_[i]-this->ref_y;
      // rotation by -ref_raw to get coordinates in local frame
      ptsx_[i] = local_x * cos(-this->ref_yaw) - local_y * sin(-this->ref_yaw);
      ptsy_[i] = local_x * sin(-this->ref_yaw) + local_y * cos(-this->ref_yaw);

    }

    return{ptsx_, ptsy_};
};


vector<vector<double>> Planner::create_trajectory(int lane, double ref_vel,double dist_add)
{
    auto dist_add_ref = std::max(ref_vel,7.0)/49.5 *20 + dist_add;
    vector<vector<double>> pts = create_spoints(lane,dist_add_ref);
    vector<double> ptsx_all = pts[0];
    vector<double> ptsy_all = pts[1];  

    tk::spline s;
    s.set_points(ptsx_all, ptsy_all);


    vector<double> next_x_vals;
    vector<double> next_y_vals;
    for (auto i=0; i<this->prev_path_size; i++)
    {
        next_x_vals.push_back(this->previous_path_x[i]);
        next_y_vals.push_back(this->previous_path_y[i]);

    }
    
    // Generate remaining part of trajectory with target position and velocity
    // caculate how to break up spline points at desired reference velocity

    double target_x =30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
    double x_start =0.0;
    for (auto i=0; i<50-this->prev_path_size; i++)
    {
        double num_spline = target_dist/(ref_vel/2.24*0.02);
        double x_point = x_start + target_x/num_spline;
        double y_point = s(x_point);
        
        x_start = x_point;

        // shift back to global frame
        // rotation by ref_yaw
        double global_x = x_point*cos(this->ref_yaw)-y_point*sin(this->ref_yaw);
        double global_y = x_point*sin(this->ref_yaw)+y_point*cos(this->ref_yaw);
        // add ego_coordinates to be back in global frame
        global_x += this->ref_x;
        global_y += this->ref_y;

        next_x_vals.push_back(global_x);
        next_y_vals.push_back(global_y);
    }
    return {next_x_vals, next_y_vals};

};

vector<Planner::FSM> Planner::successor_states()
{
    vector<Planner::FSM> states;
    int min_lane_change_on_duration =150;
    int min_keep_lane_on_duration =100;

    auto vehicle_ahead= this->ego_car.get_vehicle_ahead(this->ego_car.lane);
    
    // if (vehicle_ahead.gap>50)
    // {
    //     states.push_back(Planner::FSM::KL);
    // }    
    // else 
    if(this->state == Planner::FSM::KL) {
        states.push_back(Planner::FSM::KL);
        if (this->keep_lane_duration>min_keep_lane_on_duration)
        {        
            if (this->ego_car.lane == 0)
            {
                states.push_back(Planner::FSM::PLCR);
                
            }
            else if(this->ego_car.lane== 2)
            {
                states.push_back(Planner::FSM::PLCL);
            }
            else
            {
                states.push_back(Planner::FSM::PLCL);
                states.push_back(Planner::FSM::PLCR);
            }
        }
    } 
    else if (this->state == Planner::FSM::PLCL) {
        
        states.push_back(Planner::FSM::KL);
        if (this->ego_car.lane != 0)
        {
            states.push_back(Planner::FSM::PLCL);
            states.push_back(Planner::FSM::LCL);
        }


        // }
        // else
        // {
        // states.push_back("PLCL");
        // states.push_back("PLCR");
        // states.push_back("KL");
        // }
        
    }
    else if (this->state == Planner::FSM::PLCR) {
        states.push_back(Planner::FSM::KL);

        if (this->ego_car.lane != 2) {
        states.push_back(Planner::FSM::PLCR);
        states.push_back(Planner::FSM::LCR);
        }

        // }
        // else
        // {
        // // states.push_back("PLCL");
        // // states.push_back("PLCR");
        // states.push_back("KL");
        // }
    }
    else if (this->state == Planner::FSM::LCR){
        if (this->lane_change_on_duration>min_lane_change_on_duration)
        {
            states.push_back(Planner::FSM::KL);
        }
        states.push_back(Planner::FSM::LCR);
            // if (this->ego_car.lane != 2 ){states.push_back(Planner::FSM::LCR);}  
        // else
        // {
        //     states.push_back(Planner::FSM::LCR);
        // }
        
        
    

         
    }
    else if (this->state == Planner::FSM::LCL){
        if (this->lane_change_on_duration>min_lane_change_on_duration)
        {
            states.push_back(Planner::FSM::KL);
        }
    
        states.push_back(Planner::FSM::LCL);

        // else
        // {
        //     states.push_back(Planner::FSM::LCL);
        // }
        
            
        
    }
    return states;

};


// vector<vector<double>> Planner::generate_next_trajectory()
// {
//     this->init_path();

// };
void Planner::realize_next_state(FSM new_state)
{
    this->prev_state = this->state;
    this->state = new_state;
};

Planner::path Planner::keep_lane_path(){
    auto ref_vel = this->ego_car.get_kinematics(this->ego_car.lane);
    // std::cout<<"speed target:"<<ref_vel<<std::endl;
    double dist_add =3;

    // this->ego_car.v = ref_vel;
    auto trajectory = create_trajectory(this->ego_car.lane, ref_vel, dist_add);
    Planner::path path_KL{trajectory, ref_vel, ref_vel, this->ego_car.lane};
    // return create_trajectory(this->ego_car.lane, ref_vel);
    return path_KL;

};


Planner::path Planner::prep_lane_change_path(Planner::FSM new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);
    new_lane = new_lane > 2 ? 2 : new_lane;
    new_lane = new_lane < 0 ? 0 : new_lane;
    auto actual_lane_speed = ego_car.get_kinematics(ego_car.lane);
    auto new_lane_speed = ego_car.get_kinematics(new_lane);
    double dist_add =3;
    // std::cout<<new_lane<<td::endl;

    // this->ego_car.v = actual_lane_speed;

    // is there is no gap in the new lane keep actual lane speed
    auto trajectory = create_trajectory(this->ego_car.lane, new_lane_speed, dist_add);
    Planner::path path_PLC{trajectory, actual_lane_speed,new_lane_speed, new_lane};
    return path_PLC;

};

Planner::path Planner::lane_change_path(Planner::FSM new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);
    vector<vector<double>> trajectory;
    new_lane = new_lane > 2 ? 2 : new_lane;
    new_lane = new_lane < 0 ? 0 : new_lane;
    double dist_add =10;
    
    // if(this->prev_target_lane != new_lane && this->prev_state == new_state)
    // {
    //     new_lane = this->prev_target_lane;
    //     this->prev_target_lane = new_lane;
    // }
    if(this->lane_change_on_duration<150)
    {
        new_lane = this->prev_target_lane;
    }

    auto new_lane_speed = ego_car.get_kinematics(new_lane);


    trajectory = create_trajectory(new_lane, new_lane_speed,dist_add);

    
    Planner::path path_LC{trajectory, new_lane_speed, new_lane_speed, new_lane};
    return path_LC;

};

bool Planner::lane_change_safe(Planner::FSM new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);

    bool lane_change_gap_enough = true;
    const double gap_ahead_lane_change = 15;
    const double gap_behind_lane_change = 15;
    auto vehicle_ahead = this->ego_car.get_vehicle_ahead(new_lane);
    auto vehicle_behind = this->ego_car.get_vehicle_ahead(new_lane);

    // if (vehicle_ahead.found && vehicle_ahead.gap > gap_ahead_lane_change)

    if (vehicle_behind.gap < gap_behind_lane_change)
    {
        lane_change_gap_enough = false;
    }
    else if (vehicle_ahead.gap < gap_ahead_lane_change)
    {
        lane_change_gap_enough = false;
    };

    // std::cout<<"vehicle close around:"<<ego_car.vehicle_close_around(new_lane)<<std::endl;
    // std::cout<<"lane change gap is enough"<<lane_change_gap_enough<<std::endl;

    return lane_change_gap_enough && ego_car.vehicle_close_around(new_lane);

};



double Planner::cost_function(double real_speed, double target_speed)
{

    const double REACH_GOAL = 0.4;
    const double EFFICIENCY = 1;
    const double SPEED_LIMIT = 49.5; //mph

    double cost = (2*SPEED_LIMIT - real_speed -target_speed)/SPEED_LIMIT*EFFICIENCY;

    return cost;

};

Planner::path Planner::generate_candidate_path(Planner::FSM new_state)
{
    Planner::path candidate_path;
    // vector<vector<double>> empty_trajectory;

    if (new_state == Planner::FSM::KL) {
        candidate_path = this->keep_lane_path();} 
    else if (new_state == Planner::FSM::LCL || new_state == Planner::FSM::LCR) {

        candidate_path = this->lane_change_path(new_state);

    }
    else if (new_state == Planner::FSM::PLCL || new_state == Planner::FSM::PLCR) {
        if (lane_change_safe(new_state)) //&& this->keep_lane_duration > 150
        {
            candidate_path = this->prep_lane_change_path(new_state);
        }
        // else
        // {
        //     // std::cout<<"lane_change_not_safe, size:"<<candidate_path.trajectory.size()<<std::endl;
        // }      
    }
    return candidate_path;
};



Planner::path Planner::find_next_path()
{
    this->init_path();

    vector<Planner::FSM> states = this->successor_states();

    // std::cout<< "state_size:"<<states.size()<< std::endl;
    // vector<Planner::path> paths;
    double cost;
    
    double cost_min = std::numeric_limits<double>::max();
    Planner::path best_path;
    Planner::FSM best_state;
    for (auto i=0; i<states.size(); i++)
    {
        Planner::path candidate_path;
        // path = generate_candidate_path(states[i]);
        std::cout<<"state:"<<states[i]<<std::endl;
        candidate_path = this->generate_candidate_path(states[i]);
        auto cost_traffic = this->traffic_cost(states[i]);
        // std::cout<<"cost_traffic"<<cost_traffic<<std::endl;
        auto cost_efficiency = this->cost_function(candidate_path.real_speed, candidate_path.target_speed);

        cost = cost_traffic*5 + cost_efficiency;
        // cost = this->cost_function(candidate_path.real_speed, candidate_path.target_speed);
        // std::cout<<"efficiency_cost"<<cost<<std::endl;
        if (candidate_path.trajectory.size()!=0)
        {
            if(cost<cost_min)
            {
                cost_min =cost;
                best_path = candidate_path;
                best_state = states[i];
            }

        }
    }
    // std::cout<<"previous_state: "<<this->state<<std::endl;
    // this->prev_state = this->state;
    // this->state = best_state;

    this->realize_next_state(best_state);
    // this->get_keep_lane_duration();
    this->prev_target_lane = this->target_lane;
    this->target_lane =best_path.target_lane;
    this->ego_car.v = best_path.real_speed;

    std::cout<<"next_best_state: "<<best_state<<std::endl;
    return best_path;

};


void Planner::get_keep_lane_duration()
{   
    // if (this->prev_target_lane !=this->target_lane)
    // {
    //     this->keep_lane_duration =0;
    // }
    // else
    // {
    //     this->keep_lane_duration+=1;
    // }

    if (this->prev_state== Planner::FSM::LCL || this->prev_state== Planner::FSM::LCR)
    {
        this->keep_lane_duration =0;
    }
    else 
    {
        this->keep_lane_duration+=1;
    }

    
    // std::cout<<"previous target lane:"<<this->prev_target_lane<<std::endl;
    // std::cout<<"actual target lane:"<<this->target_lane<<std::endl;
    // std::cout<<"in lane duration:"<<keep_lane_duration<<std::endl; 
    

};


double Planner::traffic_cost(Planner::FSM new_state)
{
    // double vehicle_ahead_dist = 200;
    // double vehicle_behind_dist = 200;
    Vehicle::prediction vehicle_ahead;
    Vehicle::prediction vehicle_behind;
    if (new_state == Planner::FSM::KL)
    {
        vehicle_ahead = ego_car.get_vehicle_ahead(this->ego_car.lane);
        vehicle_behind = ego_car.get_vehicle_behind(this->ego_car.lane);
        return 1/(vehicle_ahead.gap+1);
    }
    else 
    {
        int new_lane = this->ego_car.lane + lane_direction.at(new_state);
        new_lane = new_lane > 2 ? 2 : new_lane;
        new_lane = new_lane < 0 ? 0 : new_lane;
        vehicle_ahead = ego_car.get_vehicle_ahead(new_lane);
        vehicle_behind = ego_car.get_vehicle_behind(new_lane);
    // closer the vehicle ahead or behind in the new lane, higher the cost
    // double cost_traffic_ahead = 1/vehicle_ahead_dist
 
        // if (vehicle_ahead.found)
        // {
            
            auto vehicle_ahead_dist = vehicle_ahead.gap;
            if (vehicle_ahead_dist<20)
            {
                vehicle_ahead_dist = 0.1*vehicle_ahead_dist;
            }
        // }
        // if (vehicle_behind.found)
        // {
            auto vehicle_behind_dist = vehicle_behind.gap;
            if (vehicle_behind_dist<10)
            {
                vehicle_behind_dist = 0.1*vehicle_behind_dist;
            }

        // }
        return 1/(vehicle_ahead_dist+1) + 0.5*1/(vehicle_behind_dist+1);       


    }


    // when KL,ou LC, traffic cost will be ignored
    // else
    // {
    //     return 0.0;
    // }

}


void Planner::get_lane_change_on_delai()
{
    if(this->state == Planner::FSM::LCL && this->prev_state == Planner::FSM::LCL)
    {
        this->lane_change_on_duration +=1;
    }

    else  if(this->state == Planner::FSM::LCR && this->prev_state == Planner::FSM::LCR)
    {
        this->lane_change_on_duration +=1;
    }

    else{
        this->lane_change_on_duration =0;
    }
    std::cout<<"lane change on duration: "<<lane_change_on_duration<<std::endl;

}