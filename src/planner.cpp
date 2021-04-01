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
const double EFFICIENCY_WEIGHT = 1;
const double TRAFFIC_WEIGHT = 1;
const double MANEUVER_WEIGHT = 0.2;
const double REACH_GOAL_WEIGHT = 0.1;
const double MIN_LANE_CHANGE_ON_DURATION = 4/0.02; // for 3s
const double MIN_KEEP_LANE_ON_DURATION = 4/0.02; // for 3s
// const double REACH_GOAL_WEIGHT = 10;

const double SPEED_LIMIT = 49.5; //mph
const double DISTANCE_IGNORE_VEHICLE_AHEAD = 100; //m


const double SECURITE_GAP_AHEAD_LANE_CHANGE = 8;
const double SECURITE_GAP_BEHIND_LANE_CHANGE = 15;

const double BUFFER_DISTANCE_AHEAD = 35; //m


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
    // this->state = Planner::FSM::KL;
    this->state = Planner::FSM::CS;
    this->ptsx = {};
    this->ptsy = {};

};

Planner::~Planner(){};

void Planner::update(const vector<double>& previous_path_x_, const vector<double>& previous_path_y_, const double& end_path_s_, const double& end_path_d_, const double& s_, const double& v_, const double& d_, const double& x_, const double& y_, const double& yaw_, const nlohmann::json& sensor_fusion_){


    this->previous_path_x = previous_path_x_;
    this->previous_path_y = previous_path_y_;
    this->end_path_d =end_path_d_;
    this->end_path_s =end_path_s_;
    this->prev_path_size = this->previous_path_x.size();    
    this->ego_car.update(s_,v_,d_,x_,y_,yaw_,this->prev_path_size,sensor_fusion_);
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


vector<vector<double>> Planner::create_spoints(const int& lane, const double& dist_add)
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
      // shift from global to local so that ego car position is at origin
      double local_x = ptsx_[i]-this->ref_x;
      double local_y = ptsy_[i]-this->ref_y;
      // rotation by -ref_raw to get coordinates in local frame
      ptsx_[i] = local_x * cos(-this->ref_yaw) - local_y * sin(-this->ref_yaw);
      ptsy_[i] = local_x * sin(-this->ref_yaw) + local_y * cos(-this->ref_yaw);

    }

    return{ptsx_, ptsy_};
};


vector<vector<double>> Planner::create_trajectory(const int& lane, const double& ref_vel, const double& dist_add)
{
    // auto dist_add_ref = std::max(ref_vel,5.0)/49.5 *20 + dist_add;

    vector<vector<double>> pts = create_spoints(lane,dist_add);
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
    // int min_lane_change_on_duration =150;
    // int min_keep_lane_on_duration =100;

    auto vehicle_ahead= this->ego_car.get_vehicle_ahead(this->ego_car.lane);
    
    // if (vehicle_ahead.gap>50)
    // {
    //     states.push_back(Planner::FSM::KL);
    // }    
    // else 

    if(this->state == Planner::FSM::CS)
    {
        states.push_back(Planner::FSM::KL);
    }
    else if(this->state == Planner::FSM::KL) {
        states.push_back(Planner::FSM::KL);
        if (this->keep_lane_duration>MIN_KEEP_LANE_ON_DURATION && vehicle_ahead.gap < BUFFER_DISTANCE_AHEAD)
        {        
            if (this->ego_car.lane == 0)
            {
                
                if (this->lane_change_safe(Planner::FSM::PLCR)) states.push_back(Planner::FSM::PLCR);
                
            }
            else if(this->ego_car.lane== 2)
            {
                if (this->lane_change_safe(Planner::FSM::PLCL)) states.push_back(Planner::FSM::PLCL);
            }
            else
            {
                // states.push_back(Planner::FSM::PLCL);
                // states.push_back(Planner::FSM::PLCR);
                if (this->lane_change_safe(Planner::FSM::PLCL)) states.push_back(Planner::FSM::PLCL);
                if (this->lane_change_safe(Planner::FSM::PLCR)) states.push_back(Planner::FSM::PLCR);

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
        if (this->lane_change_on_duration>MIN_LANE_CHANGE_ON_DURATION)
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
        if (this->lane_change_on_duration>MIN_LANE_CHANGE_ON_DURATION)
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
    // std::cout<<"new state:"<<state<<std::endl;
    this->state = new_state;
};

Planner::path Planner::keep_lane_path(){

    double ref_speed, target_speed;
    auto vehicle_ahead = ego_car.get_vehicle_ahead(this->ego_car.lane);
    if (this->state == Planner::FSM::CS) {
        ref_speed = .224;}
    else {
        ref_speed = this->ego_car.get_kinematics(this->ego_car.lane);}

    // If vehicle ahead are far away from ego car, speed limit will be considered as target speed 
    if (vehicle_ahead.gap > DISTANCE_IGNORE_VEHICLE_AHEAD) target_speed = SPEED_LIMIT;
    else target_speed = ref_speed;
    // std::cout<<"speed target:"<<ref_vel<<std::endl;
    
    // double dist_add = 5;
    double dist_add = std::max(ref_speed,5.0)/49.5 *5 + 3;
    // this->ego_car.v = ref_vel;
    auto trajectory = create_trajectory(this->ego_car.lane, ref_speed, dist_add);
    Planner::path path_KL{trajectory, ref_speed, target_speed, this->ego_car.lane};
    // return create_trajectory(this->ego_car.lane, ref_vel);
    return path_KL;

};


Planner::path Planner::prep_lane_change_path(const Planner::FSM& new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);
    new_lane = new_lane > 2 ? 2 : new_lane;
    new_lane = new_lane < 0 ? 0 : new_lane;
    
    auto actual_lane_speed = ego_car.get_kinematics(ego_car.lane);

    auto vehicle_ahead = ego_car.get_vehicle_ahead(new_lane);
    double new_lane_speed= ego_car.get_kinematics(new_lane);
    double new_lane_target_speed = new_lane_speed;
    if (vehicle_ahead.gap > DISTANCE_IGNORE_VEHICLE_AHEAD) new_lane_target_speed = SPEED_LIMIT;
    // auto new_lane_speed = ego_car.get_kinematics(new_lane);

    double dist_add = 3 + std::max(new_lane_speed,5.0)/49.5 *5;
    // double dist_add = 3 + std::max(new_lane_speed,5.0)/49.5 *20;

    // is there is no gap in the new lane keep actual lane speed
    auto trajectory = create_trajectory(this->ego_car.lane, new_lane_speed, dist_add);
    Planner::path path_PLC{trajectory, actual_lane_speed,new_lane_target_speed, new_lane};
    return path_PLC;

};

Planner::path Planner::lane_change_path(const Planner::FSM& new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);
    vector<vector<double>> trajectory;
    new_lane = new_lane > 2 ? 2 : new_lane;
    new_lane = new_lane < 0 ? 0 : new_lane;
    

    //Lane change should target to the same lane for at least a certain duration before making next lane change
    if(this->lane_change_on_duration < MIN_LANE_CHANGE_ON_DURATION)
    {
        new_lane = this->prev_target_lane;
    }

    auto new_lane_speed = ego_car.get_kinematics(new_lane);
    auto vehicle_ahead = ego_car.get_vehicle_ahead(new_lane);   
    double new_lane_target_speed= new_lane_speed;
    if (vehicle_ahead.gap > DISTANCE_IGNORE_VEHICLE_AHEAD) new_lane_target_speed = SPEED_LIMIT;


    // double dist_add = 10;
    double dist_add = 8 + std::max(new_lane_speed,5.0)/49.5 *15 ;

    trajectory = create_trajectory(new_lane, new_lane_speed,dist_add);

    
    Planner::path path_LC{trajectory, new_lane_speed, new_lane_target_speed, new_lane};
    return path_LC;

};

bool Planner::lane_change_safe(const Planner::FSM& new_state)
{
    int new_lane = this->ego_car.lane + lane_direction.at(new_state);
    new_lane = new_lane > 2 ? 2 : new_lane;
    new_lane = new_lane < 0 ? 0 : new_lane;

    bool lane_change_gap_enough = true;

    auto vehicle_ahead = this->ego_car.get_vehicle_ahead(new_lane);
    auto vehicle_behind = this->ego_car.get_vehicle_behind(new_lane);

    // if (vehicle_ahead.found && vehicle_ahead.gap > gap_ahead_lane_change)
    
    // Check if gap enough to change lane
    if (vehicle_behind.gap < SECURITE_GAP_BEHIND_LANE_CHANGE)
    {
        lane_change_gap_enough = false;
    }
    if (vehicle_ahead.gap < SECURITE_GAP_AHEAD_LANE_CHANGE)
    {
        lane_change_gap_enough = false;
    }

    // get speed setpoint to be applied if changing lane
    double new_lane_speed= ego_car.get_kinematics(new_lane);
    // predict ego car's position if changing lane
    double ego_car_s_pred = this->ego_car.s + double(this->prev_path_size) *.02*new_lane_speed/2.24;
    double min_gap_ahead = 5;
    double min_gap_behind = 12;
    bool collision_behind = false;

    // check if vehicle will risk to collide with vehicle behind in the new lane
    if (vehicle_behind.found)
    {
            // predict vehicle postions by assuming constant speed 
            double vehicle_behind_s_pred = double(this->prev_path_size) *.02*vehicle_behind.vel/2.24 + vehicle_behind.s;
            
            if ((ego_car_s_pred - vehicle_behind_s_pred) <= min_gap_behind)
            {
                collision_behind = true;
            }
    }
    // check if vehicle will risk to collide with vehicle ahead in the new lane
    bool collision_ahead = false;
    if (vehicle_ahead.found)
    {       
            // predict vehicle postions by assuming constant speed
            double vehicle_ahead_s_pred = double(this->prev_path_size) *.02*vehicle_ahead.vel/2.24 + vehicle_ahead.s;
            if ((vehicle_ahead_s_pred - ego_car_s_pred) < min_gap_ahead)
            {
                collision_ahead = true;
            }
    }

    // check if vehicle ahead in current lane is too close for lane change
    auto vehicle_ahead_current_lane = this->ego_car.get_vehicle_ahead(this->ego_car.lane);
    bool vehicle_ahead_too_close = vehicle_ahead_current_lane.gap < 8;

    // return collision;
    // std::cout<<"vehicle close around:"<<ego_car.vehicle_close_around(new_lane)<<std::endl;
    // std::cout<<"lane change gap is enough"<<lane_change_gap_enough<<std::endl;

    // return lane_change_gap_enough && (! ego_car.vehicle_close_around(new_lane)) && ego_car.check_collision(new_lane);
    // return lane_change_gap_enough && (! vehicle_ahead_too_close) && (!collision_behind) && (!collision_ahead);
    return  (! vehicle_ahead_too_close) && (!collision_behind) && (!collision_ahead);

};



double Planner::efficiency_cost(const double& real_speed, const double& target_speed)
{
    double cost = (2*SPEED_LIMIT - real_speed -target_speed)/SPEED_LIMIT;
    return cost;
};

Planner::path Planner::generate_candidate_path(const Planner::FSM& new_state)
{
    Planner::path candidate_path;
    // vector<vector<double>> empty_trajectory;

    if (new_state == Planner::FSM::KL) {
        candidate_path = this->keep_lane_path();} 
    else if (new_state == Planner::FSM::LCL || new_state == Planner::FSM::LCR) {

        // if (lane_change_emergency())
        // {
        //     candidate_path = this->keep_lane_path();

        // }
        // else {
            
            candidate_path = this->lane_change_path(new_state);
        // }

    }
    else if (new_state == Planner::FSM::PLCL || new_state == Planner::FSM::PLCR) {
        // check if lane change is safe to allow lane change state
        // if (lane_change_safe(new_state)) 
        candidate_path = this->prep_lane_change_path(new_state);
        // else candidate_path = this->keep_lane_path();
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
        // std::cout<<"state:"<<states[i]<<std::endl;
        candidate_path = this->generate_candidate_path(states[i]);

        // cost = this->cost_function(candidate_path.real_speed, candidate_path.target_speed);
        // std::cout<<"efficiency_cost"<<cost<<std::endl;
        if (candidate_path.trajectory.size()!=0)
        {

            auto cost_traffic = this->traffic_cost(states[i]);
            // std::cout<<"cost_traffic"<<cost_traffic<<std::endl;
            auto cost_efficiency = this->efficiency_cost(candidate_path.real_speed, candidate_path.target_speed);
            auto cost_maneuver = this->maneuver_cost(states[i]);
            auto cost_reach_goal = this->reach_goal_cost(states[i]);
            cost = cost_traffic * TRAFFIC_WEIGHT + cost_efficiency * EFFICIENCY_WEIGHT + cost_maneuver * MANEUVER_WEIGHT + cost_reach_goal*REACH_GOAL_WEIGHT;
            // cost = cost_traffic * TRAFFIC_WEIGHT + cost_efficiency * EFFICIENCY_WEIGHT + cost_maneuver * MANEUVER_WEIGHT;
   
            // std::cout<<"traffic cost: "<<cost_traffic * TRAFFIC_WEIGHT<<std::endl;
            // std::cout<<"efficiency cost: "<<cost_efficiency * EFFICIENCY_WEIGHT<<std::endl;
            // std::cout<<"maneuver cost: "<<cost_maneuver * MANEUVER_WEIGHT<<std::endl;
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
    this->prev_target_lane = this->target_lane;
    this->target_lane =best_path.target_lane;
    this->ego_car.v = best_path.real_speed;

    // std::cout<<"next_best_state: "<<best_state<<std::endl;
    // auto vehicle_ahead = ego_car.get_vehicle_ahead_debug(this->ego_car.lane);
    // std::cout<<"vehicle ahead d: "<<vehicle_ahead.d<<std::endl;
    return best_path;

};


void Planner::get_keep_lane_duration()
{   

    if (this->prev_state== Planner::FSM::LCL || this->prev_state== Planner::FSM::LCR)
    {
        this->keep_lane_duration =0;
    }
    else 
    {
        this->keep_lane_duration+=1;
    }


    // if (this->prev_state != Planner::FSM::KL)
    // {
    //     this->keep_lane_duration =0;
    // }
    // else if (this->prev_state == Planner::FSM::KL && this->state == Planner::FSM::KL)
    // {
    //     this->keep_lane_duration+=1;
    // }

    
    // std::cout<<"previous target lane:"<<this->prev_target_lane<<std::endl;
    // std::cout<<"actual target lane:"<<this->target_lane<<std::endl;
    // std::cout<<"in lane duration:"<<keep_lane_duration<<std::endl; 
    

};


double Planner::traffic_cost(const Planner::FSM& new_state)
{
    Vehicle::prediction vehicle_ahead;
    Vehicle::prediction vehicle_behind;
    if (new_state == Planner::FSM::KL)
    {
        vehicle_ahead = ego_car.get_vehicle_ahead(this->ego_car.lane);
        vehicle_behind = ego_car.get_vehicle_behind(this->ego_car.lane);
    }
    else 
    {
        int new_lane = this->ego_car.lane + lane_direction.at(new_state);
        new_lane = new_lane > 2 ? 2 : new_lane;
        new_lane = new_lane < 0 ? 0 : new_lane;
        vehicle_ahead = ego_car.get_vehicle_ahead(new_lane);
        vehicle_behind = ego_car.get_vehicle_behind(new_lane);
    }

    // closer the vehicle ahead or behind in the new lane, higher the cost
    auto vehicle_ahead_dist = vehicle_ahead.gap;
    if (vehicle_ahead_dist > DISTANCE_IGNORE_VEHICLE_AHEAD)
    {
        // vehicle_ahead_dist = std::numeric_limits<double>::max();
        vehicle_ahead_dist = DISTANCE_IGNORE_VEHICLE_AHEAD;
    }

    // else if (vehicle_ahead_dist<10)
    // {
    //     vehicle_ahead_dist = 0.1*vehicle_ahead_dist;
    // }

    // auto vehicle_behind_dist = vehicle_behind.gap;
    // if (vehicle_behind_dist > 15)
    // {
        // vehicle_behind_dist = std::numeric_limits<double>::max();
        // return 1/(vehicle_ahead_dist+1);
    if (vehicle_ahead_dist > BUFFER_DISTANCE_AHEAD)    return (DISTANCE_IGNORE_VEHICLE_AHEAD - vehicle_ahead_dist - BUFFER_DISTANCE_AHEAD)/DISTANCE_IGNORE_VEHICLE_AHEAD;
    else return (DISTANCE_IGNORE_VEHICLE_AHEAD - vehicle_ahead_dist)/DISTANCE_IGNORE_VEHICLE_AHEAD;
    // }

    // else
    // {
    //     // vehicle_behind_dist = 0.1*vehicle_behind_dist;
    //     // return (1/(vehicle_ahead_dist+1) + 0.2*1/(vehicle_behind_dist+1)); 
    //     // return (DISTANCE_IGNORE_VEHICLE_AHEAD-vehicle_ahead_dist)/DISTANCE_IGNORE_VEHICLE_AHEAD + (15-vehicle_behind_dist)/15 * 0.1;
    //     return (DISTANCE_IGNORE_VEHICLE_AHEAD-vehicle_ahead_dist)/DISTANCE_IGNORE_VEHICLE_AHEAD;

    // }

        // return (1/(vehicle_ahead_dist+1) + 0.2*1/(vehicle_behind_dist+1));   
               

    

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
    // std::cout<<"lane change on duration: "<<lane_change_on_duration<<std::endl;

}


double Planner::maneuver_cost(const Planner::FSM& new_state)
{
    double cost_maneuver;
    if (new_state == Planner::FSM::KL)
    {
        cost_maneuver = 0.0;
    }
    else if (new_state == Planner::FSM::PLCL || new_state == Planner::FSM::LCL)
    {
        cost_maneuver = 0.5;
    }
    else {
        cost_maneuver = 1;
    }

    return cost_maneuver;

}


bool Planner::lane_change_emergency()
{
    // If during a lane change a vehicle suddenlly enter into the new lane, a emergency alert should raise
    auto vehicle_ahead = this->ego_car.get_vehicle_ahead(this->ego_car.lane);
    return vehicle_ahead.gap < 10;

};


double Planner::reach_goal_cost(const Planner::FSM& new_state)
{
    int new_lane;
    if (new_state == Planner::FSM::KL || new_state == Planner::FSM::CS) new_lane = this->ego_car.lane;
    else new_lane = this->ego_car.lane + lane_direction.at(new_state);
    double dist_ahead = 0.0;
    auto best_lane = this->ego_car.lane;
    for (auto lane=0; lane<3; ++lane)
    {
        auto vehicle_ahead = ego_car.get_vehicle_ahead(lane);
        if (vehicle_ahead.gap > dist_ahead)
        {
            best_lane = lane;
            dist_ahead = vehicle_ahead.gap;
        }
    }

    auto vehicle_behind_best_lane = ego_car.get_vehicle_behind(best_lane);
    if (dist_ahead < 50 || vehicle_behind_best_lane.gap < 30) return 0.0;
    else return double(fabs(new_lane - best_lane)/3.0);
}