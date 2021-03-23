#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "planner.h"

// #include <typeinfo>
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  static double ref_vel = 0.0;
  static string state ="KL";

  // static Vehicle car=Vehicle();
  static Planner planner = Planner(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // std::cout<<"car speed:"<<car_speed<<std::endl;
          int prev_size = previous_path_x.size();

          // ref_vel = planner.ego_car.v;
          // car.update(car_s,ref_vel,car_d,car_x,car_y,car_yaw, prev_size, sensor_fusion);
          planner.update(previous_path_x,previous_path_y, end_path_s, end_path_d, car_s,car_speed,car_d,car_x,car_y,car_yaw, sensor_fusion);
          // planner.ego_car.update()
          // std::cout<<car.v<< std::endl;


        //  int car_lane = floor(car_d / 4);
        //   double prev_start_x;
        //   double prev_start_y;

        //   if (prev_size>0)
        //   {
        //     car_s = end_path_s;
        //     prev_start_x = previous_path_x[0];
        //     prev_start_y = previous_path_y[0];
        //   }
        //   double prev_start_theta = 0;
        //   if (prev_size>2)
        //   {
        //       double prev_2nd_x = previous_path_x[1];
        //       double prev_2nd_y = previous_path_y[1];
        //       prev_start_theta = atan2(prev_2nd_y-prev_start_y, prev_2nd_y-prev_start_y);
  
        //   }
        //   auto prev_start_point = getFrenet(prev_start_x, prev_start_y, prev_start_theta, map_waypoints_x, map_waypoints_y);
        //   auto prev_start_d = prev_start_point[1];

        //   int lane = car_lane;
        //   vector<double> ptsx;
        //   vector<double> ptsy;

        //   // reference x,y, yaw states

        //   double ref_x = car_x;
        //   double ref_y = car_y;
        //   double ref_yaw = deg2rad(car_yaw);   

        //   if(prev_size < 2)
        //   {
        //     double prev_car_x = car_x - cos(ref_yaw);
        //     double prev_car_y = car_y - sin(ref_yaw);

        //     ptsx.push_back(prev_car_x);
        //     ptsx.push_back(car_x);
        //     ptsy.push_back(prev_car_y);
        //     ptsy.push_back(car_y);
        //   }
          
        //   else
        //   {
        //     ref_x = previous_path_x[prev_size-1];
        //     ref_y = previous_path_y[prev_size-1];

        //     double ref_x_prev = previous_path_x[prev_size-2];
        //     double ref_y_prev = previous_path_y[prev_size-2];

        //     ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        //     ptsx.push_back(ref_x_prev);
        //     ptsx.push_back(ref_x);
        //     ptsy.push_back(ref_y_prev);
        //     ptsy.push_back(ref_y);
        //   }


          

          // path planner- chose anker point according to state
          // possibe states:
          // KL - keep lane
          // LCL - Lane change left
          // LCR - Lane change right

          // vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // ptsx.push_back(next_wp0[0]);
          // ptsx.push_back(next_wp1[0]);
          // ptsx.push_back(next_wp2[0]);

          // ptsy.push_back(next_wp0[1]);
          // ptsy.push_back(next_wp1[1]);
          // ptsy.push_back(next_wp2[1]);

          // // Create a smooth path with spline function in local frame of ego vehicle
          // // shift from global frame to local frame
          // for (auto i=0; i<ptsx.size(); i++)
          // {
          //   // shift from global to local so that ego car position is at originate
          //   double local_x = ptsx[i]-ref_x;
          //   double local_y = ptsy[i]-ref_y;
          //   // rotation by -ref_raw to get coordinates in local frame
          //   ptsx[i] = local_x * cos(-ref_yaw) - local_y * sin(-ref_yaw);
          //   ptsy[i] = local_x * sin(-ref_yaw) + local_y * cos(-ref_yaw);

          // }
          // if (ref_vel<49.5)
          // {
          //   ref_vel += .224;
          // }
          // std::cout<<lane<<std::endl;
 
          // double target_vel = keep_lane_trajectory(sensor_fusion, state, lane, prev_size, ref_vel, car_s);
          // ref_vel = target_vel;
          
          // // Generate fisrt part of trajectory with previous path which has not been travelled
          // vector<vector<double>> pts = gen_spoints(ptsx, ptsy, lane, ref_x, ref_y, ref_yaw, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // auto next_path = keep_lane_trajectory(previous_path_x, previous_path_y, ptsx, ptsy, 
          // sensor_fusion, state, lane, prev_size, ref_vel, car_s, ref_x, ref_y, ref_yaw, 
          // map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // auto next_path = choose_next_state(state,previous_path_x, previous_path_y, ptsx, ptsy, 
          // sensor_fusion, lane, prev_size, ref_vel, car_s, ref_x, ref_y, ref_yaw, 
          // map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // auto next_x_vals = next_path[0];
          // auto next_y_vals = next_path[1];
          // ref_vel = (next_path[2][0]<0) ? 1/2.24: next_path[2][0];

          // auto cur_state = state;
          // auto result = choose_next_state(cur_state,previous_path_x, previous_path_y, ptsx, ptsy, 
          // sensor_fusion, lane, prev_size, ref_vel, car_s, ref_x, ref_y, ref_yaw, 
          // map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // auto next_path = result.next_path;
          // auto next_x_vals = next_path[0];
          // auto next_y_vals = next_path[1];
          // auto next_vel = result.next_vel;
          // ref_vel = (next_vel<0) ? 1: next_vel;

          // state = result.next_state;
          // std::cout<<"state:"<<state<<std::endl;
          
          // auto next_path =planner.keep_lane_trajectory();

          Planner::path next_path = planner.find_next_path();
          // auto test = planner.prep_lane_change_trajectory(Planner::FSM::PLCL);
          auto next_x_vals = next_path.trajectory[0];
          auto next_y_vals = next_path.trajectory[1];


          // ref_vel = planner.ego_car.v;
          // Transfer to simulator
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}