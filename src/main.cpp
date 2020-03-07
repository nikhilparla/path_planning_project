#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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

  int lane = 1; // middle lane
  double ref_vel = 0.0;  // miles per hr

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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

          json msgJson;


          // the points we are building with previous values too
          vector<double> x_pts;
          vector<double> y_pts;

          int prev_size = previous_path_x.size();
          bool reduce_speed = false;
          bool change_lane = false;
          bool left_lane_free = false;
          bool right_lane_free = false;

          /* first lets avoid collision with the car in the front */
          // if we have any points left over
          if(prev_size > 0)
            car_s = end_path_s;

          for(int i=0; i< sensor_fusion.size(); i++)
          {
            // check for the cars present in our lane
            if((sensor_fusion[i][6] > (2+4*lane)-2) && (sensor_fusion[i][6] < (2+4*lane)+2))
            {
              // if its s value is less than 30 ahead of us, reduce speed
              if(sensor_fusion[i][5] < (30 + car_s) && (sensor_fusion[i][5] > car_s)){
                reduce_speed = true;
              }
            }
          }

          // 22.4 miles per hr ~= 10 mts per sec square
          // 20ms frame is 1/50th of sec
          // 22.4/50 = 0.44 mts per sec square
          // 0.224 miles per hr ~= 5 mts per sec square (less than required max)
          if (reduce_speed)
            ref_vel -= 0.224;
          else if (ref_vel < 49.5)
            ref_vel += 0.224;

          int  ref_lane = lane;

          // If reduced speed, check cars in other lanes
          if (reduce_speed)
          {
            // check for the cars present left lane
            if ((lane - 1) > -1)
            {
              lane--;

              // check if you can change the lane
              for (int i = 0; i < sensor_fusion.size(); i++)
              {

                if ((sensor_fusion[i][6] > (2 + 4 * lane) - 2) && (sensor_fusion[i][6] < (2 + 4 * lane) + 2))
                {
                  if (sensor_fusion[i][5] < (30 + car_s) && (sensor_fusion[i][5] > car_s))
                  {
                    left_lane_free = false;
                    break;
                  }
                }
                // if i finishes without breaking
                    std::cout << "left lane i value  "<< i << std::endl;
                if(i == sensor_fusion.size())
                  {
                    left_lane_free = true;
                    std::cout << "left lane free "<< std::endl;
                  }
              } // end for
            } // end if

            lane = ref_lane;

            // check for the cars present right lane
            if ((lane + 1) < 3 && !left_lane_free)
            {
              lane++;

              // check if you can change the lane
              for (int i = 0; i < sensor_fusion.size(); i++)
              {

                if ((sensor_fusion[i][6] > (2 + 4 * lane) - 2) && (sensor_fusion[i][6] < (2 + 4 * lane) + 2))
                {
                  if (sensor_fusion[i][5] < (30 + car_s) && (sensor_fusion[i][5] > car_s))
                  {
                    right_lane_free = false;
                    break;
                  }
                }
                // if i finishes without breaking
                if (i == sensor_fusion.size())
                {
                  right_lane_free = true;
                  std::cout << "left lane free " << std::endl;
                }
              } // end for
            }   // end if
          }

          // prefer left lane if free
          if (left_lane_free)
          {
            lane = ref_lane - 1;
          }
          else if (right_lane_free)
          {
            lane = ref_lane + 1;
          }
          else
          {
            lane = ref_lane;
          }

          // define some reference states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the prev state is empty , use car as starting ref
          if(prev_size < 2)
          {
            // curr is car_x. Get the prev state
            double car_x_prev = car_x - cos(car_yaw);
            double car_y_prev = car_y - sin(car_yaw);

            x_pts.push_back(car_x_prev);
            y_pts.push_back(car_y_prev);

            x_pts.push_back(car_x);
            y_pts.push_back(car_y);
          }
          else{
            // you have more points from prev state
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            // get a previous point to this too to get the tangent angle
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            // calculate what angle the car was moving in
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev );

            x_pts.push_back(ref_x_prev);
            x_pts.push_back(ref_x);

            y_pts.push_back(ref_y_prev);
            y_pts.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s + 30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          x_pts.push_back(next_wp0[0]);
          x_pts.push_back(next_wp1[0]);
          x_pts.push_back(next_wp2[0]);

          y_pts.push_back(next_wp0[1]);
          y_pts.push_back(next_wp1[1]);
          y_pts.push_back(next_wp2[1]);

          // shift points to local co-ods for spline fit
          for(int i=0; i< x_pts.size(); i++)
          {
            // shift car ref angle to 0
            double shift_x = x_pts[i] - ref_x;
            double shift_y = y_pts[i] - ref_y;

            //rotate
            x_pts[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            y_pts[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline
          tk::spline s;

          s.set_points(x_pts, y_pts);

          // the points we want to actually pass to planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all the points in the previous path
          for(int i=0; i< previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Now you need to get points for the car to follow b/w these points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_addon = 0;

          // fill up rest of the points upto 50
          // check the README for description of code
          for(int i = 1; i < 50-previous_path_x.size(); i++)
          {
            double N = (target_dist/ (0.02 * ref_vel / 2.24)); // miles - mps
            double x_pt = x_addon + (target_x)/N;
            double y_pt = s(x_pt);

            x_addon = x_pt;

            double x_ref = x_pt;
            double y_ref = y_pt;

            // rotate and translation back to normal (from local to global)
            // rotation
            x_pt = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_pt = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            // translation
            x_pt += ref_x;
            y_pt += ref_y;

            next_x_vals.push_back(x_pt);
            next_y_vals.push_back(y_pt);
          }

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