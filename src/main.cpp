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
  
  //Start in lane 1
  int lane = 1;
  
  // Reference Velocity
  double ref_vel=0.0;//mph
  
  

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
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

          //size of the previous path
          int prev_size = previous_path_x.size();
          
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
                    
          /** 
          
          		//Make the car go in a straight line at constant speed like advised in the class
          
          
          double dist_inc = 0.5;
		 	for (int i = 0; i < 50; ++i) {
  			next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
  			next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
			}
            
            Yeah !! the Car goes in a straight line without issues
            
			
          
          // 		Make the car hold the lane at a constant speed
          
          double dist_inc = 0.5;
		 	for (int i = 0; i < 50; ++i) {
              double next_s = car_s+(i+1)*dist_inc;
              double next_d = 6;
              vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              
  			  next_x_vals.push_back(xy[0]);
  			  next_y_vals.push_back(xy[1]);
              
			}
            //    Yeah !! the Car keeps its lane, but rear ends other cars on the lane as expected
            
			*/
          
          		// Code part to recognise othe vehicles through sensor fusion
          //	Sensor fusion
          
          // 1- i: refers to the i th car in a list of other cars from 0 to i

          // 3- [3]: refers to the vx of the ith car
          // 4- [4]: refers to the vy of the ith car
          // 5- [5]: refers to the s value in freenet coordinate of the ith car
          // 6- [6]: refers to d of the ith car in the freenet coordinate
           
          
          // Start in lane 1; 
          //int lane = 1;
          
          // Reference Velocity
          //double ref_vel;//mph
          
         
          // Starting off
          if (prev_size > 0)
          {
            car_s= end_path_s;
          }
          
          int veh_pos; // 1- Behind, 2- beside, 3- in front
          int  rel_speed; // relative velocity of the vehicle with the ego vehicle  1- if ego car is fastet, 0  the speed is same , 2- if ego car is slower
          int veh_zone;// Vehicle is dtected in zone
          bool veh_intv = false; // An intervention is required 
          bool slowdown = false; // start to brake
          bool check_Lane_change_left = false; // check if a vehicle is on the right of the ego vehicle
          bool check_Lane_change_right = false; // check if a vehicle is on the left of the ego vehicle
          bool change_lane_left =false; // set change lane right
          bool change_lane_right = false; // set change lane left
          double weight_slowdown = 0; // weight for slowing down
          double weight_change_left = 0; // weight for changing lane to the right
          double weight_change_right = 0; // weight for changing lane to the left
         
          
          // find ref_v to use
          for(int i= 0; i < sensor_fusion.size(); i++)
          {
            // another car is in my lane
            float d = sensor_fusion[i][6]; // refers to d of the ith car in the freenet coordinate
            double vx = sensor_fusion[i][3]; //refers to the vx of the ith car
            double vy = sensor_fusion[i][4]; //refers to the vy of the ith car

            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5]; //refers to the s value in freenet coordinate of the ith car
            check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points can project s value outwards in time
			bool rel_speed;// relative speed of the ith car wrt our car, if false crash with the ith car is morelikely
            int rel_dist; // divides the relative distance into zone-0, -1,-2 and -3 
            double lane_zone = 1.5 ; // given width from right and left of the lane ceter will be monitored
            
            // vehicle position
            if (check_car_s > car_s) // the ith car is in front of our car
            {
              veh_pos = 3;
            }
            else if (check_car_s < car_s)// the ith car is behind our car
            {
              veh_pos = 1;
            }
            else // the ith car is right next to our car
            {
              veh_pos = 2;
            }
            
            // Relative Speed
            if ((car_speed-check_speed) >0)
            {
              rel_speed =1; 
            }
            else if ((car_speed-check_speed) ==0)
            {
              rel_speed = 0;
            }
            else if ((car_speed-check_speed) < 0)
            {
              rel_speed = 2;
            }
            
            // Vehicle Zone
            if (abs(check_car_s-car_s)>=40)
            { // the ith vehicle at a sufficiently safe distance from our vehicle
              veh_zone = 3;
            }
            else if (abs(check_car_s-car_s)<40 && abs(check_car_s-car_s)>30)
            {
              veh_zone =2;
            }
            else if (abs(check_car_s-car_s)<30 && abs(check_car_s-car_s)>20)
            {
              veh_zone =1;
            }
            else 
            {
              veh_zone = 0;
            }
            
            if(veh_zone<=2 && (d< (2+4*lane + lane_zone) && d> (2+4*lane - lane_zone)) && veh_pos ==3)
            {
              veh_intv = true;
            }
           
            
             if (veh_zone == 3) // all vehicles are at a safe distance
             {
              weight_slowdown = 0.2; // braking not priority 
              weight_change_left = 0.9; // free to turn left
              weight_change_right = 0.8; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
             }
             if (veh_zone == 2)               
             {
               if (veh_pos ==1) // car is comming from behind
               {
                 if(rel_speed ==1) // Ego car is driving away from the car from behind
                 {
                   weight_slowdown = 0.4; // braking not priority 
                   weight_change_left = 0.8; // free to turn left
                   weight_change_right = 0.7; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==0) // Ego car is driving at the same speed
                 {
                   weight_slowdown = 0.5; // braking is higher priority
                   weight_change_left = 0.7; // free to turn left
                   weight_change_right = 0.6; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==2) // Ego car is going slower, danger of being rear ended
                 {
                   weight_slowdown = 0.6; // braking is higher priority
                   weight_change_left = 0.6; // free to turn left
                   weight_change_right = 0.7; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
               }
               else if (veh_pos =2) // vehicle is beside the Ego car
               {
                   weight_slowdown = 0.8; // braking not priority 
                   weight_change_left = 0.2; // free to turn left
                   weight_change_right = 0.2; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
               }  
               else if (veh_pos =3)  // vehicle is in front of the Ego car
               {
                 if(rel_speed ==1) // Ego car is at danger of rear ending the vehicle in front
                 {
                   weight_slowdown = 0.7; // braking is priority 
                   weight_change_left = 0.3; // free to turn left
                   weight_change_right = 0.2; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==0) // Ego car is driving at the same speed
                 {
                   weight_slowdown = 0.6; // braking is higher priority
                   weight_change_left = 0.5; // free to turn left
                   weight_change_right = 0.4; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==2) // Ego car is going slower, relatively low danger
                 {
                   weight_slowdown = 0.6; // braking is higher priority
                   weight_change_left = 0.6; // free to turn left
                   weight_change_right = 0.7; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
               }
             }
             if (veh_zone == 1)
             {
               if (veh_pos ==1) // car is comming from behind
               {
                 if(rel_speed ==1) // Ego car is driving away from the car from behind
                 {
                   weight_slowdown = 0.5; // braking not priority 
                   weight_change_left = 0.6; // free to turn left
                   weight_change_right = 0.5; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==0) // Ego car is driving at the same speed
                 {
                   weight_slowdown = 0.7; // braking is higher priority
                   weight_change_left = 0.4; // free to turn left
                   weight_change_right = 0.3; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==2) // Ego car is going slower, danger of being rear ended
                 {
                   weight_slowdown = 0.8; // braking is higher priority
                   weight_change_left = 0.3; // free to turn left
                   weight_change_right = 0.2; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
               }
               else if (veh_pos =2) // vehicle is beside the Ego car
               {
                   weight_slowdown = 0.9; // braking not priority 
                   weight_change_left = 0.2; // free to turn left
                   weight_change_right = 0.2; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
               }  
               else if (veh_pos =3)  // vehicle is in front of the Ego car
               {
                 if(rel_speed ==1) // Ego car is at danger of rear ending the vehicle in front
                 {
                   weight_slowdown = 0.9; // braking is priority 
                   weight_change_left = 0.2; // free to turn left
                   weight_change_right = 0.2; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==0) // Ego car is driving at the same speed
                 {
                   weight_slowdown = 0.7; // braking is higher priority
                   weight_change_left = 0.4; // free to turn left
                   weight_change_right = 0.3; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
                 else if (rel_speed ==2) // Ego car is going slower, relatively low danger
                 {
                   weight_slowdown = 0.7; // braking is higher priority
                   weight_change_left = 0.5; // free to turn left
                   weight_change_right = 0.4; // free to turn right , turning left is higher priority if both the left lane and the right lane are free
                 }
               }  
             }
             if (veh_zone == 0)
             {
               weight_slowdown = 1; // braking is highest priority
               weight_change_left = 0.1; // no free to turn left
               weight_change_right = 0.1; // no free to turn right
             }
            
              // Deciding which is the prefered intervention
              
              if(weight_change_right > weight_change_left) // if right lane change is preferred or left lane change is dangerous
              {
			  // changing to the left lane is not safe
                check_Lane_change_left = true;
              }
              else if (weight_change_right <= weight_change_left) // If left lane change is prefered or right lane change is dangerous
              {
			  // changing to the right lane is not safe
                check_Lane_change_right = true;
              }
              else
              {
                 // no danger was found
                 check_Lane_change_left = false;
                 check_Lane_change_right = false;
              }
          }
          
          if (veh_intv ==1)
          {
            if(check_Lane_change_left ==0 && lane > 0 && weight_change_left >= 0.4) // checking if left lane is free (preferred overtaking lane)
            {
              lane -=1;
              std::cout<<"swerve left: weight = "<< weight_change_left<<std::endl;

            }
            else if (check_Lane_change_right ==0 && lane < 2 && weight_change_right >= 0.4) // checking if right lane is free
            {
              lane +=1;
              std::cout<<"swerve right: weight = "<< weight_change_right <<std::endl;
            }
            else // lane change is not possible
            {
              ref_vel -=0.4*weight_slowdown; // reduce speed
              std::cout<<"slow down: weight"<< weight_slowdown <<std::endl;
            }
          }
          else if(ref_vel <49.5) // no obstacles accelerate to reference speed
          {
            ref_vel +=0.224;
            //std::cout<<"accelerate"<< too_close <<std::endl;
          }
          
          std::cout<<" lane to follow ="<< lane<<std::endl; 
          

          //std::cout<<"slow down"<< too_close <<std::endl;
          //std::cout<<"slow down"<< too_close <<std::endl;    
          
          // Code part to get the car moving in a lane smoothly
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // use current position if the car is just starting meaning if the previous size is close to zero
          if (prev_size < 2)
          {
            // use 2 points that make the path tangent of the car
            double prev_car_x = car_x -cos(car_yaw);
            double prev_car_y = car_y -sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // when the car is not freshly starting off, use the previous path's end point as the starting reference
          else
          {
            //redefine previous state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            // draw the tangent using the tw0 previous points
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
          }
          
          //In freenet add equidistantly spaced points at 30mfrom the start reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i=0; i < ptsx.size(); i++)
          {
            // shift reference to car as the point of observation
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw)- shift_y* sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+ shift_y* cos(0-ref_yaw));
          }
          
          // create a spline
          tk::spline s;
          
          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);
          
          //(x,y) points to the planner
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;
          
          // starting with previous path points
          for(int i=0; i < previous_path_x.size(); i++)
          {
            //set next values
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
                   
          // break up spline points to match the desired velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          
          double x_add_on = 0;
          
          //outputting 50 points
          
          for(int i=0; i <= 50-previous_path_x.size(); i++){
            
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Get the view back from the cars prospective to the global prospective 
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

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