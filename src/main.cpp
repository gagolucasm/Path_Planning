#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include <map>
#include <iostream>
#include <vector>

using namespace utils;
using namespace std;

using json = nlohmann::json;

int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  utils::load_map_waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                            map_waypoints_dx, map_waypoints_dy);

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  double target_velocity = 0L;  // meters per sec
  int current_lane = 1;         // starting at middle lane

  h.onMessage([&current_lane, &target_velocity, &map_waypoints_x,
               &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Variable for emergency break
          bool TOO_CLOSE = false;

          int previous_path_size = previous_path_x.size();

          if (previous_path_size > 0) {
            car_s = end_path_s;
          }

          // first we create a fusion_map this is a hash of lanes to cars
          map<int, vector<utils::SensorReading> > fusion_map;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            SensorReading reading = {sensor_fusion[i][0], sensor_fusion[i][1],
                                     sensor_fusion[i][2], sensor_fusion[i][3],
                                     sensor_fusion[i][4], sensor_fusion[i][5],
                                     sensor_fusion[i][6]};

            // check lane left
            if (reading.d > 0 && reading.d <= 4) {
              fusion_map[0].push_back(reading);
            }
            // check lane center
            if (reading.d > 4 && reading.d <= 8) {
              fusion_map[1].push_back(reading);
            }
            // check lane right
            if (reading.d > 8 && reading.d <= 12) {
              fusion_map[2].push_back(reading);
            }
          }

          // check if there is a car too close in our lane.
          auto current_lane_sensor_readings = fusion_map[current_lane];
          for (const auto &reading : current_lane_sensor_readings) {
            double check_car_s =
                reading.s + (static_cast<double>(previous_path_size) * .02 *
                             reading.speed());

            // If there is a car close or we are on the sides
            if (((check_car_s > car_s) && (check_car_s - car_s) < 30) ||
                current_lane != CENTER_LANE) {
              // set too close as True
              if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
                cout << "Too close" << endl;
                TOO_CLOSE = true;
              }

              // if we are in the left, check if we should change
              if (current_lane == LEFT_LANE) {
                auto center_lane_sensor_readings = fusion_map[CENTER_LANE];
                for (const auto &c_reading : center_lane_sensor_readings) {
                  double c_car_s =
                      c_reading.s + (static_cast<double>(previous_path_size) *
                                     .02 * c_reading.speed());
                  // cout << c_car_s - car_s << endl;

                  // if there is no car close behind or the way is clear on the
                  // center lane, change
                  if (((c_car_s - car_s) < -21) || ((c_car_s - car_s) > 40)) {
                    current_lane = CENTER_LANE;
                    // cout << "Moving to CENTER" << endl;
                  } else {
                    current_lane = LEFT_LANE;
                    // cout << "Moving to LEFT" << endl;
                    break;
                  }
                }
                // if we are on the center and too close to a car, we should
                // check moving to the left first
              } else if (current_lane == CENTER_LANE && TOO_CLOSE) {
                auto left_lane_sensor_readings = fusion_map[LEFT_LANE];
                for (const auto &l_reading : left_lane_sensor_readings) {
                  double l_car_s =
                      l_reading.s + (static_cast<double>(previous_path_size) *
                                     .02 * l_reading.speed());

                  // if there is no car close behind or the way is clear on the
                  // left lane, change
                  if (((l_car_s - car_s) < -21) || ((l_car_s - car_s) > 40)) {
                    current_lane = LEFT_LANE;
                    // cout << "Moving to LEFT" << endl;
                  } else {
                    current_lane = CENTER_LANE;
                    // cout << "Moving to CENTER" << endl;
                    break;
                  }
                }

                // if the left lane is not available, check the right one
                if (current_lane == CENTER_LANE && TOO_CLOSE) {
                  auto right_lane_sensor_readings = fusion_map[RIGHT_LANE];
                  for (const auto &r_reading : right_lane_sensor_readings) {
                    double r_car_s =
                        r_reading.s + (static_cast<double>(previous_path_size) *
                                       .02 * r_reading.speed());

                    // if there is no car close behind or the way is clear on
                    // the right lane, change
                    if (((r_car_s - car_s) < -21) || ((r_car_s - car_s) > 40)) {
                      current_lane = RIGHT_LANE;
                      // cout << "Moving to RIGHT" << endl;
                    } else {
                      current_lane = CENTER_LANE;
                      // cout << "Moving to CENTER" << endl;
                      break;
                    }
                  }
                }
                // if we are on the right one, check if we should move to the
                // center
              } else if (current_lane == RIGHT_LANE) {
                auto center_lane_sensor_readings = fusion_map[CENTER_LANE];
                for (const auto &c_reading : center_lane_sensor_readings) {
                  double c_car_s =
                      c_reading.s + (static_cast<double>(previous_path_size) *
                                     .02 * c_reading.speed());

                  // if the way is clear and no car is aproaching behind, change
                  // to the center one
                  if (((c_car_s - car_s) < -21) || ((c_car_s - car_s) > 40)) {
                    current_lane = CENTER_LANE;
                    // cout << "Moving to CENTER" << endl;
                  } else {
                    current_lane = RIGHT_LANE;
                    // cout << "Moving to RIGHT" << endl;
                    break;
                  }
                }
              }
            }
          }

          if (TOO_CLOSE) {
            target_velocity -= .224;
            cout << "Stopping" << endl;
          } else if (target_velocity < utils::MAX_VELOCITY) {
            target_velocity += .224;
            cout << "Hitting the gas" << endl;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if it is the first path
          if (previous_path_size < 2) {
            // generate the first two points
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);

            ptsx.push_back(previous_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(previous_car_y);
            ptsy.push_back(car_y);
          } else {
            // use the last and second to last points from the path generated
            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];

            double previous_ref_x = previous_path_x[previous_path_size - 2];
            double previous_ref_y = previous_path_y[previous_path_size - 2];

            ref_yaw = atan2(ref_y - previous_ref_y, ref_x - previous_ref_x);

            ptsx.push_back(previous_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(previous_ref_y);
            ptsy.push_back(ref_y);
          }

          // Now we add 3 more way points

          for (int i = 1; i <= 3; ++i) {
            auto next_waypoint =
                getXY(car_s + (i * 30), (2 + utils::LANE_WIDTH * current_lane),
                      map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_waypoint[0]);
            ptsy.push_back(next_waypoint[1]);
          }

          // change to local car space.
          utils::globaltToCarSpace(ref_x, ref_y, ref_yaw, ptsx, ptsy);

          tk::spline path_spline;
          path_spline.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // reuse the points that we have
          next_x_vals.insert(next_x_vals.end(), previous_path_x.begin(),
                             previous_path_x.end());
          next_y_vals.insert(next_y_vals.end(), previous_path_y.begin(),
                             previous_path_y.end());

          // calculate how to breakup  spline points
          double target_x = 30L;
          double target_y = path_spline(target_x);
          double target_distance =
              sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;
          double N = target_distance / (0.02 * target_velocity / 2.24);

          for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
            double x_point = x_add_on + (target_x / N);
            double y_point = path_spline(x_point);

            x_add_on = x_point;

            // transform to global space
            next_x_vals.push_back(x_point * cos(ref_yaw) -
                                  y_point * sin(ref_yaw) + ref_x);
            next_y_vals.push_back(x_point * sin(ref_yaw) +
                                  y_point * cos(ref_yaw) + ref_y);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
