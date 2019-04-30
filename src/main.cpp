#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "filter.h"
#include "helper.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // DATA LOGGING
  const char *path1 = "/home/workspace/CarND-PID-Control-Project/output_files/datalog_1.txt";
  ofstream outfile(path1);
  outfile.close();

  PID pid;
  PID pid_t_speed;
  PID pid_t_cte;
  PID pid_t_steer;
  filter filter_steer;
  /**
   * TODO: Initialize the pid variable.
   */
  //  PID - Steering Value
  double init_Kp = 0.15;
  double init_Kd = 4.05;
  double init_Ki = 0.0005;
  pid.Init(init_Kp,init_Ki,init_Kd);

  // PID - Throttle, Error = Delta Speed
  double init_Kp_t1 = .25;
  double init_Kd_t1 = -2.0; // try a negative value
  double init_Ki_t1 = 0.0;
  pid_t_speed.Init(init_Kp_t1,init_Ki_t1,init_Kd_t1);

  // PID - Throttle, Error = CTE
  double init_Kp_t2 = 2.0;
  double init_Kd_t2 = 20.0; // try 2 or 20
  double init_Ki_t2 = 0.0005;
  pid_t_cte.Init(init_Kp_t2,init_Ki_t2,init_Kd_t2);

  // PID - Throttle, Error = Steering Value
  double init_Kp_t3 = 4.0;
  double init_Kd_t3 = 40.0; // try 4 or 40
  double init_Ki_t3 = 0.0;
  pid_t_steer.Init(init_Kp_t3,init_Ki_t3,init_Kd_t3);

  double alpha = 0.99;
  double msp = 0;         // max speed
  filter_steer.Init(alpha);

  // 3rd Approach to Controller
  double c_time = 0.0; // Current frame time
  double p_time = clock(); // Previous frame time
  double t = 0.0; // Total time

  h.onMessage([&pid,&pid_t_cte,&pid_t_speed,&pid_t_steer, &filter_steer, &msp,
                     &c_time, &p_time, &t, path1](uWS::WebSocket<uWS::SERVER> ws,
                     char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle;
          double desired_speed = 80;
          double error_speed = desired_speed - speed;
          double error_cte;
          double error_steering;
          vector<double> data1;

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // data logging
          #if(false)
            if ((pid.get_stp()+0)%2 == 0) {
              data1 = {speed, throttle, steer_value, angle, cte};
              std::ofstream outfile;
              outfile.open(path1,std::ios_base::app);
              updateTextFile(outfile, data1);
              outfile.close();
            }
          #endif

          // calculate steering
          #if(false) // Approach #1 & #2
            steer_value = pid.update_val(cte);
            steer_value = filter_steer.smooth(steer_value);       // smooth out changes
          #else // Approach #3
            c_time = clock();
            double dt = (c_time - p_time) / CLOCKS_PER_SEC;
            steer_value = pid.update_steering_lin(cte, speed, dt);
          #endif

          // throttle update Simple PID
          //throttle = 0.3;
          #if(false) // Approach #1
            if (speed > 30) {                 // speed > 30
              throttle = 0.3;
            } else {                          // speed < 30
              throttle = 0.35;
            }
            if (speed > 28 and fabs(cte) > 0.2 and fabs(steer_value) > 0.25) {
              throttle = -0.2;                //break
            }
          #endif

          // Piecewise limits on error_cte & error_steering
          #if(false)
            double limBand1 = 0.00;
            double limBand2 = 0.00;
            if (fabs(cte) < limBand1) {error_cte = 0;}
            else {error_cte = cte;}
            if (fabs(steer_value) < limBand2) {error_steering = 0;}
            else {error_steering = steer_value;}
          #else
            error_cte = cte;
            error_steering = steer_value;
          #endif

          // calculate throttle Complex PID
          #if(false) // Approach #2
            throttle = -pid_t_speed.update_val(error_speed)
                      +pid_t_cte.update_val(error_cte)
                      +pid_t_steer.update_val(fabs(error_steering));
          #else // Approach #3
            throttle = 0.8;
            if (fabs(cte) > 0.5){
              throttle = 0.5;
            }
            if (fabs(pid.prev_cte - cte) > 0.1 and fabs(pid.prev_cte - cte) <= 0.2){
              throttle = 0.0;
            } else if (fabs(pid.prev_cte - cte) > 0.2 and speed > 30){
              throttle = -0.2; // Break!
            }
          #endif

          // Keep Steering Value Within Bounds
          if (steer_value < -1) {
            steer_value = -1;
          } else if (steer_value > 1) {
            steer_value = 1;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          if (speed > msp) {
            msp = speed;
          }
          std::cout << "Max Speed: " << msp << " mph : Current Speed: " << speed << " mph"<<std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
