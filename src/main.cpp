#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream> 

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main( int argc, char *argv[] ) {
    uWS::Hub h;

    PID pid;

    PidAutoTuner tuner ;

    bool reset_flag = false ;

    // Initialize the pid variables.

    pid.Init(  argc > 1 ? atof( argv[1] ) : 0.15,
               argc > 2 ? atof( argv[2] ) : 0.0004,
               argc > 3 ? atof( argv[3] ) : 1.5  ) ;

    std::ofstream ofs ;


    std::cout << "P: " << pid.Kp << " I: " << pid.Ki  << " D: " << pid.Kd << std::endl ;

    h.onMessage([&pid,&tuner,&ofs,&reset_flag](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if ( s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;

                    // update the pid with crosstrack error
                    pid.UpdateError( cte ) ;

                    //
                    steer_value = -pid.TotalError() ;

                    // make sure steering value stays between [-1,1]
                    if ( steer_value < -1.0 )
                        steer_value = -1.0 ;
                    else if ( steer_value > 1.0 )
                        steer_value = 1.0 ;

                    // send current error to auto-tuner - to twiddle the parameters

                    if ( tuner.Update(  cte  , pid ) )
                    {
                      // we request a scene reload - so we can start with same initial conditions.
                      std::string msg = "42[\"reset\",{}]";
                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                      ofs.close() ;
                      reset_flag = false ;
                    }
                    

                    ofs << cte << std::endl ;


                    json msgJson;
                    msgJson["steering_angle"] = steer_value;

                    // lets be slightly more ambitious and use a higher throttle value
                    msgJson["throttle"] = 0.4;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h,&reset_flag,&ofs](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        if (!reset_flag) {
            std::cout << "Connected!!!" << std::endl;

            ofs.open( "plot.dat" , std::ofstream::out | std::ofstream::trunc ) ;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            reset_flag = true ;
          }
    });

    h.onDisconnection([&h,&reset_flag,&ofs](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        ofs.close() ;
        std::cout << "Disconnected" << std::endl;
        reset_flag = false ;
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
