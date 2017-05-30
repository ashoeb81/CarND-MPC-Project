#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double psi_unity = j[1]["psi_unity"];
                    double v = j[1]["speed"];


                    // Project reference trajectory from world coordinate frame to vehicle coordinate frame.
                    vector<double> ref_x_vals;
                    vector<double> ref_y_vals;
                    TransformToVehicleCoordinates(ptsx, ptsy, px, py, psi, &ref_x_vals, &ref_y_vals);

                    // Fit 3rd order polynomial to reference trajectory in vehicle coordinate frame.
                    Eigen::Map<Eigen::VectorXd> xvals(&ref_x_vals[0], ref_x_vals.size());
                    Eigen::Map<Eigen::VectorXd> yval(&ref_y_vals[0], ref_y_vals.size());
                    Eigen::VectorXd coeffs = polyfit(xvals, yval, 3);

                    // Calculate initial vehicle state vector that will seed the MPC solver.
                    // 1) In the vehicle coordinate frame, the vehicle is at the origin.  At the origin, the
                    //    cross-track error (cte) is the reference polynomial evaluated at the origin (x=0)
                    double cte = polyeval(coeffs, 0);

                    // 2) In the vehicle coordinate frame, the vehicle pose is psi=0.  So, the pose error (epsi)
                    //    is negative of the tangential angle to the reference polynomial in the vehicle coordinate
                    //    frame evaluated at x=0.
                    double epsi = -atan(coeffs[1]);

                    // 3) In the vehicle coordinate frame, the starting state vector is px=py=psi=0 since the vehicle
                    //    is the origin.
                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;

                    // Calculate steering and throttle values using MPC.
                    auto solution = mpc.Solve(state, coeffs);
                    double steer_value = -solution[0];
                    double throttle_value = solution[1];


                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    // Display the MPC predicted trajectory by looping and extracting x and y coordinates of vehicle
                    // for t=1 to N.
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    int N = (solution.size()-2) / 2;
                    for (int i=0; i < N; i++) {
                        mpc_x_vals.push_back(solution[2 + i]);
                        mpc_y_vals.push_back(solution[2 + N + i]);
                    }
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;


                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    // Convert reference line to vehicle coordinate frame
                    msgJson["next_x"] = ref_x_vals;
                    msgJson["next_y"] = ref_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
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
