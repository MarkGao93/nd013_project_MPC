/**************************************************************************************************
 * @ File: main.cpp
 * @ Author: MarkGao
 * @ Date: 2020-05-15
 * @ Email: 819699632@qq.com
 * @ Version: 1.0
 * @ Description:
**************************************************************************************************/
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using std::string;
using std::cout;
using std::endl;

//double dt;    // 100ms
const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2kmh(double x) { return x * 1.609344; }
double kmh2mph(double x) { return x * 0.6213712; }

string data_file = "./dataLog.csv";
std::ofstream logData(data_file);
bool is_data_log = true;



/**************************************************************************************************
 * @ Function: hasData()
 * @ Description: Checks if the SocketIO event has JSON data.
 * @ Calls:
 * @ Input: string s
 * @ Output: void
 * @ Return: If there is data, the JSON object in string format will be returned,
 *           else the empty string "" will be returned.
 * @ Others:
**************************************************************************************************/
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");

    if(found_null != string::npos)
    {
        return "";
    }
    else if(b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }

    return "";
}



/**************************************************************************************************
 * @ Function: polyeval()
 * @ Description: Evaluate a polynomial.      // Description of the function
 * @ Calls:                       // Functions being called by this function
 * @ Input:                       // Description of input parameters, include the function, value range of each parameter
 * @ Output:                    // Description of output parameters
 * @ Return:                    // Description of return value
 * @ Others:
**************************************************************************************************/
double polyeval(Eigen::VectorXd coeffs, double x)
{
	double result = 0.0;
    for(int i=0; i<coeffs.size(); i++)
    {
		result += coeffs[i] * pow(x, i);
	}

	return result;
}



/**************************************************************************************************
 * @ Function: polyfit()
 * @ Description: Fit a polynomial.    // Description of the function
 * @ Calls:                       // Functions being called by this function
 * @ Input:                       // Description of input parameters, include the function, value range of each parameter
 * @ Output:                    // Description of output parameters
 * @ Return:                    // Description of return value
 * @ Others: Adapted from
 *           https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
**************************************************************************************************/
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

    for(int i=0; i<xvals.size(); i++)
    {
		A(i, 0) = 1.0;
	}

    for(int j=0; j<xvals.size(); j++)
    {
        for(int i=0; i<order; i++)
        {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);

	return result;
}



/**************************************************************************************************
 * @ Function: main()
 * @ Description:      // Description of the function
 * @ Calls:                       // Functions being called by this function
 * @ Input:                       // Description of input parameters, include the function, value range of each parameter
 * @ Output:                    // Description of output parameters
 * @ Return:                    // Description of return value
 * @ Others:
**************************************************************************************************/
int main()
{
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;
    if(is_data_log)
    {
        logData
                << "c_x" << ","
                << "c_y" << ","
                << "c_psi_degree" << ","
                << "psi_unity_degree" << ","
                << "c_speed_kmh" << ","
                << "steering_angle" << ","
                << "throttle" << ","
                << "a0" << ","
                << "a1" << ","
                << "a2" << ","
                << "a3" << ","
                << "cte" << ","
                << "epsi" << ","
                << "global_x_1" << ","
                << "global_x_2" << ","
                << "global_x_3" << ","
                << "global_x_4" << ","
                << "global_x_5" << ","
                << "global_x_6" << ","
                << "global_y_1" << ","
                << "global_y_2" << ","
                << "global_y_3" << ","
                << "global_y_4" << ","
                << "global_y_5" << ","
                << "global_y_6" << ","
                << "veh_x_1" << ","
                << "veh_x_2" << ","
                << "veh_x_3" << ","
                << "veh_x_4" << ","
                << "veh_x_5" << ","
                << "veh_x_6" << ","
                << "veh_y_1" << ","
                << "veh_y_2" << ","
                << "veh_y_3" << ","
                << "veh_y_4" << ","
                << "veh_y_5" << ","
                << "veh_y_6" << ","
                << std::endl;
    }


    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws,
                       char *data,
                       size_t length,
                       uWS::OpCode opCode)
    {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);

        if(sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
			string s = hasData(sdata);
            if(s != "")
            {
				auto j = json::parse(s);
				string event = j[0].get<string>();
                if(event == "telemetry")
                {
                    // >>>>> Get JSON object
					// j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];    // (Array) - The global x positions of the waypoints. the nearest 6 points
                    vector<double> ptsy = j[1]["ptsy"];    // (Array) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction. the nearest 6 points
                    double px = j[1]["x"];    // (float) - The global x position of the vehicle.
                    double py = j[1]["y"];    // (float) - The global y position of the vehicle.
                    double psi = j[1]["psi"];    // (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions
                    double psi_unity = j[1]["psi_unity"];    // (float) - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
                    double v = j[1]["speed"];    // (float) - The current velocity in mph.
                    double steering_angle = j[1]["steering_angle"];    // (float) - The current steering angle in radians. left is negative, right is positive
                    double throttle = j[1]["throttle"];    // (float) - The current throttle value [-1, 1].

                    cout<<"\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
//                    cout<<"!!! Print JSON data !!!"<<endl;
//                    cout<<"!!! ptsx size = "<<ptsx.size()<<endl;
//                    cout<<"!!! ptsy size = "<<ptsy.size()<<endl;
                    for(int i=0; i<ptsx.size(); i++)
                    {
                        cout<<"!!! ptsx = "<<ptsx[i]<<"\t"<<"ptsy = "<<ptsy[i]<<endl;
                    }
                    cout<<"!!! px = "<<px<<endl;
                    cout<<"!!! py = "<<py<<endl;
//                    cout<<"!!! psi_rad = "<<psi<<" rad"<<endl;
                    cout<<"!!! psi_degree = "<<rad2deg(psi)<<" degree"<<endl;
//                    cout<<"!!! psi_unity_rad = "<<psi_unity<<" rad"<<endl;
                    cout<<"!!! psi_unity_degree = "<<rad2deg(psi_unity)<<" degree"<<endl;
//                    cout<<"!!! v = "<<v<<" mph"<<endl;
                    cout<<"!!! v = "<<mph2kmh(v)<<" kmh"<<endl;
//                    cout<<"!!! steering_angle = "<<rad2deg(steering_angle)<<" degrees"<<endl;
//                    cout<<"!!! steering_angle = "<<steering_angle * 100<<" %"<<endl;
//                    cout<<"!!! throttle = "<<throttle<<endl;
//                    cout<<"!!! throttle = "<<throttle * 100<<" %"<<endl;



                    // TODO: Calculate steering angle and throttle using MPC. Both are in between [-1, 1].


                    // >>>>> Transform coordinate to get epsi and cte
                    // Transform waypoints from Map/Global coordinate into vehicle coordinate
                    // Vehicle coordinate : x to front, y to right
                    Eigen::VectorXd x_veh_coor(ptsx.size());
                    Eigen::VectorXd y_veh_coor(ptsy.size());
                    for(int i=0; i<ptsx.size(); i++)
                    {
						const double xx = ptsx[i] - px;
						const double yy = ptsy[i] - py;
						x_veh_coor[i] = xx * cos(-psi) - yy * sin(-psi);
						y_veh_coor[i] = xx * sin(-psi) + yy * cos(-psi);
                        cout<<"x_veh_coor = "<<x_veh_coor[i]<<"\t"<<"y_veh_coor = "<<y_veh_coor[i]<<endl;
					}

                    auto coeffs = polyfit(x_veh_coor, y_veh_coor, 3);    // Get a0, a1, a2, a3
					const double epsi = -atan(coeffs[1]);
                    const double cte = polyeval(coeffs, 0);    // set x=0, obtain a0, path position
                    cout<<"!!! epsi = "<<epsi<<endl;
                    cout<<"!!! cte = "<<cte<<endl;
                    cout<<"!!! a0 = "<<coeffs[0]<<endl;
                    cout<<"!!! a1 = "<<coeffs[1]<<endl;
                    cout<<"!!! a2 = "<<coeffs[2]<<endl;
                    cout<<"!!! a3 = "<<coeffs[3]<<endl;


                    // log data
                    if(is_data_log)
                    {
                        logData
                                << px << ","
                                << py << ","
                                << rad2deg(psi) << ","
                                << rad2deg(psi_unity) << ","
                                << mph2kmh(v) << ","
                                << rad2deg(steering_angle) << ","
                                << throttle * 100 << ","
                                << coeffs[0] << ","
                                << coeffs[1] << ","
                                << coeffs[2] << ","
                                << coeffs[3] << ","
                                << cte << ","
                                << epsi << ","
                                << ptsx[0] << ","
                                << ptsx[1] << ","
                                << ptsx[2] << ","
                                << ptsx[3] << ","
                                << ptsx[4] << ","
                                << ptsx[5] << ","
                                << ptsy[0] << ","
                                << ptsy[1] << ","
                                << ptsy[2] << ","
                                << ptsy[3] << ","
                                << ptsy[4] << ","
                                << ptsy[5] << ","
                                << x_veh_coor[0] << ","
                                << x_veh_coor[1] << ","
                                << x_veh_coor[2] << ","
                                << x_veh_coor[3] << ","
                                << x_veh_coor[4] << ","
                                << x_veh_coor[5] << ","
                                << y_veh_coor[0] << ","
                                << y_veh_coor[1] << ","
                                << y_veh_coor[2] << ","
                                << y_veh_coor[3] << ","
                                << y_veh_coor[4] << ","
                                << y_veh_coor[5] << ","
                                << std::endl;
                    }


                    // >>>>> State Update
                    // Apply kinematic model, below paratemters are all predicted value in one step future. this is the core of MPC
                    // In Vehicle coordinate, y is 0, psi is 0
					const double px_act = v * dt;					
					const double py_act = 0;
					const double psi_act = -v * steering_angle * dt / Lf;
					const double v_act = v + throttle * dt;
					const double cte_act = cte + v * sin(epsi) * dt;
					const double epsi_act = epsi + psi_act;

					Eigen::VectorXd state(6);
					state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;

					vector<double> final_mpc_results = mpc.Solve(state, coeffs);

                    // >>>>> Output control signals
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    double steer_value;    // To store steer control input
                    double throttle_value;    // To store throttle control input
                    steer_value = final_mpc_results[0] / (deg2rad(25)*Lf);    // convert to [-1, 1] range
					throttle_value = final_mpc_results[1];

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;


                    // >>>>> Display MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    for(int i=0; i<final_mpc_results.size()/2-1; i++)
                    {
                        mpc_x_vals.push_back(final_mpc_results[2*i+2]);
                        mpc_y_vals.push_back(final_mpc_results[2*i+3]);
                    }
                    // Add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // points in the simulator are connected by a Green line
					msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
//                    cout<<"!!! final_mpc_results size = "<<final_mpc_results.size()<<endl;
//                    cout<<"!!! final_mpc_results x = "<<mpc_x_vals[0]<<endl;
//                    cout<<"!!! final_mpc_results y = "<<mpc_x_vals[1]<<endl;


                    // >>>>> Display the waypoints/reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;
                    for(int i=0; i<ptsx.size(); i++)
                    {
                        next_x_vals.push_back(x_veh_coor[i]);
                        next_y_vals.push_back(y_veh_coor[i]);
                    }
                    // Add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // points in the simulator are connected by a Yellow line
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;



					auto msg = "42[\"steer\"," + msgJson.dump() + "]";


					// Latency
					// The purpose is to mimic real driving conditions where
                    // the car does NOT actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }    // end if elementary
            }    // end if s is not void
            else
            {
				// Manual driving
                string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
        }    // end s data begins with 4 and 2
    });    // end h.onMessage

    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
    {
        const string s = "<h1>Hello world!</h1>";
        if(req.getUrl().valueLength == 1)
        {
			res->end(s.data(), s.length());
		}
        else
        {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        cout<<"!!! Connected !!!"<<endl;
	});

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
		ws.close();
        cout<<"!!! Disconnected !!!"<<endl;
	});

	int port = 4567;
    if(h.listen(port))
    {
        cout<<"!!! Listening to port "<<port<<endl;
	}
    else
    {
        std::cerr<<"!!! Failed to listen to port"<<endl;
		return -1;
	}

	h.run();
}
