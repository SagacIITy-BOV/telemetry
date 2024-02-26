#include<ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
//#include <geodesy/utm.h>
#include <stdlib.h>
#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Variables to hold subscibed data
//float steering_angle_kbd;
float steering_angle_encoder;
float cur_loc_x;
float cur_loc_y;
float cur_gSpeed;
float cur_heading;
float cur_heading_comp;
float cur_throttle_value;


//vector<float> ptsx;
//vector<float> ptsy;
const int Num_WP = 15;
float ptsx_float[Num_WP] = {};
float ptsy_one_elem;
float ptsy_float[Num_WP] = {};


float asd;

//std_msgs::Float64MultiArray array_msg;
double test;

/*
void callbackSteeringAngleKbd(const std_msgs::Float32& msg){
  steering_angle_kbd = msg.data;  // it can be anywhere between -500 deg to +500 deg
  steering_angle_kbd = deg2rad(steering_angle_kbd*8.336535919);

} */

void callbackSteeringAngle(const std_msgs::Float32& msg){
  steering_angle_encoder = msg.data ;  // it can be anywhere between -670 deg to +670 deg
  steering_angle_encoder = deg2rad(steering_angle_encoder/11.166666667);  // mapping to (-pi/3,pi/3)
}

void callbackCurLocX(const std_msgs::Float32& msg){
  cur_loc_x = msg.data;
}

void callbackCurLocY(const std_msgs::Float32& msg){
  cur_loc_y = msg.data;
}

void callbackPtsX(const std_msgs::Float32MultiArray::ConstPtr& msg){
  int i = 0;
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		ptsx_float[i] = *it;
		i++;
	}
}

void callbackPtsY(const std_msgs::Float32MultiArray::ConstPtr& msg){
  int i = 0;
  ptsy_one_elem = *msg->data.begin();
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		ptsy_float[i] = *it;
		i++;
	}
}

void callbackCurGSpeed(const std_msgs::Float32& msg){
  cur_gSpeed = msg.data;
}

void callbackCurHeading(const std_msgs::Float32& msg){
  cur_heading = msg.data;
}

void callbackCurHeadingComp(const std_msgs::Float32& msg){
  cur_heading_comp = msg.data;
}

void callbackTC(const std_msgs::Float32& msg){
  cur_throttle_value = msg.data;
}

#define COLS 2

float run_avg_steer_value=0.0;
float run_tot = 0.0;
float leaving_val;
const int run_len = 10;
int run_count = 0;
float run_window[run_len] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main(int argc, char **argv){

  // MPC is initialized here!
  MPC mpc;

  ros::init(argc,argv,"publish_velocity");
  ros::NodeHandle nh;

  //ros::Subscriber sub_steering_ang_kbd = nh.subscribe("/keyboard/steering_command", 1000, &callbackSteeringAngleKbd); // in degrees
  ros::Subscriber sub_steering_ang_encd = nh.subscribe("/steering_angle", 1000, &callbackSteeringAngle); // in degrees
  ros::Subscriber sub_cur_loc_x = nh.subscribe("/cur_loc_x", 1000, &callbackCurLocX);
  ros::Subscriber sub_cur_loc_y = nh.subscribe("/cur_loc_y", 1000, &callbackCurLocY);
  //ros::Subscriber sub_cur_loc_z = nh.subscribe("/cur_loc_z", 1000, &callbackCurLocZ);
  ros::Subscriber sub_ptsx = nh.subscribe("/ptsx", 1000, &callbackPtsX);
  ros::Subscriber sub_ptsy = nh.subscribe("/ptsy", 1000, &callbackPtsY);
  ros::Subscriber sub_cur_gSpeed = nh.subscribe("/cur_gSpeed", 1000, &callbackCurGSpeed);
  ros::Subscriber sub_cur_heading = nh.subscribe("/cur_heading", 1000, &callbackCurHeading);
  ros::Subscriber sub_cur_heading_Comp = nh.subscribe("/cur_heading_comp", 1000, &callbackCurHeadingComp);
  ros::Subscriber sub_cur_TC = nh.subscribe("/throttle_enc", 1000, &callbackTC);

  ros::Publisher pub_steering = nh.advertise<std_msgs::Float32> ("steering_command",1000);
  ros::Publisher pub_throttle = nh.advertise<std_msgs::Float32> ("throttle_command",1000);
  ros::Publisher pub_MPCx = nh.advertise<std_msgs::Float32MultiArray> ("MPCx",1000);
  ros::Publisher pub_MPCy = nh.advertise<std_msgs::Float32MultiArray> ("MPCy",1000);
  ros::Publisher pub_NEXTx = nh.advertise<std_msgs::Float32MultiArray> ("NEXTx",1000);
  ros::Publisher pub_NEXTy = nh.advertise<std_msgs::Float32MultiArray> ("NEXTy",1000);

  ros::spinOnce();
  
  // Initialized two float arrays and 7 float variables
  float st_ang;
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  float px;
  float py;
  float psi_gps;
  float psi_comp;
  float psi;
  float v;
  float steer_value;
  float steer_value_encode; //in degrees: can be from -670 to +670
  float steer_angle2MPC;  // in degrees from -60 to +60
  float throttle_value;
  float a;
  

  ros::Rate rate(10);

    
  while(ros::ok()){
   
    ptsx.insert(ptsx.begin(), std::begin(ptsx_float), std::end(ptsx_float));
    ptsy.insert(ptsy.begin(), std::begin(ptsy_float), std::end(ptsy_float));
    px = cur_loc_x;
    py = cur_loc_y;
    psi_gps = cur_heading; // in radians from GPS_RTK
    psi_comp = cur_heading_comp;   // in radians from HMC5882
    v = cur_gSpeed*5/18;    // in kmph from GPS_RTK. Being converted to mps

    steer_value_encode = steering_angle_encoder;
    //steer_value_encode = steering_angle_kbd;

    a = 0;
    if(cur_gSpeed<2.0){
      ROS_INFO_STREAM(".............................................................");
      ROS_INFO_STREAM("psi has been received from COMPASS");
      psi = psi_comp;
    }else
    {
      ROS_INFO_STREAM(".............................................................");
      ROS_INFO_STREAM("psi has been received from GPS");
      psi = psi_gps;
    }
   

    if (ptsx[0]!=0.0){
      vector<double> waypoints_x;
      vector<double> waypoints_y;
      // transform waypoints to be from car's perspective
      // this means we can consider px = 0, py = 0, and psi = 0
      // greatly simplifying future calculations
      /**
      ROS_INFO_STREAM("Before converting to local coordinate system");
      //ROS_INFO_STREAM("px = " << px << ", py = " << py << ", psi (deg) = " << rad2deg(psi) << ", v in mps (received in kmph) = " << v);
      //ROS_INFO_STREAM("steer_value_encoder (in degrees) = " << (steer_value_encode));
      ROS_INFO_STREAM("ptsx = " << ptsx[0] << ", " << ptsx[1] << ", " << ptsx[2] << ", " << ptsx[3] << ", " << ptsx[4] << ", " << ptsx[5]<<", " << ptsx[6]<<", " << ptsx[7]<<", " << ptsx[8]);
      ROS_INFO_STREAM("ptsx_cont = " << ptsx[9] << ", " << ptsx[10] << ", " << ptsx[11] << ", " << ptsx[12] << ", " << ptsx[13] << ", " << ptsx[14]);
      ROS_INFO_STREAM("ptsy = " << ptsy[0] << ", " << ptsy[1] << ", " << ptsy[2] << ", " << ptsy[3] << ", " << ptsy[4] << ", " << ptsy[5]<<", " << ptsy[6]<<", " << ptsy[7]<<", " << ptsy[8]);
      ROS_INFO_STREAM("ptsy_cont = " << ptsy[9] << ", " << ptsy[10] << ", " << ptsy[11] << ", " << ptsy[12] << ", " << ptsy[13] << ", " << ptsy[14]);
      //ROS_INFO_STREAM("ptsy = " << ptsy[0] << ", " << ptsy[1] << ", " << ptsy[2] << ", " << ptsy[3] << ", " << ptsy[4] << ", " << ptsy[5]);
      //ROS_INFO_STREAM("ptsy_one_elem = "<< ptsy_one_elem);
      **/
      for (int i = 0; i < ptsx.size(); i++) {
        double dx = ptsx[i] - px;
        double dy = ptsy[i] - py;
        waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
        waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
        //ROS_INFO_STREAM("dx = " << px << "dy = " << py);
      }
      
      ROS_INFO_STREAM("After converting to local coordinate system");
      ROS_INFO_STREAM("waypoints_x = " << waypoints_x[0] << ", " << waypoints_x[1] << ", " << waypoints_x[2] << ", " << waypoints_x[3] << ", " << waypoints_x[4] << ", " << waypoints_x[5]<< ", " << waypoints_x[6] << ", " << waypoints_x[7] << ", " << waypoints_x[8] << ", " << waypoints_x[9] );
      ROS_INFO_STREAM("waypoints_x_contd = " << waypoints_x[10] << ", " << waypoints_x[11] << ", " << waypoints_x[12] << ", " << waypoints_x[13] << ", " << waypoints_x[14] );
      ROS_INFO_STREAM("waypoints_y = " << waypoints_y[0] << ", " << waypoints_y[1] << ", " << waypoints_y[2] << ", " << waypoints_y[3] << ", " << waypoints_y[4] << ", " << waypoints_y[5]<< ", " << waypoints_y[6] << ", " << waypoints_y[7] << ", " << waypoints_y[8] << ", " << waypoints_y[9] );
      ROS_INFO_STREAM("waypoints_y_contd = " << waypoints_y[10] << ", " << waypoints_y[11] << ", " << waypoints_y[12] << ", " << waypoints_y[13] << ", " << waypoints_y[14] );
      //ROS_INFO_STREAM("waypoints_y = " << waypoints_y[0] << ", " << waypoints_y[1] << ", " << waypoints_y[2] << ", " << waypoints_y[3] << ", " << waypoints_y[4] << ", " << waypoints_y[5]);
      
      double* ptrx = &waypoints_x[0];
      double* ptry = &waypoints_y[0];
      Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, Num_WP);
      Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, Num_WP);

      auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
      ROS_INFO_STREAM("Poly coefficients = " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3]);
      double cte = polyeval(coeffs, 0);  // px = 0, py = 0
      double epsi = -atan(coeffs[1]);  // p
      ROS_INFO_STREAM("cte (m) = " << cte << ", epsi (deg) = " << rad2deg(epsi) );
      

      //double steer_value = j[1]["steering_angle"];
      //double throttle_value = j[1]["throttle"];
      double Lf = 1.57;
      double latency = 0;
      //double pred_px = px+v*cos(psi)*latency; 
      //double pred_py = py + v*sin(psi)*latency; 
      double pred_px = 0;
      double pred_py = 0;
      double pred_psi = v*(-steer_value_encode/Lf) * latency;
      double pred_v = v + a*latency;
      double pred_cte = cte + v*sin(epsi)*latency;
      double pred_epsi = epsi + v*(-steer_value_encode/Lf)*latency;


      Eigen::VectorXd state(6);
      state[0] = pred_px;
      state[1] = pred_py;
      state[2] = pred_psi;
      state[3] = pred_v;
      state[4] = pred_cte;
      state[5] = pred_epsi;
      //Eigen::VectorXd state(6);
      //state << 0, 0, 0, v, cte, epsi;
      auto vars = mpc.Solve(state, coeffs);
      steer_value = rad2deg(vars[0]) * 11.16666667;   // convert from (-pi/3,pi/3) to (-670,670)----- from encoder
      //steer_value = rad2deg(vars[0]) * (3.5) +250;   // convert from (-pi/3,pi/3) to (-500,500)----- from keyboard
      throttle_value = vars[1];

      //run_count++;
      for (int i = 0; i < run_len-1; i++) {
        run_window[i] = run_window[i+1];
      }
      run_window[run_len-1] = steer_value;
      run_tot = 0.0;
      for (int i = 0; i <= run_len-1; i++) {
        run_tot = run_tot + run_window[i];
      }
      run_avg_steer_value = run_tot/run_len;

      //Display the MPC predicted trajectory 
      vector<double> mpc_x_vals;
      vector<double> mpc_y_vals;

      //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
      // the points in the simulator are connected by a Green line

      for (int i = 2; i < vars.size(); i ++) {
        if (i%2 == 0) {
          mpc_x_vals.push_back(vars[i]);
        }
        else {
          mpc_y_vals.push_back(vars[i]);
        }
      }
      //ROS_INFO_STREAM("MPC Points x = " << mpc_x_vals[0] << ", " << mpc_x_vals[3] << ", " << mpc_x_vals[7] << ", " << mpc_x_vals[11] << ", " << mpc_x_vals[15] << ", " << mpc_x_vals[18]);
      //ROS_INFO_STREAM("MPC Points y = " << mpc_y_vals[0] << ", " << mpc_y_vals[3] << ", " << mpc_y_vals[7] << ", " << mpc_y_vals[11] << ", " << mpc_y_vals[15] << ", " << mpc_y_vals[18]);
      
      //Display the waypoints/reference line
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
      // the points in the simulator are connected by a Yellow line

      for (double i = 0; i < 30; i += 2){
        next_x_vals.push_back(i);
        next_y_vals.push_back(polyeval(coeffs, i));
      }

      //ROS_INFO_STREAM("NEXT Points x = " << next_x_vals[0] << ", " << next_x_vals[2] << ", " << next_x_vals[4] << ", " << next_x_vals[6] << ", " << next_x_vals[8] << ", " << next_x_vals[9]);
      //ROS_INFO_STREAM("NEXT Points y = " << next_y_vals[0] << ", " << next_y_vals[2] << ", " << next_y_vals[4] << ", " << next_y_vals[6] << ", " << next_y_vals[8] << ", " << next_y_vals[9]);

    
    //ROS_INFO_STREAM("x = [ " << ptsx[0] <<", " << ptsx[1] <<", " << ptsx[2] <<", " << ptsx[3] <<", " << ptsx[4] <<", " << ptsx[5] <<"]");
    //ROS_INFO_STREAM("y = [ " << ptsy[0] <<", " << ptsy[1] <<", " << ptsy[2] <<", " << ptsy[3] <<", " << ptsy[4] <<", " << ptsy[5] <<"]");
	  //ROS_INFO_STREAM("steering_angle_from_encoder = " << steering_angle_encoder.data);
    //ROS_INFO_STREAM("x = " << px << ", y = " << py << ", v = " << v << ", psi = " << psi);

    //drive_command[0] = 280.0;

    std_msgs::Float32 msg_throttle_command;
    std_msgs::Float32 msg_steering_command;
    std_msgs::Float32MultiArray msg_MPCx;
    std_msgs::Float32MultiArray msg_MPCy;
    std_msgs::Float32MultiArray msg_NEXTx;
    std_msgs::Float32MultiArray msg_NEXTy;

    // create an empty vector and push all elements
	  // set up dimensions
    msg_MPCx.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_MPCx.layout.dim[0].size = mpc_x_vals.size();
    msg_MPCx.layout.dim[0].stride = 1;
    msg_MPCx.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1
    msg_MPCx.data.clear();
    msg_MPCx.data.insert(msg_MPCx.data.end(), mpc_x_vals.begin(), mpc_x_vals.end());

    msg_MPCy.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_MPCy.layout.dim[0].size = mpc_y_vals.size();
    msg_MPCy.layout.dim[0].stride = 1;
    msg_MPCy.layout.dim[0].label = "y"; // or whatever name you typically use to index vec1
    msg_MPCy.data.clear();
    msg_MPCy.data.insert(msg_MPCy.data.end(), mpc_y_vals.begin(), mpc_y_vals.end());

    msg_NEXTx.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_NEXTx.layout.dim[0].size = next_x_vals.size();
    msg_NEXTx.layout.dim[0].stride = 1;
    msg_NEXTx.layout.dim[0].label = "l"; // or whatever name you typically use to index vec1
    msg_NEXTx.data.clear();
    msg_NEXTx.data.insert(msg_NEXTx.data.end(), next_x_vals.begin(), next_x_vals.end());

    msg_NEXTy.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_NEXTy.layout.dim[0].size = next_y_vals.size();
    msg_NEXTy.layout.dim[0].stride = 1;
    msg_NEXTy.layout.dim[0].label = "m"; // or whatever name you typically use to index vec1
    msg_NEXTy.data.clear();
    msg_NEXTy.data.insert(msg_NEXTy.data.end(), next_y_vals.begin(), next_y_vals.end());

    msg_steering_command.data = -run_avg_steer_value;// putting a negative sign to go with the convention of steering_control: clockwise positive
    msg_throttle_command.data = throttle_value;


    pub_steering.publish(msg_steering_command);
    pub_throttle.publish(msg_throttle_command);
    pub_MPCx.publish(msg_MPCx);
    pub_MPCy.publish(msg_MPCy);
    pub_NEXTx.publish(msg_NEXTx);
    pub_NEXTy.publish(msg_NEXTy);

    //ROS_INFO_STREAM("Steering command: " << msg_steering_command.data);
    //ROS_INFO_STREAM("Throttle command: " << msg_throttle_command.data);
     
    }
    ros::spinOnce();
    rate.sleep();
  }
return 0;
}
