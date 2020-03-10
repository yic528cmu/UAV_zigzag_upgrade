/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"

#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Char.h>

#include <iostream>
#include <fstream>

#define AS_PI 3.141590118

using namespace std;
using namespace Eigen;

float DESIRED_DISTANCE_TO_WALL;

float MAX_HORIZONTAL_MOVEMENT; //change this value for horizontal movement

float HEIGHT_STEP;  //change this value for vertical shift step during zigzag movement

float HEIGHT_DESIRE;

int MAX_HEIGHT_LEVEL;

float NO_WALL_TOP_THRESHOLD = 100; // counter threshold

ifstream posData;

const float deg2rad = AS_PI/180.0;
const float rad2deg = 180.0/AS_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlPosYawRatePub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;

bool control_initialized = false;

double x_v_global = 0.0;
double y_v_global = 0.0;
double x_velocity = 0.0;
double y_velocity = 0.0;
double z_velocity = 0.0;
double yaw_rate = 0.0;
double current_yaw, desired_yaw;
double roll, pitch;
double height_cur = 0.0;
double inten = 0.0;
double z_velocity_global = 0.0;
float dist_to_wall;

int control = 0;
int no_wall_counter = 0;
int no_wall_at_top_counter = 0;


double yaw_change = 0;

vector<double> control_gains_p = {0.7,0.7,0.7,1};
vector<double> control_gains_d = {0.07,0.07,0.07,0.06};

vector<double> velocity_final = {0.0, 0.0, 0.0, 0.0};

int counter_move=0; //count the number of movement along the wall
int height_level=0; //count the number of zigzag movement
double z_change=0;

bool obtained = false;

geometry_msgs::Pose Pose_cur_drone, Pose_target_drone;
geometry_msgs::Twist Vel_cur_drone_global, Vel_target_drone, Vel_cur_drone_local;

// ____________________________            DEFAULT FUNCTIONS FOR DJI SDK                   ____________________________________________________
bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;

  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  authority.response.result = false;

  while (!authority.response.result){

    authority.request.control_enable=1;
    sdk_ctrl_authority_service.call(authority);

    if(!authority.response.result)
    {
      ROS_ERROR("obtain control failed!");
    }

  }

  printf("Obtained Control \n");
  return true;

}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

/*!
 * TAKE OFF FOR A3/N3)
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
   display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
   ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
  ros::spinOnce();
}

if(ros::Time::now() - start_time > ros::Duration(5)) {
  ROS_ERROR("Takeoff failed. Motors are not spinnning.");
  return false;
}
else {
  start_time = ros::Time::now();
  ROS_INFO("Motor Spinning ...");
  ros::spinOnce();
}


  // Step 1.2: Get in to the air
while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
  (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
  ros::Time::now() - start_time < ros::Duration(20)) {
  ros::Duration(0.01).sleep();
ros::spinOnce();
}

if(ros::Time::now() - start_time > ros::Duration(20)) {
  ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
  return false;
}
else {
  start_time = ros::Time::now();
  ROS_INFO("Ascending...");
  ros::spinOnce();
}

  // Final check: Finished takeoff
while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
  ros::Time::now() - start_time < ros::Duration(20)) {
  ros::Duration(0.01).sleep();
ros::spinOnce();
}

if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
{
  ROS_INFO("Successful takeoff!");
  start_time = ros::Time::now();
}
else
{
  ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
  return false;
}

return true;
}
/*!
 * TAKE OFF FOR M100
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(6))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
    current_gps.altitude - home_altitude < 0.5)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}


// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

double bound_yaw(double yaw)
{
  while(yaw > AS_PI) {
    yaw = yaw - 2.0*AS_PI;
  }
  while(yaw < -AS_PI) {
    yaw = yaw + 2.0*AS_PI;
  }
  return yaw;
}

void set_velocities_zero(){
  x_velocity = 0.0;
  y_velocity = 0.0;
  z_velocity = 0.0;
  yaw_rate = 0.0;
  for (int i=0; i<4; i++){
    velocity_final[i] = 0;
  }
}

double PD_Control_xyz(double k_p, double k_d, double pos_current, double pos_desired, double vel_current, double vel_desired)
{
  double v_control;
  v_control = k_p*(pos_desired - pos_current) + k_d*(vel_desired - vel_current);

  //cout <<"pos_d: " <<pos_desired << " pos_c: " <<pos_current << " vel_d: " <<vel_desired << " vel_c: " <<vel_current << endl;
  return v_control;
}

double PD_Control_yaw(double k_p, double k_d, double current_yaw, double desired_yaw, double current_v, double desired_v)
{
  double difference = bound_yaw(desired_yaw - current_yaw);
  return (k_p*difference + k_d*(desired_v - current_v));
}

double control_height(double height_desire, double height_cur)
{
  if (height_cur > height_desire + 0.1) {
    return -0.10;
  } else if (height_cur < height_desire - 0.1) {
    return 0.10;
  } else {
    return 0;
  }
}

void control_callback(const std_msgs::Char& a){
  printf("control = %d, data = %c\n", control, a.data);
  if (a.data == 'o')
  {
    obtain_control();
    obtained = true;
  }
}

void LIMIT_VELOCITY_FOR_SAFETY()
{
  // Enter the limits of the velocities for the drone
  double x_limit = 0.25;
  double y_limit = 0.25;
  double z_limit = 0.25;
  double yaw_limit = 25*AS_PI/180;

  if (x_velocity >= x_limit){
    x_velocity = x_limit;
  }
  else if (x_velocity <= -x_limit) {
    x_velocity = -x_limit;
  }

  if (y_velocity >= y_limit) {
    y_velocity = y_limit;
  }
  else if (y_velocity <= -y_limit) {
    y_velocity = -y_limit;
  }

  if (z_velocity >= z_limit) {
    z_velocity = z_limit;
  }
  else if (z_velocity <= -z_limit) {
    z_velocity = -z_limit;
  }

  if (yaw_rate >= yaw_limit) {
    yaw_rate = yaw_limit;
  }
  else if (yaw_rate <= -yaw_limit) {
    yaw_rate = -yaw_limit;
  }
}

void reach_goal(const geometry_msgs::Point& msg){
  dist_to_wall = DESIRED_DISTANCE_TO_WALL - msg.y;
  y_v_global = PD_Control_xyz(control_gains_p[1], control_gains_d[1], 0, msg.y, 0, 0);
  yaw_change = msg.z;
  yaw_rate = PD_Control_yaw(control_gains_p[3], control_gains_d[3], 0, msg.z, 0,0);
}

void height_reach_goal(const sensor_msgs::LaserScan& msg){
  height_cur = msg.ranges[0];
  inten = msg.intensities[0];
  if (inten == 0) {
    z_velocity_global = z_velocity_global;
  } else {
    z_velocity_global = control_height(HEIGHT_DESIRE+HEIGHT_STEP*height_level,height_cur);
    z_change=HEIGHT_DESIRE+HEIGHT_STEP*height_level-height_cur;
  }
}

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
 sensor_msgs::NavSatFix& target,
 sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}



int main(int argc, char** argv)
{
  // UAV
  ros::init(argc, argv, "control_node_single_plane_following_with_height");

  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber signal = nh.subscribe("control_signal",10,&control_callback);
  // UAV
  ros::Subscriber drone_target_pose = nh.subscribe("/plane_offset/pose" ,1, &reach_goal);
  ros::Subscriber sub = nh.subscribe("/guidance/ultrasonic" ,1, &height_reach_goal);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlPosYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  posData.open("/home/cerlab/catkin_ws/src/configuration.txt");

  string posInfoNo;
  getline(posData, posInfoNo);

  string posInfo;

  getline(posData, posInfo);


  istringstream iss(posInfo);

  vector<string> ReadData((istream_iterator<string>(iss)), istream_iterator<string>());

  DESIRED_DISTANCE_TO_WALL = stof(ReadData[0]);
  HEIGHT_DESIRE = stof(ReadData[1]);
  MAX_HORIZONTAL_MOVEMENT = stof(ReadData[2]);
  HEIGHT_STEP = stof(ReadData[3]);
  MAX_HEIGHT_LEVEL = stoi(ReadData[4]);

  cout << MAX_HORIZONTAL_MOVEMENT << "  " << HEIGHT_DESIRE << "   " << HEIGHT_STEP << "   " << MAX_HEIGHT_LEVEL << "   " << endl;

  bool takeoff_result;

  while(ros::ok()){
    if (obtained)
    {
      break;
    }
    ROS_INFO("Please press o to take control!");
    usleep(20000);
    ros::spinOnce();
  }

  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if (!takeoff_result)
  {
    return 1;
  }
  // BEFORE ACTUAL TASK --> CHECK DISTANCE TO WALL
  while(ros::ok())
  {
    z_velocity = z_velocity_global;
    x_velocity = -y_v_global;

    if (yaw_rate > 0.2)
    {
        x_velocity = 0;
    }

    sensor_msgs::Joy controlVelYawRate;

    LIMIT_VELOCITY_FOR_SAFETY();

    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
      DJISDK::HORIZONTAL_VELOCITY |
      DJISDK::YAW_RATE            |
      DJISDK::HORIZONTAL_BODY   |
      DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(x_velocity); // to the wall
    controlVelYawRate.axes.push_back(0); // along the wall
    controlVelYawRate.axes.push_back(z_velocity); // upward
    controlVelYawRate.axes.push_back(-yaw_rate); // yaw rate
    controlVelYawRate.axes.push_back(flag);

    ctrlPosYawRatePub.publish(controlVelYawRate);

    ROS_INFO("adjusting! vx: %f, vz: %f, yawrate: %f,", x_velocity, z_velocity, yaw_rate);
    //_________________tiecheng & yi________________________
    if (yaw_change < 0.2 && z_change < 0.1 && x_velocity < 0.15)
    {
      no_wall_counter++;
      ROS_INFO("Distance to wall is %f", dist_to_wall);
      ROS_INFO("no_wall_counter is %d --> 60", no_wall_counter);
      // add distance to wall counter properly
    } else {
      no_wall_counter = 0;
    }

    if (no_wall_counter > 60)
    {
        ROS_INFO("drone has maintained its position, now begin its task!");
        break;
    }
    //_________________tiecheng & yi________________________
    usleep(20000);
    ros::spinOnce();
  }

  sensor_msgs::NavSatFix start_gps_location = current_gps;
  geometry_msgs::Vector3     localOffset;

  bool directionChanged = false;

  while(ros::ok()){

    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

    double offsetDistance =  sqrt(pow(localOffset.x, 2) + pow(localOffset.y, 2));

    sensor_msgs::Joy controlVelYawRate;

    x_velocity = -y_v_global;
    y_velocity = 0.25*pow(-1,height_level);
    z_velocity = z_velocity_global;

    if (abs(x_velocity) > 0.25 || abs(yaw_change) > 0.3 || abs(z_change) > 0.25)
    { //dd
        y_velocity = 0;
        if (abs(yaw_change) > 0.3)
        {
            x_velocity = 0;
        }
        ROS_INFO("UAV is trying to adjust its position! Distance : %f", offsetDistance);
    }
    else
    {
        ROS_INFO("D: %f --> %f,current height level: %d --> %d",offsetDistance, MAX_HORIZONTAL_MOVEMENT, height_level, MAX_HEIGHT_LEVEL);
    }
    // Check if task is finished
    if (no_wall_at_top_counter > NO_WALL_TOP_THRESHOLD || (abs(offsetDistance) > MAX_HORIZONTAL_MOVEMENT && height_level == MAX_HEIGHT_LEVEL))
    {
      ROS_INFO("Wall inspection task is accomplished, please land the UAV!");
      while(ros::ok())
      {
        uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_RATE            |
          DJISDK::HORIZONTAL_BODY   |
          DJISDK::STABLE_ENABLE);
          controlVelYawRate.axes.push_back(0); //x
          controlVelYawRate.axes.push_back(0); //y
          controlVelYawRate.axes.push_back(-0.15); //z
          controlVelYawRate.axes.push_back(0); //yaw rate
          controlVelYawRate.axes.push_back(flag);

        ctrlPosYawRatePub.publish(controlVelYawRate);
        if (height_cur < 0.8)
        {
          ROS_INFO("ready to land");
          break;
        }
        ROS_INFO("drone is landing! current height : %f", height_cur);
        usleep(20000);
        ros::spinOnce();
      }
      if(takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
      {
        ROS_INFO("Successful landing!");
        return 1;
      }
      else
      {
        ROS_INFO("land Failed!");
        while(1){
               uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY   |
                DJISDK::STABLE_ENABLE);
                controlVelYawRate.axes.push_back(0); //x
                controlVelYawRate.axes.push_back(0); //y
                controlVelYawRate.axes.push_back(0); //z
                controlVelYawRate.axes.push_back(0); //yaw rate
                controlVelYawRate.axes.push_back(flag);

                ctrlPosYawRatePub.publish(controlVelYawRate);
        }
      }
    }
    // FINISHED ONE MOVEMENT
    // Change state either when no wall is detected (truncate algorithm) or horizontal movement is reached.
    if (!directionChanged && (dist_to_wall > 5  || abs(offsetDistance) > MAX_HORIZONTAL_MOVEMENT)) // may need to add counter
    {
      height_level ++;
      directionChanged = true;
      start_gps_location = current_gps;
      ROS_INFO("UAV move up and change direction!");
      no_wall_at_top_counter = 0; // reset the no_Wall counter at top
    }
    if (dist_to_wall > 5) {
      no_wall_counter++;
    }

    // CHANGE THE DIRECTION BACK
    if (abs(y_velocity) == 0.25 && abs(z_change) < 0.1)
    {
      directionChanged = false;
    }

    LIMIT_VELOCITY_FOR_SAFETY();

    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY     |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(x_velocity); //x
    controlVelYawRate.axes.push_back(y_velocity); //y
    controlVelYawRate.axes.push_back(z_velocity); //z
    controlVelYawRate.axes.push_back(-yaw_rate); //yaw rate
    controlVelYawRate.axes.push_back(flag);

    ctrlPosYawRatePub.publish(controlVelYawRate);
    usleep(20000);
     ros::spinOnce();
    }


  ros::spin();
  return 0;
}
