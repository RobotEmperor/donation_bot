/*
 * motor_algorithm.h
 *
 *      Author: robotemperor
 */

//pin information

#define motor1_IN1 26
#define motor1_PWM 12

#define motor1_FG1 6 //22
#define motor1_FG2 5


#define motor2_IN1 19
#define motor2_PWM 13

#define motor2_FG1 22 //23
#define motor2_FG2 27


#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <wiringPi.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

//custom header
#include <robot1/motor_cmd.h>
#include <robot1/motor.h>

class TrajectoryGenerator
{
  public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();
    double linear_function(double desired_value, double acceleration);

  private:
    double pre_desired_value;
    double current_desired_value;
    double out_value;
    double time_count;
    bool   tra_done_check;

};


//ros communication
ros::Publisher  angle_control_done_pub;
ros::Publisher  desired_rpm1_pub;
ros::Publisher  desired_rpm2_pub;

ros::Publisher  result_rpm1_pub;
ros::Publisher  result_rpm2_pub;

ros::Subscriber motor_theta_dist_sub;

//message for communication
std_msgs::String angle_control_done_msg;
std_msgs::Float64  desired_rpm1_msg;
std_msgs::Float64  desired_rpm2_msg;

std_msgs::Float64  result_rpm1_msg;
std_msgs::Float64  result_rpm2_msg;


DcMotorForRaspberryPi* motor1;
DcMotorForRaspberryPi* motor2;

TrajectoryGenerator* tra_motor1;
TrajectoryGenerator* tra_motor2;

double current_desired_speed_motor1;
double current_desired_speed_motor2;

double reference_angle;
double reference_distance;

//function
void initialize();
void algorithm(double angle, double distance);
void motor_control(int id, int motor_line1, int mode, bool direction, int desired_speed_rpm, int angle, bool on_off);

void motor_theta_dist_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

//timer
void controlFunction(const ros::TimerEvent&);






