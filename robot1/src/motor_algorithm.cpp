/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <robot1/motor_algorithm.h>

//////////////////////////////////////////////////////////////////////////////
TrajectoryGenerator::TrajectoryGenerator()
{
  pre_desired_value = 0;
  current_desired_value = 0;
  out_value = 0;
  time_count = 0;
  tra_done_check = 0;

}
TrajectoryGenerator::~TrajectoryGenerator()
{

}
double TrajectoryGenerator::linear_function(double desired_value, double acceleration)
{
  static double time = 0;

  if(current_desired_value != desired_value)
  {
    time_count = 0;
    tra_done_check = false;
    pre_desired_value = out_value;
  }

  time = fabs((pre_desired_value - desired_value) / acceleration);

  current_desired_value = desired_value;

  time_count = time_count + 0.01;

  if(time_count >= time)
  {
    tra_done_check = true;
    pre_desired_value = desired_value;
    return pre_desired_value;
  }

  if(pre_desired_value != desired_value && tra_done_check == false)
  {
    if(pre_desired_value > desired_value) // 하강 트레젝토리 y = -at + b
    {
      out_value = -(acceleration)*time_count + pre_desired_value;
      return out_value;
    }
    if(pre_desired_value < desired_value)// 상승 트레젝토리 y = at + b
    {
      out_value = (acceleration)*time_count + pre_desired_value;
      return out_value;
    }
  }
  else
  {
    return pre_desired_value;
  }

}
//////////////////////////////////////////////////////////////////////////////
void initialize()
{
  reference_angle = 0;
  reference_distance = 0;

  motor1 = new DcMotorForRaspberryPi(399,100,2);
  motor2 = new DcMotorForRaspberryPi(399,100,2);

  tra_motor1 = new TrajectoryGenerator;
  tra_motor2 = new TrajectoryGenerator;

  current_desired_speed_motor1 = 0;
  current_desired_speed_motor2 = 0;

  motor1->check_position = true;  
  motor2->check_position = true;
}
void motor1_encoder_1(void)
{
  motor1->encoder_pulse1 ++;
  motor1->encoder_pulse_position1 ++;
}
void motor1_encoder_2(void)
{
  motor1->encoder_pulse2++;
  motor1->encoder_pulse_position2 ++;
}

void motor2_encoder_1(void)
{
  motor2->encoder_pulse1 ++;
  motor2->encoder_pulse_position1 ++;
}
void motor2_encoder_2(void)
{
  motor2->encoder_pulse2 ++;
  motor2->encoder_pulse_position2 ++;
}
//test
void motor_theta_dist_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
 // reference_angle = msg->data[0];
 // reference_distance = msg->data[1];

  motor1->speed_motor = msg->data[0];
  motor2->speed_motor = msg->data[1];


//  motor1->encoder_pulse_position1 = 0;
//  motor1->encoder_pulse_position2 = 0;
//  motor2->encoder_pulse_position1 = 0;
//  motor2->encoder_pulse_position2 = 0;

  motor1->check_position = false;
  motor2->check_position = false;
}
void algorithm(double angle, double distance)
{
  static int motion_sequence = 1;
  if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 1)
  {
    motion_sequence ++;
    motor1->encoder_pulse_position1 = 0;
    motor1->encoder_pulse_position2 = 0;
    motor2->encoder_pulse_position1 = 0;
    motor2->encoder_pulse_position2 = 0;

    motor1->check_position = false;
    motor2->check_position = false;
    printf("Motion change \n");
  }
  else if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 2)
  {
    motion_sequence ++;
    angle_control_done_msg.data = "done";
    angle_control_done_pub.publish(angle_control_done_msg);

    motor1->angle_motor = 0;
    motor2->angle_motor = 0;

    printf("Motion done! \n");
  }
  else if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 3)
  {
    printf("waiting! \n");
  }
  else if(motor1->check_position == false && motor2->check_position == false && motion_sequence == 3)
  {
    motion_sequence = 1;
    printf("Motion init! \n");
  }
  else
  {
    printf("running! \n");
  }
  switch(motion_sequence)
  {
  case 1 :
  {
    motor1->position_max_rpm = 20;
    motor2->position_max_rpm = 20;
    motor1->angle_motor = (int) fabs(angle*2);////
    motor2->angle_motor = motor1->angle_motor;

    if(angle < 0)
    {
      motor1->direction = false;
      motor2->direction = false;
    }
    else
    {
      motor1->direction = true;
      motor2->direction = true;
    }
    break;
  }
  case 2 :
  {
    motor1->position_max_rpm = 50;
    motor2->position_max_rpm = 50;
    motor1->angle_motor = (int) ((fabs(distance)/0.035)*180)/M_PI; ///
    motor2->angle_motor = motor1->angle_motor;
    if(distance > 0)
    {
      motor1->direction = true;
      motor2->direction = true;
    }
    if(distance < 0)
    {
      motor1->direction = false;
      motor2->direction = false;
    }
    break;
  }
  default :
    break;
  }
}
/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int id, int motor_line1, int mode, bool direction, int desired_speed_rpm, int angle, bool on_off)
{
  if(on_off == true)
  {
    if(direction == true)//CW
    {
      digitalWrite(motor_line1,HIGH);
    }
    else if (direction == false)//CCW
    {
      digitalWrite(motor_line1,LOW);
    }

    switch (id)
    {
    case 1 :
      if(mode == 1)
      {
        desired_speed_rpm = motor1->position_controller(angle, motor1->position_max_rpm);
      }
      current_desired_speed_motor1 = tra_motor1->linear_function(desired_speed_rpm, motor1->acceleration_value);
      desired_speed_rpm = current_desired_speed_motor1;
     
      motor1->speed_controller(desired_speed_rpm);
      break;
    case 2 :
      if(mode == 1)
      {
        desired_speed_rpm = motor2->position_controller(angle, motor2->position_max_rpm);
      }
      current_desired_speed_motor2 = tra_motor2->linear_function(desired_speed_rpm, motor2->acceleration_value);
      desired_speed_rpm = current_desired_speed_motor2;

      motor2->speed_controller(desired_speed_rpm);
      break;
    default :
      break;
    }
  }

  if(on_off == false)
  {
    pwmWrite(motor1_PWM, 0);
    pwmWrite(motor2_PWM, 0);
  }
}

void controlFunction(const ros::TimerEvent&)
{
  motor1->onoff = 1;
  motor2->onoff = 1;

  //algorithm(reference_angle, reference_distance);

  motor_control(1, motor1_IN1, 0,  motor1->direction, motor1->speed_motor, motor1->angle_motor, motor1->onoff);
  motor_control(2, motor2_IN1, 0,  motor2->direction, motor2->speed_motor, motor2->angle_motor, motor2->onoff);

  pwmWrite(motor1_PWM, (int) motor1->pwm_value_motor);
  pwmWrite(motor2_PWM, (int) motor2->pwm_value_motor);

}
int main (int argc, char **argv)
{
  wiringPiSetupGpio();

  initialize();

  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;

  ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), controlFunction); // 10ms

  motor_theta_dist_sub   = nh.subscribe("motor_theta_dist", 1, motor_theta_dist_callback); // test
  angle_control_done_pub = nh.advertise<std_msgs::String>("angle_control_done",10);
  result_rpm1_pub = nh.advertise<std_msgs::Float64>("result_rpm1",10);
  result_rpm2_pub = nh.advertise<std_msgs::Float64>("result_rpm2",10);

  desired_rpm1_pub = nh.advertise<std_msgs::Float64>("desired_rpm1",10);
  desired_rpm2_pub = nh.advertise<std_msgs::Float64>("desired_rpm2",10);

  pinMode(motor1_IN1, OUTPUT);
  pinMode(motor1_FG1, INPUT);
  pinMode(motor1_FG2, INPUT);

  pinMode(motor2_IN1, OUTPUT);
  pinMode(motor2_FG1, INPUT);
  pinMode(motor2_FG2, INPUT);

  wiringPiISR(motor1_FG1, INT_EDGE_RISING, &motor1_encoder_1);
  wiringPiISR(motor1_FG2, INT_EDGE_RISING, &motor1_encoder_2);
  wiringPiISR(motor2_FG1, INT_EDGE_RISING, &motor2_encoder_1);
  wiringPiISR(motor2_FG2, INT_EDGE_RISING, &motor2_encoder_2);

  pinMode(motor1_PWM, PWM_OUTPUT);
  pinMode(motor2_PWM, PWM_OUTPUT);
  pwmSetMode (PWM_MODE_MS);
  pwmSetRange(512);
  pwmSetClock(2);

  digitalWrite(motor1_IN1,HIGH);
  digitalWrite(motor2_IN1,LOW);


  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  while(ros::ok())
  {
    //desired_rpm1_msg.data = current_desried_speed_motor1;
    //desired_rpm2_msg.data = current_desried_speed_motor2;
    //result_rpm1_msg.data = motor1->result_rpm;
    //result_rpm2_msg.data = motor2->result_rpm;
    usleep(100);
    //result_rpm1_pub.publish(result_rpm1_msg);
    //result_rpm2_pub.publish(result_rpm2_msg);
    //desired_rpm1_pub.publish(desired_rpm1_msg);
    //desired_rpm2_pub.publish(desired_rpm2_msg);

    ros::spinOnce();
  }

  delete motor1;
  delete motor2;
  delete tra_motor1;
  delete tra_motor2;
  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  return 0;
}


