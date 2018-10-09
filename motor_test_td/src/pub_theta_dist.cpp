#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h" //theta, distance
#include "std_msgs/String.h"            //done

bool flag=1;                            //flag

//done subscribe
void d_msgCallback(const std_msgs::String::ConstPtr& if_done)
{
 ROS_INFO("recieved : %s",if_done->data.c_str());
 flag=1;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"pub_theta_dist");
  ros::NodeHandle nh;

  ros::Publisher td_pub_ij=nh.advertise<std_msgs::Float64MultiArray>("motor_theta_dist",0);
  ros::Subscriber td_sub_it=nh.subscribe("angle_control_done",0,d_msgCallback);
  
  float num1, num2;
  while(ros::ok())
  { 
    
    std_msgs::Float64MultiArray msgin;
//    if(flag==1)
    {
    	std::cout << "Send Motor1 (rpm), Motor2 (rpm) [Ex.30 10] : ";
    	std::cin >> num1 >> num2;
        msgin.data.push_back(num1);
        msgin.data.push_back(num2);
        ROS_INFO("send Motor1 = %f, Motor2 = %f",num1,num2);
        td_pub_ij.publish(msgin);
        msgin.data.clear();
       // flag=0;
    }
    ros::spinOnce();
  } 
  return 0;

}
