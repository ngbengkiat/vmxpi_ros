#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "Cobra_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "Ultrasonic_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include "OmniDrive.h"
#include "SpeedProfile.h"

static double PI = 3.14159265;
OmniDrive g_OmniDrive;


void cmd_velCallback(const geometry_msgs::Twist& msg)
{
    double x = -msg.linear.x*0.45;    //Right side joystick is negative when right.
    double y = msg.linear.y*0.45;     //Right up/dn joystick is positive when up
    double w = msg.angular.z*2;
    g_OmniDrive.SetRobotSpeedxyw(x, y, w);

}

int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "main_node");
  
   ros::AsyncSpinner spinner(4); //Allows callbacks from multiple threads; spins asynchronously using 4 threads
   spinner.start(); //Starts this spinner spinning asynchronously
   
   ros::NodeHandle nh; //Internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //Realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   /**
    * START CODE HERE
    * 
    */
    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    navXROSWrapper navx(&nh, &vmx);
    ROS_INFO("navX driver is now started");

    ros::Subscriber sub_vel_cmd = nh.subscribe("robot/cmd_vel", 1, cmd_velCallback);
    ros::Publisher pub_debug = nh.advertise<std_msgs::Float32>("debug", 1);
    g_OmniDrive.Init(&nh);

    ros::Rate loop_rate(50);

    double t = 0;
    while (ros::ok()) {
       
        loop_rate.sleep();
    }
   return 0;
}
