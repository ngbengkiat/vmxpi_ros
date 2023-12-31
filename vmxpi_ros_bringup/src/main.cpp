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

    ros::Publisher pub_debug = nh.advertise<std_msgs::Float32>("debug", 1);
   OmniDrive m_omnidrive(&nh);

    ros::Rate loop_rate(50);

    double t = 0;
    while (ros::ok()) {
       
        // m_omnidrive.Move(1, 1);
        // m_omnidrive.Move(1, -1);
        // m_omnidrive.Move(0, 1);
        // m_omnidrive.Move(0, -1);
        m_omnidrive.Move(2, 3.141592653589793/2);
    }
   return 0;
}
