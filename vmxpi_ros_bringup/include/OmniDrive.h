
#pragma once

#include <memory>
#include <ros/ros.h>
#include <stdint.h>
#include <thread>
#include "TitanDriver_ros.h"
#include <unistd.h>
#include <sys/syscall.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <vmxpi_ros/TitanInfo.h>
#include <vmxpi_ros/UniqueID.h>
#include <vmxpi_ros/Int.h>
#include <vmxpi_ros/StopMode.h>
#include <vmxpi_ros/LimitSwitch.h>
#include <vmxpi_ros/MotorSpeed.h>

class PID {

    private:
        double m_kP, m_kI, m_kD;
        double m_dT;
        double m_measurement, m_setpoint, m_output, m_error, m_error2, m_dError;
        double m_errorSum;
        bool m_continuous;
        double m_minimumInput, m_maximumInput, m_inputRange;

    public:
        PID();
        void enableContinuousInput(double minimumInput, double maximumInput);
        double getContinuousError(double error);
        double calculate(double measurement, double setpoint);
        void setGains(double P, double I, double D, double T);
        double getError() { return m_error; }

};

class OmniDrive {
    
    private:
        double pidIn[3];
        double pidOut[3];
        double feedback[3];
        double encoderDist[3], encoderDist_2[3], encoderCnt[3];
        double encoderSpeed[3];
        double wheelSpeed[3];
        double wheelPos[3];
        double robotSpeed[3];
        double curHeading, targetHeading;
        double motorOut[3];
        PID m_pidController[3];
        const double pid_freq = 50;
        const double pid_dT = 1/pid_freq;
        std::thread runth;
        bool m_pidFlag = true;
        bool m_newMeasurementFlag = false;
        
        ros::Subscriber sub_enc0_dist, sub_enc1_dist, sub_enc2_dist, sub_angle, sub_yawAngle;
        ros::Subscriber sub_enc0_cnt, sub_enc1_cnt, sub_enc2_cnt;
        ros::ServiceClient srv_motor_speed, srv_resetNavx, srv_reset_encoder;
        ros::Publisher pub_pidInput, pub_pidOutput, pub_motorOut, pub_error, pub_fb;

        public:
        OmniDrive();
        OmniDrive(ros::NodeHandle *nh);
        void Run(); // Run loop on separate thread to disable/enable Titan
        void DoRun(); // Run loop on separate thread to disable/enable Titan
        void encoderCntCallback(const std_msgs::Int32::ConstPtr& msg, int enc);
        void encoderDistCallback(const std_msgs::Float32::ConstPtr& msg, int enc);
        void yawCallback(const std_msgs::Float32::ConstPtr& msg);

        void Init(ros::NodeHandle *nh);
        void UpdateOdometry();
        void CalWheeSpeeds(double x, double y);
        void DoPID();
        void DoPID2();
        void DoWork();
        void SetMotorOut012(double s0, double s1, double s2);
        void LimitMotorOut();
        void SetRobotSpeedType(int type, double s);
        void SetRobotSpeedxyw(double x, double y, double w);
        void SetRobotSpeed_open(double x, double y, double w);
        double* GetMotorOuts();
        void publish_motors();
        void publish_data();
        void stop_motors();
        void reset();
        // void Move(int type, double dist);
        // void Move(int type, double dist, double startSpeed, double endSpeed);
        // void Move(int type, double dist, double startSpeed, double endSpeed, double maxSpeed, double maxAcc);
        void Move(int type, double dist, double startSpeed=0, double endSpeed=0, double maxSpeed=0.4, double maxAcc=0.5);

};
