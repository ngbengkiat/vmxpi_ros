#include "OmniDrive.h"
#include "math.h"
#include "SpeedProfile.h"

#define PI  3.141592653589793

PID::PID(){
    m_continuous = false;
}
void PID::enableContinuousInput(double minimumInput, double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    m_inputRange = maximumInput - minimumInput;
}

double PID::getContinuousError(double error) {
    if (m_continuous && m_inputRange > 0) {
      error = std::fmod(error, m_inputRange);
      if (std::abs(error) > m_inputRange / 2) {
        if (error > 0) {
          return error - m_inputRange;
        } else {
          return error + m_inputRange;
        }
      }
    }
    return error;
  }
double PID::calculate(double measurement, double setpoint){
    m_setpoint = setpoint;
    m_measurement = measurement;

    m_error = getContinuousError(m_setpoint - measurement);
    m_dError = (m_error - m_error2) / m_dT;
    m_error2 = m_error;

    if (m_kI != 0) {
        m_errorSum = m_errorSum + m_error * m_dT;
        //Limit m_errorSum???
        //m_errorSum = std::clamp(m_errorSum, 01.0 / m_kI, 1.0 / m_kI);
    }

    return m_output = m_kP * m_error + m_kI * m_errorSum + m_kD * m_dError;
};

void PID::setGains(double P, double I, double D, double T){
    m_kP = P;
    m_kI = I;
    m_kD = D;
    m_dT = T;
};
        
// Callbacks for Encoder Distance values
void OmniDrive::encoderDistCallback(const std_msgs::Float32::ConstPtr& msg, int enc)
{
    encoderDist[enc] = msg->data;
    m_newMeasurementFlag = true;
}

// Callbacks for Encoder Count values
void OmniDrive::encoderCntCallback(const std_msgs::Int32::ConstPtr& msg, int enc)
{
    encoderCnt[enc] = msg->data;
}

// Callbacks for Yaw
void OmniDrive::yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
   curHeading = -msg->data*PI/180;
}

OmniDrive::OmniDrive(ros::NodeHandle *nh) {

    srv_motor_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed", true);
    srv_resetNavx = nh->serviceClient<std_srvs::Empty>("reset_navx");
    srv_reset_encoder = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");

    sub_enc0_dist = nh->subscribe<std_msgs::Float32>("titan/encoder0/distance", 3, boost::bind(&OmniDrive::encoderDistCallback, this, _1, 0) );
    sub_enc1_dist = nh->subscribe<std_msgs::Float32>("titan/encoder1/distance", 3, boost::bind(&OmniDrive::encoderDistCallback, this, _1, 1) );
    sub_enc2_dist = nh->subscribe<std_msgs::Float32>("titan/encoder2/distance", 3, boost::bind(&OmniDrive::encoderDistCallback, this, _1, 2) );

    // sub_enc0_cnt = nh->subscribe<std_msgs::Int32>("titan/encoder0/count", 1, boost::bind(&OmniDrive::encoderCntCallback, this, _1, 0) );
    // sub_enc1_cnt = nh->subscribe<std_msgs::Int32>("titan/encoder1/count", 1, boost::bind(&OmniDrive::encoderCntCallback, this, _1, 1) );
    // sub_enc2_cnt = nh->subscribe<std_msgs::Int32>("titan/encoder2/count", 1, boost::bind(&OmniDrive::encoderCntCallback, this, _1, 2) );
    sub_yawAngle = nh->subscribe("navx/yaw", 3, &OmniDrive::yawCallback, this);
    
    pub_pidInput = nh->advertise<std_msgs::Float32>("pidInput", 1);
    pub_pidOutput = nh->advertise<std_msgs::Float32>("pidOutput", 1);
    pub_motorOut = nh->advertise<std_msgs::Float32>("motorOut", 1);
    pub_error = nh->advertise<std_msgs::Float32>("error", 1);
    pub_fb = nh->advertise<std_msgs::Float32>("feedback", 1);

    m_pidController[0].setGains(0.5, 8.0, 0.0, pid_dT);
    m_pidController[1].setGains(0.5, 8.0, 0.0, pid_dT);
    m_pidController[2].setGains(2.0, 0.0, 0.04, pid_dT);
    m_pidController[2].enableContinuousInput(-PI, PI);
    // Start separate run loop thread to prevent blocking sent packets
    runth = std::thread(&OmniDrive::Run, this);

}

void OmniDrive::Run() {
    ROS_INFO_STREAM("OmniDrive thread: " << syscall(SYS_gettid));

    //reset();
    // ros::AsyncSpinner spinner(4); //Allows callbacks from multiple threads; spins asynchronously using 4 threads
    // spinner.start(); //Starts this spinner spinning asynchronously
    ros::Rate loop_rate(pid_freq);
    for (int i=0; i<3; i++) {
        encoderDist_2[i] = encoderDist[i];
    }
    while(ros::ok()) {

        if (m_pidFlag && m_newMeasurementFlag) {
            m_newMeasurementFlag = false;
            DoPID();
        }
        publish_motors();
        publish_data();     //for debug
        ros::spinOnce();

        loop_rate.sleep();
        // ROS_INFO("t1: %f \n", loop_rate.cycleTime().toSec());
        // ros::Duration(0.001).sleep();

    }
}

//For testing purpose
void OmniDrive::DoRun() {


        if (m_pidFlag && m_newMeasurementFlag) {
            m_newMeasurementFlag = false;
            DoPID();
        }
        publish_motors();
        publish_data();     //for debug

}

void OmniDrive::UpdateOdometry(){

}
void OmniDrive::CalWheeSpeeds(double x, double y){

}
void OmniDrive::DoPID2(){
    // pidIn[0] = robotSpeed[0];
    // pidIn[1] = robotSpeed[1];
    // pidIn[2] = robotSpeed[2];

    pidIn[0] = (-0.5*robotSpeed[0] - 0.866025*robotSpeed[1]);
    pidIn[1] = (     robotSpeed[0] + 0               );
    pidIn[2] = (-0.5*robotSpeed[0] + 0.866025*robotSpeed[1]);

    //This is for translational speed PID
    //First calculate wheel speed from encoder feedback
    double dcValue = 0.0;
    for (int i=0; i<3; i++) {
        wheelSpeed[i] = encoderSpeed[i] = (encoderDist[i]-encoderDist_2[i])/pid_dT;
        wheelPos[i] += encoderDist[i]-encoderDist_2[i];
        encoderDist_2[i] = encoderDist[i];
        dcValue += wheelSpeed[i];
    }

    //Subtract rotational component from encoder speed
    //Rotational PID is handled by gyro separately.
    //Maybe good to combine this dc value with gyro value??????
    dcValue /= 3;
    for (int i=0; i<3; i++) {
       wheelSpeed[i] -= dcValue;
    }
    

    //PID control for wheel speed
    //Speed control 
    pidOut[0] = m_pidController[0].calculate(wheelSpeed[0], pidIn[0]);
    pidOut[1] = m_pidController[1].calculate(wheelSpeed[1], pidIn[1]);
    pidOut[2] = m_pidController[2].calculate(wheelSpeed[2], pidIn[2]);

    motorOut[0] = pidOut[0];
    motorOut[1] = pidOut[1];
    motorOut[2] = pidOut[2];

    LimitMotorOut();

}

void OmniDrive::DoPID(){
    pidIn[0] = robotSpeed[0];
    pidIn[1] = robotSpeed[1];
    pidIn[2] = robotSpeed[2];

    /////////////////////////////////////////////////////////////////////////////////////////
    //This is for translational speed PID
    /////////////////////////////////////////////////////////////////////////////////////////
    //First calculate wheel speed from encoder feedback
    double dcValue = 0.0;
    for (int i=0; i<3; i++) {
        wheelSpeed[i] = encoderSpeed[i] = (encoderDist[i]-encoderDist_2[i])/pid_dT;
        encoderDist_2[i] = encoderDist[i];
        dcValue += wheelSpeed[i];
    }

    //Subtract rotational component from encoder speed
    //Rotational PID is handled by gyro separately.
    //Maybe good to combine this dc value with gyro value??????
    dcValue /= 3;
    for (int i=0; i<3; i++) {
        wheelSpeed[i] -= dcValue;
    }
    
    //Estimates x and y speed from individual wheel speeds
    //See formula below
    feedback[0] = (-(wheelSpeed[0] + wheelSpeed[2]) + wheelSpeed[1])/2;
    feedback[1] = (-wheelSpeed[0] + wheelSpeed[2])/(0.866025*2);

    //PID control for x and y speed
    //Speed control 
    pidOut[0] = m_pidController[0].calculate(feedback[0], pidIn[0]);
    pidOut[1] = m_pidController[1].calculate(feedback[1], pidIn[1]);

    /////////////////////////////////////////////////////////////////////////////////////////
    //This is for rotational speed PID
    /////////////////////////////////////////////////////////////////////////////////////////
    targetHeading += pidIn[2]*pid_dT;   

    //Limit targetHeading to -Pi to +Pi
    if (targetHeading>PI) targetHeading -= PI*2;
    if (targetHeading<-PI) targetHeading += PI*2;

    pidOut[2] = m_pidController[2].calculate(curHeading, targetHeading);


    //Translate x and y output to wheel outputs
    // The x and y speed are resolved into individual wheel speed
    // 3 wheel omni drive
    // R is distance of wheel from robot centre
    // M0 = [-sin(150) cos(150) R] * [x y w]'   //Left-front wheel
    // M1 = [-sin(270) cos(270) R]              //Back wheel
    // M2 = [-sin(30)  cos(30)  R]              //Right-front wheel
    motorOut[0] = (-0.5*pidOut[0] - 0.866025*pidOut[1]) + pidOut[2];
    motorOut[1] = (     pidOut[0] + 0                 ) + pidOut[2];
    motorOut[2] = (-0.5*pidOut[0] + 0.866025*pidOut[1]) + pidOut[2];

    LimitMotorOut();

}

void OmniDrive::SetMotorOut012(double s0, double s1, double s2)
{
    motorOut[0] = s0;
    motorOut[1] = s1;
    motorOut[2] = s2;
}
void OmniDrive::SetRobotSpeedType(int type, double s) {
    robotSpeed[type] = s; 
}
void OmniDrive::SetRobotSpeedxyw(double x, double y, double w) {
    robotSpeed[0] = x; 
    robotSpeed[1] = y;
    robotSpeed[2] = w; 
    m_pidFlag = true;
}
void OmniDrive::SetRobotSpeed_open(double x, double y, double w){
    // M0 = [-sin(150) cos(150) R] * [x y w]'   //Left-front wheel
    // M1 = [-sin(270) cos(270) R]              //Back wheel
    // M2 = [-sin(30)  cos(30)  R]              //Right-front wheel
    double robotRadius = 0.099;
    motorOut[0] = (-0.5*x - 0.866025*y + w);
    motorOut[1] = (     x + 0          + w);
    motorOut[2] = (-0.5*x + 0.866025*y + w);

    LimitMotorOut();
    m_pidFlag = false;
}

void OmniDrive::LimitMotorOut() {
    double max=fabs(motorOut[0]);
    for (int i=1; i<3; i++) {
        if (fabs(motorOut[i]) > max)
            max = fabs(motorOut[i]);
    }
    if (max>1.0) {
        for (int i=0; i<3; i++) {
            motorOut[i] /= max;
        }   
    }
}

double* OmniDrive::GetMotorOuts() {
    return motorOut;
}

void OmniDrive::publish_data()
{
    int i = 1;
    std_msgs::Float32 msg;
    msg.data = pidIn[i];
    pub_pidInput.publish(msg);
    msg.data = pidOut[i];
    pub_pidOutput.publish(msg);
    msg.data = robotSpeed[i];
    pub_motorOut.publish(msg);
    msg.data = feedback[i];
    pub_fb.publish(msg);
    msg.data = m_pidController[i].getError();
    pub_error.publish(msg);
}
void OmniDrive::publish_motors()
{
    double t0, t1;
    vmxpi_ros::MotorSpeed msg1;
    t0 = ros::Time::now().toSec();
    msg1.request.speed = -motorOut[0]; //Motor speed id reversed from out convention
    msg1.request.motor = 0;
    srv_motor_speed.call(msg1);

    msg1.request.speed = -motorOut[1];
    msg1.request.motor = 1;
    srv_motor_speed.call(msg1);

    msg1.request.speed = -motorOut[2];
    msg1.request.motor = 2;
    srv_motor_speed.call(msg1);
    t1 = ros::Time::now().toSec();
    ROS_INFO("msg: %f \n", t1-t0);
}

void OmniDrive::stop_motors()
{
    vmxpi_ros::MotorSpeed msg1;

    msg1.request.speed = 0.0;
    msg1.request.motor = 0;
    srv_motor_speed.call(msg1);

    msg1.request.speed = 0.0;
    msg1.request.motor = 1;
    srv_motor_speed.call(msg1);

    msg1.request.speed = 0.0;
    msg1.request.motor = 2;
    srv_motor_speed.call(msg1);
}

void OmniDrive::reset()
{
    stop_motors();
    std_srvs::Trigger msg1;
    srv_reset_encoder.call(msg1); // Resets displacement encoders
    std_srvs::Empty msg2;
    srv_resetNavx.call(msg2); // Resets yaw variable
}

// void OmniDrive::Move(int type, double dist) {
//     Move(type, dist, 0, 0, 0.4, 0.5);
// }
void OmniDrive::Move(int type, double dist, double startSpeed, double endSpeed, double maxSpeed, double maxAcc) {
    double speed ;
    SpeedProfile profile( dist,  startSpeed,  endSpeed,  maxSpeed, maxAcc);
    
    ros::Rate loop_rate(50);
    
    double t = 0;
    while (ros::ok()) {
    
        if (profile.IsFinished(t)) {
            break;
        }
        speed = profile.calculate(t);
        SetRobotSpeedType(type, speed);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("t: %f speed: %f \n", t, speed);
        t += 0.02;

    }
}