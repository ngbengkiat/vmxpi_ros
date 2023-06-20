#include "SpeedProfile.h"
#include "math.h"
#include <ros/ros.h>

SpeedProfile::SpeedProfile(double dist):SpeedProfile(dist, 0, 0) {
    
}

SpeedProfile::SpeedProfile(double dist, double startSpeed, double endSpeed):SpeedProfile(dist, startSpeed, endSpeed, 0.5, 0.5) {
    
}

SpeedProfile::SpeedProfile(double dist, double startSpeed, double tgtSpeed, double maxSpeed, double maxAcc) {
    // d0 - distance to accelerate from 0 speed to startSpeed;
    // d1 - distance to accelerate from start speed to maxSpeed
    // d2 - constant top speed distance
    // d3 - distance to decelerate from maxSpeed to endSpeed
    // d4 - distance tpo decelerate from endSpeed to 0 speed
    if (dist<0) {
        dist = -dist;
        m_dirSgn = -1;
    }
    else {
        m_dirSgn = 1;
    }
    m_tgtDist = dist;
    m_maxSpeed = maxSpeed;
    m_maxAcc = maxAcc;
    m_startSpeed = startSpeed;
    m_tgtSpeed = tgtSpeed;
    m_curSpeed = startSpeed;
    m_curDist = 0;

    // start speed may not be zero
    double t0 = startSpeed/maxAcc;
    double d0 = startSpeed*t0/2.0;

    // target speed may not be zero
    double t4 = tgtSpeed/maxAcc;
    double d4 = tgtSpeed*t0/2.0;


    double d_fullTrapezoid = m_tgtDist + d0 + d4; //dist for start speed and target speed equal zero
    double t_fullAcc = maxSpeed/maxAcc;     //Time for full acceleration from zero speed to max speed
    double maxSpeedDist =  d_fullTrapezoid - t_fullAcc*t_fullAcc*maxAcc;  //max speed distance

    //Speed profile may not have enought distance to reach max speed
    if (maxSpeedDist < 0) {
        t_fullAcc = sqrt(d_fullTrapezoid/m_maxAcc); //Profile is traingular. Time to reach triangle top.
        maxSpeedDist = 0; //No max speed distance
    }

    m_time_to_acc = t_fullAcc - t0;
    m_time_to_dec = m_time_to_acc + maxSpeedDist/m_maxSpeed;
    m_time_to_tgt = m_time_to_dec + t_fullAcc - t4;

    m_endFlag = false;

    ROS_INFO("maxSpeedDist=%f", maxSpeedDist);
    ROS_INFO("m_time_to_acc=%f", m_time_to_acc);
    ROS_INFO("m_time_to_dec=%f", m_time_to_dec);
    ROS_INFO("m_time_to_tgt=%f", m_time_to_tgt);
}


double SpeedProfile::calculate(double t) {
    if (t < m_time_to_acc) {  //Accelerating
        m_curSpeed = m_startSpeed + t * m_maxAcc;  //Inc speed
        m_curDist = (m_startSpeed + t * m_maxAcc / 2.0) * t;
    } 
    else if (t < m_time_to_dec) { // top speed
        m_curSpeed = m_maxSpeed;
        m_curDist  = (m_startSpeed + m_time_to_acc * m_maxAcc / 2.0) * m_time_to_acc +
                        m_maxSpeed * (t - m_time_to_acc);
    } 
    else if (t < m_time_to_tgt) {  //decelerating
        double timeLeft = m_time_to_tgt - t;
        m_curSpeed = m_tgtSpeed + timeLeft * m_maxAcc;  //Inc speed
        m_curDist  = m_tgtDist - (m_tgtSpeed + timeLeft * m_maxAcc / 2.0) * timeLeft;
    } else 
    {
        m_curSpeed = m_tgtSpeed;
        m_curDist  = m_tgtDist;
        m_endFlag = true;
    }
    return m_curSpeed*m_dirSgn;
}

bool SpeedProfile::IsFinished(double t ) { 
    //ROS_INFO("t=%f m_time_to_tgt=%f m_time_to_acc=%f m_tgtDist=%f", t, m_time_to_tgt, m_time_to_acc, m_tgtDist);
    return m_endFlag; 
    
}

