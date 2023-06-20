#pragma once

class SpeedProfile {
    private:
        double m_maxSpeed, m_maxAcc;
        double m_startSpeed, m_tgtSpeed;
        double m_tgtDist, m_curDist, m_curSpeed, m_curTime;
        double m_time_to_acc, m_time_to_dec, m_time_to_tgt;
        bool   m_endFlag;
        int    m_dirSgn;

    public:
        SpeedProfile(double dist, double startSpeed, double endSpeed, double maxSpeed, double maxAcc);
        SpeedProfile(double dist, double startSpeed, double endSpeed);
        SpeedProfile(double dist);
        double calculate(double time);
        double getSpeed() { return m_curSpeed; }
        bool IsFinished(double t ) ;
};