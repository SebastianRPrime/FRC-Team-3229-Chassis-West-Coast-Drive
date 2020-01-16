#pragma once

#include <DriveTrajectory.h>
#include <Waypoints.h>

#include <frc2/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>


class Autonomous
{
    public:
        Autonomous(Drivetrain * drive, DriveTrajectory * traj, Waypoints * waypoint);
        ~Autonomous();
        void AutoInit();
	    void ReadStation();
	    void AutoPeriodic();
	    void AddOptions();
	    void SetupAutoCommands();

	    bool autodone = false;
    private:
        Drivetrain * m_drive;
        DriveTrajectory * m_traj;
        Waypoints * m_waypoints;

        frc2::Timer m_timer;

        frc::SendableChooser<int> * m_positionChooser;
        frc::SendableChooser<int> * m_targetChooser;
        
        frc::Trajectory currentTraj;
        enum Positions{LeftUpper,LeftLower,Center,RightUpper,RightLower};
        enum Targets{LeftRocket,LeftShip,FrontShip,RightShip,RightRocket};
        enum Movements{M0,M1,M2,M3,M4,M5,M6};
        enum Commands{TrajConfig,TrajCubicGen,TrajHermiteGen,TrajFol,Stop,Done};

        struct cmd
        {
            Commands command;
            //trajConfig 
            units::meters_per_second_t startspeed;
            units::meters_per_second_t endspeed;
            bool reverse;

            //trajCubicGen
            frc::Pose2d start;
            frc::Pose2d end;
            std::vector<frc::Translation2d> cubicpoints;
            //Hermite Gen
            std::vector<frc::Pose2d> hermitepoints;            
            //stop
            units::second_t stoptime;
        };
        cmd autocommand [5][5][7];//positions | targets | movements
        int movement = 0;
        Positions positionEnum = LeftUpper;
        Targets targetEnum = LeftRocket;
        units::time::second_t m_timeLimit;
};