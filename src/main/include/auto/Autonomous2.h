#pragma once

#include <DriveTrajectory.h>
#include <Waypoints2.h>

#include <frc2/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

class Autonomous2
{
    public:
        Autonomous2(Drivetrain * drive, DriveTrajectory * traj, Waypoints2 * waypoint);
        ~Autonomous2();
        void AutoInit();
	    void ReadStation();
	    void AutoPeriodic();
	    void AddOptions();
	    void SetupAutoCommands();

	    bool autodone = false;
    private:
        Drivetrain * m_drive;
        DriveTrajectory * m_traj;
        Waypoints2 * m_waypoints;

        frc2::Timer m_timer;

        frc::SendableChooser<int> * m_positionChooser;
        frc::SendableChooser<int> * m_targetChooser;
        
        frc::Trajectory currentTraj;
        enum Positions{LeftTrench,Loader,Center,PowerPort,RightTrench,TestP}; //robot side location (left to right)
        enum Targets{CrossLine,ThreeEdge,SixTwo,ThreeFive,ElevenEnd,Test}; 
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
        cmd autocommand [6][6][7];//positions | targets | movements
        int movement = 0;
        Positions positionEnum = LeftTrench;
        Targets targetEnum = CrossLine;
        units::time::second_t m_timeLimit;
};