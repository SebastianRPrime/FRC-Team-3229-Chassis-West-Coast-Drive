#include <auto/Autonomous.h>

Autonomous::Autonomous(Drivetrain * drive, DriveTrajectory * traj, Waypoints * waypoint)
{
    m_positionChooser = new frc::SendableChooser<int>();
    m_targetChooser = new frc::SendableChooser<int>();
    m_drive = drive;
    m_traj = traj;
    m_waypoints = waypoint;
}

Autonomous::~Autonomous()
{
    delete m_positionChooser;
    delete m_targetChooser;
    delete m_drive;
    delete m_traj;
    delete m_waypoints;
}

void Autonomous::AutoInit()
{
    ReadStation();
    m_timer.Reset();
    movement = 0;
    autodone = false;
}

void Autonomous::ReadStation()
{
    int targetChoice = m_targetChooser->GetSelected();
    int positionChoice = m_positionChooser->GetSelected();

    if(targetChoice == 1)
    	targetEnum = LeftShip;
    else if(targetChoice == 2)
    	targetEnum = FrontShip;
    else if(targetChoice == 3)
        targetEnum = RightShip;
    else if(targetChoice == 4)
    	targetEnum = RightRocket;

    if(positionChoice == 1)
    {
        positionEnum = LeftLower;
        m_drive->SetPose(m_waypoints->kLeftLowerStartTraj);
    }
    else if(positionChoice == 2)
    {
        positionEnum = Center;
        m_drive->SetPose(m_waypoints->kMidStartTraj);
    }   
    else if(positionChoice == 3)
    { 
        positionEnum = RightUpper;
        m_drive->SetPose(m_waypoints->kRightUpperStartTraj);
    }
    else if(positionChoice == 4)
    {
        positionEnum = RightLower;
        m_drive->SetPose(m_waypoints->kRightLowerStartTraj);
    }
    else
    {
        m_drive->SetPose(m_waypoints->kLeftUpperStartTraj);
    }
    
}

void Autonomous::AutoPeriodic()
{
    while(!autodone)
    {
        switch(autocommand[positionEnum][targetEnum][movement].command)
        {
            case TrajConfig:
                m_traj->setConfig(  autocommand[positionEnum][targetEnum][movement].startspeed,
                                    autocommand[positionEnum][targetEnum][movement].endspeed,
                                    autocommand[positionEnum][targetEnum][movement].reverse);
                movement++;
                break;
            case TrajFol:
                if(m_timer.Get()==0_s)
                {
                    m_timeLimit = currentTraj.TotalTime();
                    m_timer.Start();
                }
                if(m_timer.Get() < m_timeLimit)
                {
                    m_traj->followRamsete(m_timer.Get(),currentTraj);
                }
                else
                {
                    m_drive->Drive(0_mps,0_rad_per_s);
                    m_timer.Stop();
                    m_timer.Reset();
                    movement++;
                }
                break;
            case TrajCubicGen:
                currentTraj = m_traj->generateClampedTraj(  autocommand[positionEnum][targetEnum][movement].start,
                                                            autocommand[positionEnum][targetEnum][movement].end,
                                                            autocommand[positionEnum][targetEnum][movement].cubicpoints);
                movement++;
                break;
            case TrajHermiteGen:
                currentTraj = m_traj->generateHermiteTraj(autocommand[positionEnum][targetEnum][movement].hermitepoints);
                movement++;
                break;
            case Stop:
                if(m_timer.Get()==0_s)
                    m_timer.Start();
                if(m_timer.Get()<autocommand[positionEnum][targetEnum][movement].stoptime)
                    m_drive->Drive(0_mps,0_rad_per_s);
                else
                {
                    m_timer.Stop();
                    m_timer.Reset();
                    movement++;
                }
                break;
            case Done:
                autodone = true;
                break;
        }
    }
}

void Autonomous::AddOptions()
{
    int leftUpperOpt = 0;
    int leftLowerOpt = 1;
    int centerOpt = 2;
    int rightUpperOpt = 3;
    int rightLowerOpt = 4;
    m_positionChooser->SetDefaultOption("Left 2nd level",leftUpperOpt);
    m_positionChooser->AddOption("Left 1st level",leftLowerOpt);
    m_positionChooser->AddOption("Center",centerOpt);
    m_positionChooser->AddOption("Right 2nd level",rightUpperOpt);
    m_positionChooser->AddOption("Right 1st level",rightLowerOpt);
    frc::SmartDashboard::PutData("Starting Position",m_positionChooser);

    int leftRocketHatchOpt = 0;
    int leftShipCargoOpt = 1;
    int frontCargoHatchOpt = 2;
    int rightShipCargoOpt = 3;
    int rightRocketHatchOpt = 4;
    m_targetChooser->SetDefaultOption("Left Rocket Hatch",leftRocketHatchOpt);
    m_targetChooser->AddOption("Left Ship Cargo",leftShipCargoOpt);
    m_targetChooser->AddOption("Front Ship Hatch",frontCargoHatchOpt);
    m_targetChooser->AddOption("Right Ship Cargo",rightShipCargoOpt);
    m_targetChooser->AddOption("Right Rocket Hatch",rightRocketHatchOpt);
    frc::SmartDashboard::PutData("Targeted Area",m_targetChooser);
}

void Autonomous::SetupAutoCommands()
{
    //testing trajectory
    autocommand[LeftLower][LeftRocket][M0].command = TrajCubicGen;                                  //0 generate traj
    autocommand[LeftLower][LeftRocket][M0].start = m_waypoints->kLeftLowerStartTraj;
    autocommand[LeftLower][LeftRocket][M0].end = m_waypoints->kLeftFrontRocket;
    autocommand[LeftLower][LeftRocket][M0].cubicpoints = m_waypoints->RocketLeftFrontWayPoints;
    autocommand[LeftLower][LeftRocket][M1].command = TrajFol;                                       //1 follow traj
    autocommand[LeftLower][LeftRocket][M2].command = Stop;                                          //2 stop
    autocommand[LeftLower][LeftRocket][M2].stoptime = .5_s;
    autocommand[LeftLower][LeftRocket][M3].command = TrajConfig;                                    //3 change config
    autocommand[LeftLower][LeftRocket][M3].startspeed = 0_mps;
    autocommand[LeftLower][LeftRocket][M3].endspeed = 0_mps;
    autocommand[LeftLower][LeftRocket][M3].reverse = true;
    autocommand[LeftLower][LeftRocket][M4].command = TrajCubicGen;                                  //4 generate traj
    autocommand[LeftLower][LeftRocket][M4].start = m_drive->GetPose();
    autocommand[LeftLower][LeftRocket][M4].end = m_waypoints->rocketLeftTransitionHatch;
    autocommand[LeftLower][LeftRocket][M4].cubicpoints = m_waypoints->RocketLeftFrontTransitionPoint;
    autocommand[LeftLower][LeftRocket][M5].command = TrajFol;                                       //5 follow traj
    autocommand[LeftLower][LeftRocket][M6].command = Done;                                          //6 end auto
}