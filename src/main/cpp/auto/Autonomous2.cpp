#include <auto/Autonomous2.h>

Autonomous2::Autonomous2(Drivetrain * drive, DriveTrajectory * traj, Waypoints2 * waypoint)
{
    m_positionChooser = new frc::SendableChooser<int>();
    m_targetChooser = new frc::SendableChooser<int>();
    m_drive = drive;
    m_traj = traj;
    m_waypoints = waypoint;
}

Autonomous2::~Autonomous2()
{
    delete m_positionChooser;
    delete m_targetChooser;
    delete m_drive;
    delete m_traj;
    delete m_waypoints;
}

void Autonomous2::AutoInit()
{
    ReadStation();
    m_timer.Reset();
    movement = 0;
    autodone = false;
}

void Autonomous2::ReadStation()
{
    int targetChoice = m_targetChooser->GetSelected();
    int positionChoice = m_positionChooser->GetSelected();

    if(targetChoice == 1)
    	targetEnum = ThreeEdge;
    else if(targetChoice == 2)
    	targetEnum = SixTwo;
    else if(targetChoice == 3)
        targetEnum = ThreeFive;
    else if(targetChoice == 4)
    	targetEnum = ElevenEnd;
    else if(targetChoice == 5)
        targetEnum = Test;

    if(positionChoice == 1)
    {
        positionEnum = Loader;
        m_drive->SetPose(m_waypoints->kLoaderStartTraj);
    }
    else if(positionChoice == 2)
    {
        positionEnum = Center;
        m_drive->SetPose(m_waypoints->kCenterStartTraj);
    }   
    else if(positionChoice == 3)
    { 
        positionEnum = PowerPort;
        m_drive->SetPose(m_waypoints->kPowerPortStartTraj);
    }
    else if(positionChoice == 4)
    {
        positionEnum = RightTrench;
        m_drive->SetPose(m_waypoints->kRightTrenchStartTraj);
    }
    else if(positionChoice == 5)
    {
        positionEnum = TestP;
        m_drive->SetPose(m_waypoints->testStartPose);
    }
    else
    {
        m_drive->SetPose(m_waypoints->kLeftTrenchStartTraj);
    }
    
}

void Autonomous2::AutoPeriodic()
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
                debugCons("\ntraj config set");
                break;
            case TrajFol:
                if(m_timer.Get()==0_s)
                {
                    m_timeLimit = currentTraj.TotalTime();
                    m_timer.Start();
                    debugCons("\nstart traj follow");
                }
                if(m_timer.Get() < m_timeLimit)
                {
                    m_traj->followRamsete(m_timer.Get(),currentTraj);
                }
                else
                {
                    m_drive->StopMotor();
                    m_timer.Stop();
                    m_timer.Reset();
                    movement++;
                    debugCons("\nend traj follow");
                }
                break;
            case TrajCubicGen:
                currentTraj = m_traj->generateClampedTraj(  autocommand[positionEnum][targetEnum][movement].start,
                                                            autocommand[positionEnum][targetEnum][movement].end,
                                                            autocommand[positionEnum][targetEnum][movement].cubicpoints);
                movement++;
                debugCons("\ntraj cubic generate");
                break;
            case TrajHermiteGen:
                currentTraj = m_traj->generateHermiteTraj(autocommand[positionEnum][targetEnum][movement].hermitepoints);
                movement++;
                debugCons("\ntraj hermite generate");
                break;
            case Stop:
                if(m_timer.Get()==0_s)
                {
                    m_timer.Start();
                    debugCons("\nstart stopping");
                }
                if(m_timer.Get()<autocommand[positionEnum][targetEnum][movement].stoptime)
                    m_drive->StopMotor();
                else
                {
                    debugCons("\nstopping complete");
                    m_timer.Stop();
                    m_timer.Reset();
                    movement++;
                }
                break;
            case Done:
                debugCons("\nauto routine done");
                autodone = true;
                break;
        }
    }
}

void Autonomous2::AddOptions()
{
    int LeftTrenchOpt = 0;
    int LoaderOpt = 1;
    int CenterOpt = 2;
    int PowerportOpt = 3;
    int RightTrenchOpt = 4;
    int TestPOpt = 5;
    m_positionChooser->SetDefaultOption("Left side trench",LeftTrenchOpt);
    m_positionChooser->AddOption("loader, opponent",LoaderOpt);
    m_positionChooser->AddOption("Center",CenterOpt);
    m_positionChooser->AddOption("power port, alliance",PowerportOpt);
    m_positionChooser->AddOption("Right side trench",RightTrenchOpt);
    m_positionChooser->AddOption("testing",TestPOpt);
    frc::SmartDashboard::PutData("Starting Position (Robot side, human view, left to right)",m_positionChooser);

    int CrossLineOpt = 0;
    int ThreeEdgeOpt = 1;
    int SixTwoOpt = 2;
    int ThreeFiveOpt = 3;
    int ElevenEndOpt = 4;
    int TestOpt = 5;
    m_targetChooser->SetDefaultOption("Cross line",CrossLineOpt);
    m_targetChooser->AddOption("Shoot Three & Get to edge",ThreeEdgeOpt);
    m_targetChooser->AddOption("Shoot Six & Hold Two",SixTwoOpt);
    m_targetChooser->AddOption("Shoot Three & Hold Five",ThreeFiveOpt);
    m_targetChooser->AddOption("Shoot Eleven",ElevenEndOpt);
    m_targetChooser->AddOption("testing",TestOpt);
    frc::SmartDashboard::PutData("Auto Task Goals",m_targetChooser);
}

void Autonomous2::SetupAutoCommands()
{
    autocommand[TestP][Test][M0].command = TrajCubicGen;                                  //0 generate traj
    autocommand[TestP][Test][M0].start = m_drive->GetPose();
    autocommand[TestP][Test][M0].end = m_waypoints->testEndPose;
    autocommand[TestP][Test][M0].cubicpoints = m_waypoints->emptyVector;
    autocommand[TestP][Test][M1].command = TrajFol;                                       //1 follow traj
    autocommand[TestP][Test][M2].command = Stop;                                          //2 stop
    autocommand[TestP][Test][M2].stoptime = 3_s;
    autocommand[TestP][Test][M3].command = Done;                                          //6 end auto
}