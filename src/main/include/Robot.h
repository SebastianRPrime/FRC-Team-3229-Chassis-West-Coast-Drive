/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#include <string>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <auto/Autonomous2.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::XboxController m_controller{0};
  units::meters_per_second_t m_x1, m_y1;
  units::radians_per_second_t m_x2;
  
  Drivetrain m_drive;
  DriveTrajectory m_trajectory;
  Waypoints2 m_waypoint;
  Autonomous2 m_auto{&m_drive,&m_trajectory,&m_waypoint};
};
