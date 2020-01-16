
#include "Drivetrain.h"

Drivetrain::Drivetrain()
{
  m_leftUpper = new rev::CANSparkMax(1,rev::CANSparkMax::MotorType::kBrushless);
  m_leftFront = new rev::CANSparkMax(2,rev::CANSparkMax::MotorType::kBrushless);
  m_leftBack = new rev::CANSparkMax(3,rev::CANSparkMax::MotorType::kBrushless);
  m_rightUpper = new rev::CANSparkMax(4,rev::CANSparkMax::MotorType::kBrushless);
  m_rightFront = new rev::CANSparkMax(5,rev::CANSparkMax::MotorType::kBrushless);
  m_rightBack = new rev::CANSparkMax(6,rev::CANSparkMax::MotorType::kBrushless);
  
  navXGyro = new AHRS(SPI::Port::kMXP);

  m_leftBack->Follow(*m_leftUpper);
  m_leftFront->Follow(*m_leftUpper);

  m_rightBack->Follow(*m_rightUpper);
  m_rightFront->Follow(*m_rightUpper);
  
  m_leftPosition = units::meter_t(m_leftUpper->GetEncoder().GetPosition() * (2*wpi::math::pi*kWheelRadius)/kEncToWheel);
  m_rightPosition = units::meter_t(m_leftUpper->GetEncoder().GetPosition() * (2*wpi::math::pi*kWheelRadius)/kEncToWheel);
  

}

Drivetrain::~Drivetrain()
{
  delete m_leftUpper;
  delete m_leftFront;
  delete m_leftBack;
  delete m_rightUpper;
  delete m_rightFront;
  delete m_rightBack;

  delete navXGyro;
}
//returns angle in radians
frc::Rotation2d Drivetrain::GetAngle() const {
  double robotangle = navXGyro->GetYaw();
  double radianangle = robotangle*wpi::math::pi/180;
  debugDashNum("degree gyro",robotangle);
  debugDashNum("radian gyro",radianangle);
  return units::radian_t(radianangle);
}
//return m_left and m_right wheel speeds
frc::DifferentialDriveWheelSpeeds Drivetrain::GetSpeeds() const {
  return {units::meters_per_second_t(m_leftVelocity),
          units::meters_per_second_t(m_rightVelocity)};
}
//return Pose2d (for Ramsete controller)
frc::Pose2d Drivetrain::GetPose() const
{
  return m_odometry.GetPose();
}
frc::DifferentialDriveKinematics Drivetrain::GetKinematics() const
{
  return m_kinematics;
}
void Drivetrain::SetPose(frc::Pose2d pose)
{
  m_odometry.ResetPosition(pose,GetAngle());
}
void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  m_leftVelocity = (m_leftUpper->GetEncoder().GetVelocity() / kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60;
  m_rightVelocity = (m_rightUpper->GetEncoder().GetVelocity() / kEncToWheel) *2*wpi::math::pi* kWheelRadius / 60;
  
  debugDashNum("leftVelocity",m_leftVelocity);
  debugDashNum("rightVelocity",m_rightVelocity);
  
  const auto leftOutput = m_leftPIDController.Calculate(
      m_leftVelocity, speeds.left.to<double>());
  const auto rightOutput = m_rightPIDController.Calculate(
      m_rightVelocity, speeds.right.to<double>());

  m_rightUpper->Set(rightOutput);
  m_leftUpper->Set(leftOutput);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot) 
{
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() 
{
  m_odometry.Update(GetAngle(), m_leftPosition,m_rightPosition);
}

void Drivetrain::StopMotor()
{
  m_rightUpper->StopMotor();
  m_leftUpper->StopMotor();
}