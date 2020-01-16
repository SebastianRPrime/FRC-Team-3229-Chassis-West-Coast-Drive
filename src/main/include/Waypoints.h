/*
Terms:
General - general waypoint for a part of game field
Left - left side of field
Right - right side of field               left      right
Back - furthest away from driver station \    |   |     /
Mid - 2nd closest to driver station       |   |   |    |
Front - close to driver station          /    |   |     \
Facing - directly facing driver station        - -
Intake - Intake area for hatch/ball      --            --
*/
/*

Example points (not actual location)

*/
#include <frc/geometry/Pose2d.h>
#include <vector>
#include <algorithm>
#pragma once
class Waypoints
{
    public:
        //manipulating waypoints and stuff
        //when reversing waypoints is needed
        Waypoints(){};
        std::vector<frc::Translation2d> ReverseWayPoints(std::vector<frc::Translation2d> waypoints)
        {
        std::reverse(waypoints.begin(),waypoints.end());
        return waypoints;
        };
        
        //always start points
        const frc::Pose2d kLeftUpperStartTraj {3_m,0_m,frc::Rotation2d(0_deg)};
        const frc::Pose2d kLeftLowerStartTraj {3_m,1_m,frc::Rotation2d(0_deg)};
        const frc::Pose2d kMidStartTraj  {5_m,1_m,frc::Rotation2d(0_deg)};
        const frc::Pose2d kRightLowerStartTraj{7_m,1_m,frc::Rotation2d(0_deg)};
        const frc::Pose2d kRightUpperStartTraj{7_m,0_m,frc::Rotation2d(0_deg)};
        //waypoints
        //leftRocketWaypoints
        std::vector<frc::Translation2d> RocketLeftFrontWayPoints {m_rocketLeftGeneralWayPoint,
                                                                  m_rocketLeftFrontWayPoint};
        std::vector<frc::Translation2d> RocketLeftMidWayPoints   {m_rocketLeftGeneralWayPoint,
                                                                  m_rocketLeftMidWayPoint};
        std::vector<frc::Translation2d> RocketLeftBackWayPoints  {m_rocketLeftGeneralWayPoint,
                                                                  m_rocketLeftBackWayPoint};
        std::vector<frc::Translation2d> RocketLeftFrontTransitionPoint {m_rocketLeftFrontToHatchPoint};
        //transition left rocket to hatch pick up
        std::vector<frc::Pose2d> LeftStartShipLeftFront    {kLeftLowerStartTraj,
                                                       shipLeftTransitionHatch,
                                                       kLeftFrontShip};
        //end points (could be start points)
            //rocket
        const frc::Pose2d kLeftFrontRocket{1_m,4_m,frc::Rotation2d(40_deg)};
        const frc::Pose2d kLeftMidRocket  {1_m,5_m,frc::Rotation2d(90_deg)};
        const frc::Pose2d kLeftBackRocket {1_m,6_m,frc::Rotation2d(130_deg)};
            //hatch pickup
        const frc::Pose2d kLeftHatchPickUp{1_m,1_m,frc::Rotation2d(180_deg)};
            //cargo ship
        const frc::Pose2d kLeftFrontShip  {2.5_m,4_m,frc::Rotation2d(-90_deg)};
        //Mid field (use when crossing field side)
        const frc::Pose2d midFieldLeftWayPoint   {5_m,3_m,frc::Rotation2d(90_deg)};
        const frc::Pose2d midFieldRightWayPoint  {5_m,3_m,frc::Rotation2d(-90_deg)};
        //pose2d rocket to hatch transition points
        const frc::Pose2d rocketLeftTransitionHatch     {2_m,4_m,frc::Rotation2d(90_deg)};
        //pose2d ship to hatch transition points
        const frc::Pose2d shipLeftTransitionHatch       {1_m,4_m,frc::Rotation2d(-135_deg)};
    private:
        //position for waypoints
        //3-1 = 2, 4-1 = 3
        //Left Rocket Waypoints
        const frc::Translation2d m_rocketLeftGeneralWayPoint   {2_m,2_m};
        const frc::Translation2d m_rocketLeftFrontWayPoint     {2_m,3_m};
        const frc::Translation2d m_rocketLeftMidWayPoint       {2_m,5_m};
        const frc::Translation2d m_rocketLeftBackWayPoint      {2_m,7_m};
           //
        const frc::Translation2d m_rocketLeftFrontToHatchPoint      {1.2_m,3.8_m}; 
        //Right Rocket Waypointsm_
        const frc::Translation2d m_rocketRightGeneralWayPoint  {8_m,3_m};
        const frc::Translation2d m_rocketRightFrontWayPoint    {8_m,3_m};
        const frc::Translation2d m_rocketRightMidWayPoint      {8_m,5_m};
        const frc::Translation2d m_rocketRightBackWayPoint     {8_m,7_m};
        
};