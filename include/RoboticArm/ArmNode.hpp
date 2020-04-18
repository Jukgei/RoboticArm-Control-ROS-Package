#ifndef ARMNODE_H
#define ARMNODE_H

#include <vector>
#include "arm.hpp"
#include <fstream>
#include "ros/ros.h"
#include "RoboticArm/controls.h"
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"

#define PI 3.1415926

namespace RoboticArm{

class RoboticArmNode{
public:
   RoboticArmNode(ros::NodeHandle &n); 
   auxiliary::arm myArm[6];
   void InitSubcribers(ros::NodeHandle &n);
   void InitPublishers(ros::NodeHandle &n);
   void InitArmControlThread();
   void RoboticArmControlThread();  //run robotic arm control and planning algorithm
   void Publish();
   bool SetActionTime(float actionTime);
   bool SetDt(float deltaTime); 

private:
   ros::Subscriber ArmPosSubscriber;      //Last moment robotic arm pos
   ros::Subscriber ArmSetPointSubscriber; //From perception
   ros::Publisher  ArmControlPublisher;
   void GetArmPosCallBack(const RoboticArm::state::ConstPtr& msg);
   void GetSetPointCallBack(const RoboticArm::setpoint::ConstPtr& msg);
   float SetPointAttitude[3]; //Roll Pitch Yaw   front not used  1: yaw 2: Roll
   float SetPointPosition[3]; //X Y Z front not used .    1: target h; 2: target l
    
   bool Ikinematics();
   std::vector<float>  GetParam( float qEnd, float qStart, float vEnd, float aEnd, float vStart, float aStart);
   float Planning(std::vector<float> &param, float t);
   void TrajPlan();

   uint16_t CoderAngle(float angle, int ID);
   float DecoderAngle(uint16_t angle, int ID);
   std::vector<float> armState = std::vector<float>(6,0.0f);
   std::vector<float> armDesire = std::vector<float>(6,0.0f);
   static const float L[];
   std::vector<std::vector<float>> qTraj;
   float actionTime;
   float dt;
   int trajSize;  
   int publishFrequency;
   bool TrajUpdate;
};

}

//const float RoboticArm::RoboticArmNode::L[6] = {0,11,12,13,14,17};


#endif

