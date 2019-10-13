#ifndef ARMNODE_H
#define ARMNODE_H

#include "arm.hpp"
#include "ros/ros.h"
#include "RoboticArm/controls.h"
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"

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
    

private:
   ros::Subscriber ArmPosSubscriber;      //Last moment robotic arm pos
   ros::Subscriber ArmSetPointSubscriber; //From perception
   ros::Publisher  ArmControlPubscriber;
   void GetArmPosCallBack(const RoboticArm::state::ConstPtr& msg);
   void GetSetPointCallBack(const RoboticArm::setpoint::ConstPtr& msg);
   float SetPointAttitude[3]; //Roll Pitch Yaw
   float SetPointPosition[3]; //X Y Z


};

}




#endif


