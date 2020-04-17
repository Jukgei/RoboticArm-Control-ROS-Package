#include <iostream>
#include "../include/RoboticArm/ArmNode.hpp"
#include <ros/ros.h>
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"
#include "RoboticArm/controls.h"

const float RoboticArm::RoboticArmNode::L[6]={0,0.05,0.04,0.1,0.1,0.22};

int main( int argc, char **argv ){
    ros::init(argc, argv, "RoboticArm");
    ros::NodeHandle n;
    RoboticArm::RoboticArmNode * myArmNode = new RoboticArm::RoboticArmNode(n);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    delete myArmNode;
    myArmNode = nullptr;
    
    return 0;

}
