#include "../include/RoboticArm/ArmNode.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <unistd.h>

#include "RoboticArm/controls.h" 
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"


RoboticArm::RoboticArmNode::RoboticArmNode(ros::NodeHandle &n){
   this->InitPublishers(n);

   this->InitSubcribers(n);

   this->InitArmControlThread();

}

void RoboticArm::RoboticArmNode::InitSubcribers(ros::NodeHandle &n){
    ArmPosSubscriber = n.subscribe<RoboticArm::state>
        ("state",10,&RoboticArmNode::GetArmPosCallBack,this);

    ArmSetPointSubscriber = n.subscribe<RoboticArm::setpoint>
        ("setpoint",10,&RoboticArmNode::GetSetPointCallBack,this);
}

void RoboticArm::RoboticArmNode::InitPublishers(ros::NodeHandle &n){
    this->ArmControlPublisher = n.advertise<RoboticArm::controls>("controls",10);
    std::thread pub(std::bind(&RoboticArmNode::Publish,this));

    pub.detach();
}

void RoboticArm::RoboticArmNode::InitArmControlThread(){
    std::thread ArmCtr(std::bind(&RoboticArmNode::RoboticArmControlThread,this));
    ArmCtr.detach();
}

void RoboticArm::RoboticArmNode::RoboticArmControlThread(){
    while(true){
        //Robotic Control Algorithm
        
        //test 
        for(int i = 1; i <= 5; i++){
            this->myArm[i].CtrPos( 500 );
            this->myArm[i].CtrTime( 500 );
        }
        
        usleep(20000);
    }
        
}

void RoboticArm::RoboticArmNode::Publish(){
    RoboticArm::controls ctr;
    ros::Rate LoopRate(50); //50Hz

    while(ros::ok()){
        
        std::vector<uint16_t> CtrPos;
        std::vector<uint16_t> CtrTime;
        CtrPos.push_back(0xFFFF);   //placeholder
        CtrTime.push_back(0xFFFF);  //placeholder
        for(int i = 1; i <= 5; i++ ){
            CtrPos.push_back(this->myArm[i].GetCtrPos());
            CtrTime.push_back(this->myArm[i].GetCtrTime());
        }
        //DEBUG PRINT
        //for(int i = 1; i <= 5; i++){
        //    printf("Arm[%d] Ctr Pos: %d, Ctr Time:%d \n",i,CtrPos[i],CtrTime[i]);
        //}
        ctr.armCtr = CtrPos;
        ctr.timeCtr = CtrTime;
        this->ArmControlPublisher.publish( ctr );
        LoopRate.sleep();
        
    }

}


void RoboticArm::RoboticArmNode::GetArmPosCallBack(const RoboticArm::state::ConstPtr& msg){
    std::vector<uint16_t> pos = msg->arm;
    for(int i = 1; i <= 5; i++)
        this->myArm[i].SetPos( pos[i] );
       
    
    //DEBUG PRINT
    //printf("Height: %f \n", msg->height);
    //for(int i = 1; i <=5; i++)
    //    printf("Arm[%d] Pos: %d\n",i,pos[i] );
}

void RoboticArm::RoboticArmNode::GetSetPointCallBack(const RoboticArm::setpoint::ConstPtr& msg){
    std::vector<float> attitude = msg-> attitude;
    std::vector<float> position = msg-> position;

    for( int i = 0; i < 3; i++ ){
        this->SetPointAttitude[i] = attitude[i];
        this->SetPointPosition[i] = position[i];
    }


}
