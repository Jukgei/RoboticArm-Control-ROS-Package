#include "../include/RoboticArm/ArmNode.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <unistd.h>

#include "RoboticArm/controls.h" 
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"


//const float RoboticArm::RoboticArmNode::L = {0,10,11,12,13,23};

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
        Ikinematics();
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

bool RoboticArm::RoboticArmNode::Ikinematics(){
    float h = SetPointPosition[0];
    float l = SetPointPosition[1];
    h = 0.03; l = 0.377;
    
    float yaw = SetPointAttitude[0];
    float H = h - L[1];
    float bigL = l-L[4]-L[5];
    float L2L3 = L[2] + L[3];
    float L2DL3 = L[2]-L[3];
    float delta = (-(L2DL3*L2DL3) + (H*H) + (bigL*bigL) ) * ( (L2L3*L2L3) - (H*H) - (bigL*bigL) ); 
    if( delta < 0)
    {
        std::cout<< "Sorry, can't not find solution."<<std::endl;
        return false;
    }
    else{
        // solve q2
        float rootDelta = sqrt(delta);
        float q2Numerator = 2*L[2]*H + rootDelta;
        float q2Denominator = L[2]*L[2] + 2*L[2]*bigL - L[3]*L[3] + H*H + bigL*bigL;
        float q2 = 2 * atan(q2Numerator/q2Denominator);
        float q3Denominator = -L2DL3*L2DL3 + H * H  + bigL*bigL;
        float q3 = 2 *atan(rootDelta/q3Denominator);
        float q4 = PI/2 + q3 -q2;
        std::cout<<"q2:"<<q2<<"   q3:"<<q3<<"    q4:"<<q4<<std::endl;

    }
    return true; 
}

