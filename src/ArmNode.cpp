#include "../include/RoboticArm/ArmNode.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <unistd.h>

#include "RoboticArm/controls.h" 
#include "RoboticArm/setpoint.h"
#include "RoboticArm/state.h"


RoboticArm::RoboticArmNode::RoboticArmNode(ros::NodeHandle &n){
   
    
   //armDesire{0,0,0,0,0,0};

   actionTime = 0.25;
   dt = 0.005;
   publishFrequency = (int) 1/dt;
   trajSize = (int) (actionTime /dt);
   TrajUpdate = false;
   stateStable = false;

    std::cout<<trajSize<<std::endl;
   
   for(int i = 0; i < 5; i ++)
       qTraj.push_back(std::vector<float>(trajSize,0.0f));

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
    bool first_flag = true;
    while(ros::ok()){
        //Robotic Control Algorithm
        if(stateStable){
            Ikinematics();

            TrajPlan();
            //if(first_flag)
            //{
            //    first_flag = false;
            //    TrajUpdate = true;
            //}
            TrajUpdate = true;
        }
        
        //test 
        //for(int i = 1; i <= 5; i++){
        //    this->myArm[i].CtrPos( 500 );
        //    this->myArm[i].CtrTime( 500 );
        //}
        
        ros::Duration(actionTime).sleep();    
    }
        
}

void RoboticArm::RoboticArmNode::Publish(){
    RoboticArm::controls ctr;
    ros::Rate LoopRate(publishFrequency); //50Hz
    //ros::Rate LoopRate(1); //50Hz
    int index = 0;
    while(ros::ok()){
        
        std::vector<uint16_t> CtrPos;
        std::vector<uint16_t> CtrTime;
        CtrPos.push_back(0xFFFF);   //placeholder
        CtrTime.push_back(0xFFFF);  //placeholder
        if(TrajUpdate){
            for(int i = 1; i <= 5; i++){
                std::vector<float> tmp;
                {
                    std::lock_guard<std::mutex> traj(varLock);
                    tmp = qTraj[i-1];
                }
                this->myArm[i].CtrPos(CoderAngle(tmp[index],i));
                this->myArm[i].CtrTime((uint16_t)dt*1000);
                //std::cout<<"The "<<i<<"th "<<"arm traj size is : "<<tmp.size()<<std::endl;
                //if(i == 1)
                //    std::cout<<"The robotic arm of "<< i <<" joint is "<< CoderAngle(tmp[index],i)<<" "<<"The rad of the value is "<< tmp[index]<<'.' <<"The index is "<< index << ' '<< std::endl;
            } 
            //std::cout<<"send a frame"<<std::endl;
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
            index++;
            if(index >= trajSize){
                std::cout<<"You are over"<<std::endl;
                TrajUpdate = false;
                index = 0;
            }
        }

        LoopRate.sleep();
        
    }

}


void RoboticArm::RoboticArmNode::GetArmPosCallBack(const RoboticArm::state::ConstPtr& msg){
    std::vector<uint16_t> pos = msg->arm;
    stateStable = true;
    for(int i = 1; i <= 5; i++)
    {
        this->myArm[i].SetPos( pos[i] );
        armState[i] = DecoderAngle(this->myArm[i].GetPos(),i);
        //std::cout<<"The "<<i<<"th "<<"angle is"<< DecoderAngle(pos[i],i)<<" value is "<<pos[i] <<"."<<std::endl;
    }
     
    
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
    std::cout<<"Height is "<<position[0]<<'.'<<"Distance is " <<position[1]<<'.'<<std::endl;
    std::cout<<"Yaw is    "<<attitude[0]<<'.'<<"Roll     is " <<attitude[1]<<'.'<<std::endl;

}

bool RoboticArm::RoboticArmNode::SetActionTime(float actionTime)
{
    this->actionTime = actionTime;
    trajSize = (int) actionTime /dt;
    return true;
}

bool RoboticArm::RoboticArmNode::SetDt(float deltaTime)
{
    this->dt = deltaTime;
    this->publishFrequency = 1/deltaTime;
    trajSize = (int) this->actionTime /dt;
    return true;
}

bool RoboticArm::RoboticArmNode::Ikinematics(){
    float h = SetPointPosition[0];
    float l = SetPointPosition[1];
    //h = 0.02828; l = 0.43488;
    float yaw = SetPointAttitude[0];
    //yaw = 0; 
    
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
        float q4 = q3 -q2 ;
        float q5 = SetPointAttitude[1];
        //q5 = PI;
        float q1 = yaw;
        std::cout<<"q2: "<<CoderAngle(q2,2)<<"q2 rad: "<<q2<<"   q3:"<<CoderAngle(q3,3)<<"    q4:"<<CoderAngle(q4,4)<<" q4angle: "<<q4<<std::endl;
        armDesire[0] = 1;
        armDesire[1] = q1; armDesire[2] = q2; armDesire[3] = q3; armDesire[4] = q4;
        armDesire[5] = q5; 
    }
    return true; 
}


float RoboticArm::RoboticArmNode::Planning(std::vector<float> &param, float t){
    
    float a = param[0]; float b = param[1]; float c = param[2];
    float d = param[3]; float e = param[4]; float f = param[5];
    
    float squareT = t*t;    
    float cubeT = squareT*t;
    float fourT = cubeT * t;
    float rad = a * t *exp(-t) + b * fourT+ c * cubeT + d * squareT + e*t + f;
    return rad;
}


std::vector<float> RoboticArm::RoboticArmNode::GetParam( float qEnd, float qStart, float vEnd, float aEnd, float vStart, float aStart){
    float t = this->actionTime;
    float squareT = t*t;
    float cubeT = t*t*t;
    float fourT = cubeT *t;
    float exponent = exp(-t);
    float f = qStart;
    float a = -(12*f - 12*qEnd + 6*t*vEnd + 6*t*vStart - aEnd*squareT + aStart*squareT)/(6*t*exponent - 6*t + 4*squareT*exponent + cubeT*exponent + 2*squareT); 
    float b = (12*f - 12*qEnd + 12*qEnd*exponent - 12*f*t + 12*qEnd*t + 12*t*vEnd - 4*aEnd*squareT + 2*aEnd*cubeT - 2*aStart*squareT - 8*squareT*vEnd - 4*squareT*vStart - 12*f*exponent - 12*t*vEnd*exponent + 4*aEnd*squareT*exponent + 2*aEnd*cubeT*exponent + 2*aStart*squareT*exponent + 2*aStart*cubeT*exponent + aStart*fourT*exponent + 6*f*squareT*exponent - 6*qEnd*squareT*exponent - 4*squareT*vEnd*exponent + 2*cubeT*vEnd*exponent + 4*squareT*vStart*exponent + 4*cubeT*vStart*exponent)/(2*fourT*(2*t + 6*exponent + 4*t*exponent + squareT*exponent - 6));
    float c = -(12*f - 12*qEnd + 12*qEnd*exponent - 16*f*t + 16*qEnd*t + 12*t*vEnd - 3*aEnd*squareT + 2*aEnd*cubeT - 3*aStart*squareT - 10*squareT*vEnd - 6*squareT*vStart - 12*f*exponent + 4*f*t*exponent - 4*qEnd*t*exponent - 12*t*vEnd*exponent + 3*aEnd*squareT*exponent + aEnd*cubeT*exponent + 3*aStart*squareT*exponent + 3*aStart*cubeT*exponent + aStart*fourT*exponent + 4*f*squareT*exponent - 4*qEnd*squareT*exponent - 2*squareT*vEnd*exponent + cubeT*vEnd*exponent + 6*squareT*vStart*exponent + 3*cubeT*vStart*exponent)/(squareT*(6*t*exponent - 6*t + 4*squareT*exponent + cubeT*exponent + 2*squareT));
    float d = (24*qEnd - 24*f - 6*aStart*t - 12*t*vEnd - 12*t*vStart + 2*aEnd*squareT + 6*aStart*t*exponent + 4*aStart*squareT*exponent + aStart*cubeT*exponent)/(2*(6*t*exponent - 6*t + 4*squareT*exponent + cubeT*exponent + 2*squareT));
    float e = (12*f - 12*qEnd + 6*t*vEnd - aEnd*squareT + aStart*squareT + 2*squareT*vStart + 6*t*vStart*exponent + 4*squareT*vStart*exponent + cubeT*vStart*exponent)/(6*t*exponent - 6*t + 4*squareT*exponent + cubeT*exponent + 2*squareT); std::vector<float> ans = {a,b,c,d,e,f}; 
    return ans; 
} 
    
void RoboticArm::RoboticArmNode::TrajPlan() {
    float qStart[6] = {0};
    float qEnd[6] = {0};

    for(int i = 1; i <= 5; i++){
        qStart[i] = armState[i];
        qEnd[i] = armDesire[i];
        std::vector<float> param = GetParam(qEnd[i],qStart[i],0,0,0,0);
        std::vector<float> qTrajBuf;
        int loopTime = actionTime /dt;
        for(float j = 0; j < loopTime; j++)
            qTrajBuf.push_back(Planning(param,j*dt));
        //qTraj.push_back(qTrajBuf);
        {
            std::lock_guard<std::mutex> traj(varLock);
            qTraj[i-1] = qTrajBuf;
        }
    }
    std::ofstream out;
   out.open("data.txt");
   for(int i = 0; i < trajSize; i ++)
   {
       for(int j = 1; j <= 5; j++)
       {
           std::vector<float> tmp = qTraj[j-1];
           out<<CoderAngle(tmp[i],j)<<' ';
           //out<<tmp[i]<<' ';
       }
       out<<'\n';
   }
   out.close();
   std::cout<<"Data record"<<std::endl;
}


uint16_t RoboticArm::RoboticArmNode::CoderAngle(float angle, int ID){
    uint16_t ans;
    angle = angle /PI * 180;
    switch(ID)
    {
    case 1:
        ans = (uint16_t)(500 + angle/0.24);
        break;
    case 2:
        ans = (uint16_t)(875 - angle/0.24);
        break;
    case 3:
        ans = (uint16_t)(500 - angle/0.24);
        break;
    case 4:
        ans = (uint16_t)(500 + angle/0.24);
        break;
    case 5: 
        ans = 500;
        break;
    }
    
    return ans;
}

float RoboticArm::RoboticArmNode::DecoderAngle(uint16_t angle, int ID){
    float rad;
    switch(ID)
    {
    case 1:
        rad = (angle - 500)*0.24;
        break;
    case 2:
        rad = (875-angle)*0.24;
        break;
    case 3:
        rad = (500-angle)*0.24;
        break;
    case 4:
        rad = (angle - 500) *0.24;
        break;
    case 5: 
        rad = 0;
        break;
    }
    rad = rad / 180 * PI;
    return rad;
}
