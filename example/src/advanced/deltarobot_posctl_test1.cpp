#include <iostream>
#include <cmath>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include "DeltaRobotKinematics.h"
#include "DeltaRobotControl.h"


bool bExit = false;


void processSignal(int sign)
{
    ActuatorController::getInstance()->disableAllActuators();
    this_thread::sleep_for(std::chrono::milliseconds(200));
    bExit = true;
}


int main(int argc, char *argv[])
{
    //Associate program interrupt signals and call processSignal when you end the program with ctrl-c
    signal(SIGINT, processSignal);

    //define a robor handle and open it
    DeltaRobot::DeltaRobotControl robot;
    robot.Open();
    //printf robot information
    robot.PrintfRobotInfomation();
    //set the robot in position mode
    robot.SetControlMode(DeltaRobot::ROBOT_MODE_POSITION);
    //move the robot to a target point 
    robot.JointControl(DeltaRobot::v4DJoint(0, 0, 0, 0));

    //while loop...
    while(!bExit)
    {
        robot.PositionControl(DeltaRobot::v4DPoint(DeltaRobot::v3DPoint(0, 0, 0.15), 0), 0.8, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        robot.PositionControl(DeltaRobot::v4DPoint(DeltaRobot::v3DPoint(0, 0, 0.25), 0), 0.8, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    //delay
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //clese the robot
    robot.Close();
    
    //return
    return(0);
}


