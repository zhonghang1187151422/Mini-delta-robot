#include <iostream>
#include <cmath>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
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

    DeltaRobot::DeltaRobotControl robot;

    robot.Open();
    robot.SetControlMode(DeltaRobot::ROBOT_MODE_POSITION);

    robot.JointControl(DeltaRobot::v4DJoint(0, 0, 0, 0));

    while(!bExit)
    {
        robot.PrintfRobotStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    //return
    return(0);
}


