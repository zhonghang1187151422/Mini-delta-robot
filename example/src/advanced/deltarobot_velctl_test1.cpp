#include <iostream>
#include <linux/input.h>
#include <termio.h>
#include <stdio.h> 
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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


void velocitycontrolthread(DeltaRobot::DeltaRobotControl *handles)
{
    bool ret = true;
    while((!bExit) && ret)
    {
        ret = handles->VelocityControlLoop();
    }
    std::cout << "velocity control loop out, there has a error in control loop!" << std::endl;
}


int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}


int main(int argc, char *argv[])
{
    //Associate program interrupt signals and call processSignal when you end the program with ctrl-c
    signal(SIGINT, processSignal);

    DeltaRobot::DeltaRobotControl robot;
    robot.Open();
    //printf robot information
    robot.PrintfRobotInfomation();
    //set the robot in position mode
    robot.SetControlMode(DeltaRobot::ROBOT_MODE_POSITION);
    //move the robot to a target point 
    robot.JointControl(DeltaRobot::v4DJoint(0, 0, 0, 0));

    //creat a velocity control thread and start
    std::thread velstl_thread(velocitycontrolthread, &robot);
    velstl_thread.detach();

    DeltaRobot::v4DVelocity v4dvelocity(0, 0, 0, 0);
    //while loop...
    while(!bExit)
    {
        //get the keyboard value
        int keyvalue = scanKeyboard();
        std::cout << " keyboard value is: " << keyvalue << std::endl;
        switch(keyvalue)
        {
            case 97:
                v4dvelocity.Velocity.Vx = 0.1;
            break;
            case 100:
                v4dvelocity.Velocity.Vx = -0.1;
            break;
            case 119:
                v4dvelocity.Velocity.Vz = 0.1;
            break;
            case 120:
                v4dvelocity.Velocity.Vz = -0.1;
            break;
            case 115:
                v4dvelocity.Velocity = DeltaRobot::v3DVelocity(0,0,0);
            break;
            default:
                v4dvelocity.Velocity = DeltaRobot::v3DVelocity(0,0,0);
            break;
        }
        //robot control
        robot.VelocityControl(v4dvelocity);
    }
    
    //delay
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //clese the robot
    robot.Close();
    
    //return
    return(0);
}


