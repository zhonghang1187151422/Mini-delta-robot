/*
* actuator test1
*/
#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include "DeltaRobotKinematics.h"


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
    signal(SIGINT,processSignal);
    //Initialize the controller
    ActuatorController * pController = ActuatorController::initController();
    //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    //when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    //Find the connected actuators and get the unified ID.
    std::vector<ActuatorController::UnifiedID> UnifiedidArray = pController->lookupActuators(ec);

    //enable all actuators and set the actuators in Position mode
    if(UnifiedidArray.size() > 0)
    {
        std::cout << "find " << UnifiedidArray.size() << " actuators." << std::endl;
        pController->enableActuatorInBatch(UnifiedidArray);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        pController->activateActuatorModeInBantch(UnifiedidArray, ActuatorMode::Mode_Pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "enable and set the actuators in position mode successfully!" << std::endl;
    }else
    {
        return 1;
    }

    //get the current position
    double inipos = pController->getPosition(UnifiedidArray.at(0).actuatorID, true);
    
    double tick = 0;
    int period = 5;
    // while loop ...
    while (!bExit)
    {
        //tick refresh
        tick += static_cast<double>(period) / 1000;
        //cal
        double y = 30 * std::sin(0.2 * tick) + inipos;

        //control the actuators
        pController->setPosition(UnifiedidArray.at(0).actuatorID, y);
        std::cout << "time: " << tick << "value: " << y << std::endl;

        //system delay
        std::this_thread::sleep_for(std::chrono::milliseconds(period));
    }
    

    //return 
    return 0;
}

