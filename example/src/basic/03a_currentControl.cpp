﻿/*
*执行器电流控制
*/

#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
using namespace std;


int main(int argc, char *argv[])
{
    //Initialize the controller
    ActuatorController * pController = ActuatorController::initController();
    //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    //when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    //Find the connected actuators and return the UnifiedID of all actuators found.
    std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
    //If the size of the uIDArray is greater than zero, the connected actuators have been found
    if(uIDArray.size() > 0)
    {
        ActuatorController::UnifiedID actuator = uIDArray.at(0);
        //Enable an actuator,If there are no actuators with the same ID under multiple IP addresses, you can omit the ipAddress parameter
        pController->enableActuator(actuator.actuatorID,actuator.ipAddress);
        //Activate current mode
        pController->activateActuatorMode(actuator.actuatorID,Actuator::Mode_Cur);

        cout << "set current to 2.6A" << endl;
        pController->setCurrent(actuator.actuatorID,2.6);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cout << "set current to -2.6A" << endl;
        pController->setCurrent(actuator.actuatorID,-2.6);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //Disable all connected actuators
        pController->disableAllActuators();
        //insure that all actuators have been closed
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else
    {
        cout << "Connected error code:" << hex << ec << endl;
    }

    return 0;
}
