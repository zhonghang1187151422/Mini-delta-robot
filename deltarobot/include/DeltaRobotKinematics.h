#ifndef DELTAROBOT_KINEMATICS_H
#define DELTAROBOT_KINEMATICS_H

#include <vector>
#include <iostream>
#include "DeltaRobotDataType.h"


namespace DeltaRobot
{
    // define default delta parameters
    #define DEFAULT_R       0.0638
    #define DEFAULT_r       0.0538
    #define DEFAULT_L1      0.1065
    #define DEFAULT_L2      0.15
    #define DEFAULT_H       0.035

    //define delta robot joint max and min value
    #define JOINT_MIN       -M_PI / 6
    #define JOINT_MAX       M_PI / 2

    //class
    class DeltaRobotKinematics
    {
    private:
        //delta robot geometric parameters
        double alpha1, alpha2, alpha3; //actuators angles (rad)
        double R, r, L1, L2; //active arm and passive arm radius and length
        double H; //actuator circle to top mounting plate length

    public:
        //construct function
        DeltaRobotKinematics();
        DeltaRobotKinematics(double iR, double ir, double iL1, double iL2, double iH);
        //destructor function
        ~DeltaRobotKinematics();
        //forward kinematics
        bool ForwardKinematics(v3DJoint th, v3DPoint& pos);
        //inverse kinematics
        bool InverseKinematics(v3DPoint pos, v3DJoint& th);
        //forward singular kinematics
        bool ForwardSingularKinematics(v3DJoint th, v3DJointRate vth, v3DVelocity& vel);
        //inverse singular kinematics
        bool InverseSingularKinematics(v3DJoint th, v3DVelocity vel, v3DJointRate& vth);
        
    };
    

} // namespace DeltaRobot

#endif //DELTAROBOT_KINEMATICS_H