#ifndef DELTAROBOT_CONTROL_H
#define DELTAROBOT_CONTROL_H


#include <vector>
#include "DeltaRobotDataType.h"
#include "DeltaRobotKinematics.h"
#include "DeltaRobotMotionPlanning.h"
#include "actuatorcontroller.h"


namespace DeltaRobot
{
    //define actuator ID
    #define ACTUATOR_ID_1           11
    #define ACTUATOR_ID_2           12
    #define ACTUATOR_ID_3           13
    #define ACTUATOR_ID_4           14
    //define joint min and max value
    #define ACTUATOR_MIN_ANGLE      -M_PI/6
    #define ACTUATOR_MAX_ANGLE      M_PI/2
    //define actuator reducer gear ratio
    #define REDUCER_RATIO           36
    //system control period
    #define CTL_PERIOD              0.005
    //robot maximum velocity
    #define MAX_VELOCITY            0.3
    //robot minimum velocity
    #define MIN_VELOCITY            0.001

    //robot status enum
    enum ROBOT_STATUS
    {
        ROBOT_STS_OFF,
        ROBOT_STS_ERROR,
        ROBOT_STS_ON,
    };

    //robot control mode
    enum ROBOT_MODE
    {
        ROBOT_MODE_CURRENT,
        ROBOT_MODE_VELOCITY,
        ROBOT_MODE_POSITION,
        ROBOT_MODE_ERROR,
    };

    //class
    class DeltaRobotControl : private DeltaRobotKinematics, private MotionPlanning
    {
    private:
        //control period
        double ControlPeriod;
        //actuators ID
        vector<uint8_t> ActuatorsIDList;

    private:
        //the controller handle
        ActuatorController * pController;
        //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
        //when the error occurs, ec value will be modified by SDK to the corresponding error code
        Actuator::ErrorsDefine ec;
        //the connected actuators and get the unified ID.
        std::vector<ActuatorController::UnifiedID> UnifiedidArray;

    private:
        v4DVelocity TargetVelocity;
        double TargetVelocityAccel;

    public:
        //default construct function
        DeltaRobotControl();
        //construct function
        DeltaRobotControl(double iR, double ir, double iL1, double iL2, double iH);
        //open the delta robot
        ROBOT_STATUS Open(void);
        //close the delta robot
        ROBOT_STATUS Close(void);
        //set the delta robot control mode
        bool SetControlMode(ROBOT_MODE mode);
        //get the delta robot control mode
        ROBOT_MODE GetControlMode(void);
        //get the delta robot joint angles
        v4DJoint GetJointAngles(void);
        //get the delta robot current position
        v4DPoint GetPosition(void);
        //delta robot joint control
        bool JointControl(v4DJoint joint, double velrate = 0.5, double accelrate = 0.5);
        //delta robot position control
        bool PositionControl(v4DPoint pos, double velrate = 0.5, double accelrate = 0.5);
        //delta robot position control minor
        bool PositionControlMinor(v4DPoint pos);
        //velocity control refresh loop
        bool VelocityControlLoop(void);
        //set the robot velocity
        bool VelocityControl(v4DVelocity vel);
        /*========================================== display function ============================================*/
        //printf robot infomation
        void PrintfRobotInfomation(void);
        //printf robot status
        void PrintfRobotStatus(void);

    private:
        //inline function for ActuatorController::UnifiedID vector sort
        static bool UnifiedIDCompare(const ActuatorController::UnifiedID &uID1, const ActuatorController::UnifiedID &uID2)
        {
            return(uID1.actuatorID < uID2.actuatorID);
        }

    };

}




#endif //DELTAROBOT_CONTROL_H