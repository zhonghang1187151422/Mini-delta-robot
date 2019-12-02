#include <iostream>
#include <iomanip>
#include <string.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include "DeltaRobotControl.h"
#include "DeltaRobotKinematics.h"
#include "DeltaRobotMotionPlanning.h"
#include "actuatorcontroller.h"


using namespace std;


namespace DeltaRobot
{
    //default construct function
    DeltaRobotControl::DeltaRobotControl(): ControlPeriod(CTL_PERIOD), TargetVelocity(v3DVelocity(0,0,0), 0), TargetVelocityAccel(0), DeltaRobotKinematics()
    {
        //actuators ID
        ActuatorsIDList.push_back(ACTUATOR_ID_1);
        ActuatorsIDList.push_back(ACTUATOR_ID_2);
        ActuatorsIDList.push_back(ACTUATOR_ID_3);
        ActuatorsIDList.push_back(ACTUATOR_ID_4);
    }

    //construct function
    DeltaRobotControl::DeltaRobotControl(double iR, double ir, double iL1, double iL2, double iH): ControlPeriod(CTL_PERIOD), TargetVelocity(v3DVelocity(0,0,0), 0), TargetVelocityAccel(0), DeltaRobotKinematics(iR, ir, iL1, iL2, iH)
    {
        //actuators ID
        ActuatorsIDList.push_back(ACTUATOR_ID_1);
        ActuatorsIDList.push_back(ACTUATOR_ID_2);
        ActuatorsIDList.push_back(ACTUATOR_ID_3);
        ActuatorsIDList.push_back(ACTUATOR_ID_4);
    }

    /**
     * @brief open the delta robot: enable all the actuators and the actuators enter in current mode
     * @param void
     * @return ROBOT_STATUS enum
     * @warning none
    **/
    ROBOT_STATUS DeltaRobotControl::Open(void)
    {
        //Initialize the controller
        pController = ActuatorController::initController();
        //Find the connected actuators and get the unified ID.
        UnifiedidArray = pController->lookupActuators(ec);
        //check the UnifiedID if or not correct
        if(UnifiedidArray.size() == 4)
        {
            //sort the UnifiedidArray by actuatorID
            sort(UnifiedidArray.begin(), UnifiedidArray.end(), UnifiedIDCompare);
            //check the actuators ID
            for(int i=0; i<ActuatorsIDList.size(); i++)
            {
                if(UnifiedidArray.at(i).actuatorID != ActuatorsIDList.at(i))
                {
                    cout << "actuator ID error: " << UnifiedidArray.at(i).actuatorID << endl;
                    return(ROBOT_STS_ERROR);
                }
            }
        }else
        {
            cout << "find actuators are shown in blow: " << endl;
            for(auto uID : UnifiedidArray)
            {
                cout << (int)uID.actuatorID << "\t" << uID.ipAddress.c_str() << endl;
            }
            return(ROBOT_STS_ERROR);
        }
        //enable all the actuators
        if(!pController->enableActuatorInBatch(UnifiedidArray))
        {
            cout << "enable actuators failed!" << endl;
            return(ROBOT_STS_ERROR);
        }
        //set the actuators in current mode
        pController->activateActuatorModeInBantch(UnifiedidArray, Actuator::Mode_Cur);
        //set the actuators min and max value
        for(int i=0; i<3; i++)
        {
            pController->setPositionUmin(UnifiedidArray.at(i).actuatorID, ACTUATOR_MIN_ANGLE * REDUCER_RATIO / (2*M_PI));
            pController->setPositionUmax(UnifiedidArray.at(i).actuatorID, ACTUATOR_MAX_ANGLE * REDUCER_RATIO / (2*M_PI));
        }
        pController->setPositionUmin(UnifiedidArray.at(3).actuatorID, -120);
        pController->setPositionUmax(UnifiedidArray.at(3).actuatorID, 120);
        //return 
        return(ROBOT_STS_ON);
    }

    /**
     * @brief close the delta robot: disable all the actuators
     * @param void
     * @return ROBOT_STATUS enum
     * @warning none
    **/
    ROBOT_STATUS DeltaRobotControl::Close(void)
    {
        //disable all actuators
        if(ActuatorController::getInstance()->disableAllActuators())
        {
            return(ROBOT_STS_OFF);
        }else
        {
            return(ROBOT_STS_ERROR);
        }
    }

    /**
     * @brief set the delta robot control mode
     * @param mode -> <ROBOT_MODE> the mode of the robot: ROBOT_CURRENT, ROBOT_VELOCITY, ROBOT_POSITION
     * @return bool -> true is success, false is failed
     * @warning none
    **/
    bool DeltaRobotControl::SetControlMode(ROBOT_MODE mode)
    {
        switch(mode)
        {
            case ROBOT_MODE_CURRENT:
                pController->activateActuatorModeInBantch(UnifiedidArray, ActuatorMode::Mode_Cur);
            break;
            case ROBOT_MODE_VELOCITY:
                pController->activateActuatorModeInBantch(UnifiedidArray, ActuatorMode::Mode_Vel);
            break;
            case ROBOT_MODE_POSITION:
                pController->activateActuatorModeInBantch(UnifiedidArray, ActuatorMode::Mode_Pos);
            break;
            default:
                return(false);
            break;
        }
        //return
        return(true);
    }

    /**
     * @brief get the delta robot control mode
     * @param void -> <ROBOT_MODE> the mode of the robot: ROBOT_CURRENT, ROBOT_VELOCITY, ROBOT_POSITION
     * @return ROBOT_MODE -> <ROBOT_MODE> the mode of the robot: ROBOT_CURRENT, ROBOT_VELOCITY, ROBOT_POSITION, ROBOT_MODE_ERROR
     * @warning none
    **/
    ROBOT_MODE DeltaRobotControl::GetControlMode(void)
    {
        vector<Actuator::ActuatorMode> actuatorsmode;
        //read the actuator mode back
        for(auto uID : UnifiedidArray)
        {
            actuatorsmode.push_back(pController->getActuatorMode(uID.actuatorID));
        }
        //check the actuators mode
        if(actuatorsmode == vector<Actuator::ActuatorMode>{ActuatorMode::Mode_Cur, ActuatorMode::Mode_Cur, ActuatorMode::Mode_Cur, ActuatorMode::Mode_Cur})
        {
            return(ROBOT_MODE_CURRENT);
        }else if(actuatorsmode == vector<Actuator::ActuatorMode>{ActuatorMode::Mode_Vel, ActuatorMode::Mode_Vel, ActuatorMode::Mode_Vel, ActuatorMode::Mode_Vel})
        {
            return(ROBOT_MODE_VELOCITY);
        }else if(actuatorsmode == vector<Actuator::ActuatorMode>{ActuatorMode::Mode_Pos, ActuatorMode::Mode_Pos, ActuatorMode::Mode_Pos, ActuatorMode::Mode_Pos})
        {
            return(ROBOT_MODE_POSITION);
        }else
        {
            return(ROBOT_MODE_ERROR);
        }
    }

    /**
     * @brief get the delta robot joint angles
     * @param void
     * @return v4DJoint-> the return joint angles data, Units: (rad)
     * @warning 
    **/
    v4DJoint DeltaRobotControl::GetJointAngles(void)
    {
        v4DJoint joint;
        //read the actuators rot data and convert to rad
        joint.Joint.th1 = pController->getPosition(UnifiedidArray.at(0).actuatorID, true) * 2*M_PI / REDUCER_RATIO;
        joint.Joint.th2 = pController->getPosition(UnifiedidArray.at(1).actuatorID, true) * 2*M_PI / REDUCER_RATIO;
        joint.Joint.th3 = pController->getPosition(UnifiedidArray.at(2).actuatorID, true) * 2*M_PI / REDUCER_RATIO;
        joint.YawAngle = pController->getPosition(UnifiedidArray.at(3).actuatorID, true) * 2*M_PI / REDUCER_RATIO;
        //return
        return(joint);
    }

    /**
     * @brief get the delta robot current position
     * @param void
     * @return v4DPoint -> the return point data
     * @warning 
    **/
    v4DPoint DeltaRobotControl::GetPosition(void)
    {
        //read the joint angles
        v4DJoint jointangles = this->GetJointAngles();
        //forward kinematics
        v4DPoint pointpos;
        this->ForwardKinematics(jointangles.Joint, pointpos.Position);
        pointpos.YawAngle = jointangles.YawAngle;
        //return
        return(pointpos);
    }

    /**
     * @brief delta robot joint control
     * @param joint -> <v4DJoint> target joint value, Units: (rad)
     * @return true is successful, flase is failed
     * @warning 
    **/
    bool DeltaRobotControl::JointControl(v4DJoint joint, double velrate, double accelrate)
    {
        //get the current joint value
        v4DJoint curr_v4djoint = this->GetJointAngles();
        //calculate velocity and accelerate
        double ia = -0.2 * accelrate + 0.25; 
        this->SetMotionPlanningPara(0.5, ia, ControlPeriod);
        //joint motion planning
        vector<v4DJoint> v4djointbuff;
        double vel = velrate * (MAX_VELOCITY - MIN_VELOCITY) + MIN_VELOCITY;
        if(!this->JointMotionPlanning(curr_v4djoint, joint, vel, v4djointbuff))
        {
            cout << "joint planning motion failed!" << endl;
            return(false);
        }
        //send the joint data to the actuators
        cout << "planning point buff: " << v4djointbuff.size() << endl;
        for(auto v4djoint : v4djointbuff)
        {
            pController->setPosition(UnifiedidArray.at(0).actuatorID, (v4djoint.Joint.th1 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(1).actuatorID, (v4djoint.Joint.th2 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(2).actuatorID, (v4djoint.Joint.th3 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(3).actuatorID, (v4djoint.YawAngle * REDUCER_RATIO / (2*M_PI)));
            //system delay
            std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1000*CTL_PERIOD)));
            //cout << v4djoint.Joint.th1 << "\t" << v4djoint.Joint.th2 << "\t" << v4djoint.Joint.th3 << "\t" << v4djoint.YawAngle << endl;
        }
        //return
        return(true);
    }

    /**
     * @brief delta robot position control
     * @param pos -> <v3DPoint> target 3D position
     * @param velrate -> <double> velocity rate, range: 0-1, default = 0.5
     * @param accelrate -> <double> accelerate rate, range: 0-1, default = 0.5
     * @return true is successful, flase is failed
     * @warning this function is a blocking type
    **/
    bool DeltaRobotControl::PositionControl(v4DPoint pos, double velrate, double accelrate)
    {
        //get the robot current position
        v4DPoint cur_v4dpoint = this->GetPosition();
        //calculate velocity and accelerate
        double ia = -0.2 * accelrate + 0.25; 
        this->SetMotionPlanningPara(0.5, ia, ControlPeriod);
        //Line motion planning
        vector<v4DPoint> v4dpointbuff;
        double vel = velrate * (MAX_VELOCITY - MIN_VELOCITY) + MIN_VELOCITY;
        if(!this->LineMotionPlanning(cur_v4dpoint, pos, vel, v4dpointbuff))
        {
            cout << "line planning motion failed!" << endl;
            return(false);
        }
        // delta robot inverse kinematics
        vector<v4DJoint> v4djointbuff;
        for(auto v4dpoint : v4dpointbuff)
        {
            v3DJoint th;
            v4DJoint joint;
            if(this->InverseKinematics(v4dpoint.Position, th))
            {
                joint.Joint = th;
                joint.YawAngle = v4dpoint.YawAngle;
                v4djointbuff.push_back(joint);
            }else
            {
                return(false);
            }
        }
        // send the joint data to the actuators
        cout << "planning point buff: " << v4djointbuff.size() << endl;
        for(auto v4djoint : v4djointbuff)
        {
            pController->setPosition(UnifiedidArray.at(0).actuatorID, (v4djoint.Joint.th1 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(1).actuatorID, (v4djoint.Joint.th2 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(2).actuatorID, (v4djoint.Joint.th3 * REDUCER_RATIO / (2*M_PI)));
            pController->setPosition(UnifiedidArray.at(3).actuatorID, (v4djoint.YawAngle * REDUCER_RATIO / (2*M_PI)));
            //system delay
            std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1000*CTL_PERIOD)));
            //cout << v4djoint.Joint.th1 << "\t" << v4djoint.Joint.th2 << "\t" << v4djoint.Joint.th3 << "\t" << v4djoint.YawAngle << endl;
        }
    }

    /**
     * @brief delta robot position control minor
     * @param pos -> <v4DPoint> target position point
     * @return bool, true is successful, flase is failed
     * @warning 
    **/
    bool DeltaRobotControl::PositionControlMinor(v4DPoint pos)
    {
        //inverse kinematics
        v3DJoint th;
        v4DJoint joint;
        if(this->InverseKinematics(pos.Position, th))
        {
            joint.Joint = th;
            joint.YawAngle = pos.YawAngle;
        }else
        {
            return(false);
        }
        // send the joint data to the actuators
        pController->setPosition(UnifiedidArray.at(0).actuatorID, (joint.Joint.th1 * REDUCER_RATIO / (2*M_PI)));
        pController->setPosition(UnifiedidArray.at(1).actuatorID, (joint.Joint.th2 * REDUCER_RATIO / (2*M_PI)));
        pController->setPosition(UnifiedidArray.at(2).actuatorID, (joint.Joint.th3 * REDUCER_RATIO / (2*M_PI)));
        pController->setPosition(UnifiedidArray.at(3).actuatorID, (joint.YawAngle * REDUCER_RATIO / (2*M_PI)));
        //system delay
        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1000*CTL_PERIOD)));
        //return
        return(true);
    }

    /**
     * @brief velocity control refresh loop
     * @param void
     * @return bool, true is successful, flase is failed
     * @warning 
    **/
    bool DeltaRobotControl::VelocityControlLoop(void)
    {
        //get the robot current position
        v4DPoint cur_v4dpoint = this->GetPosition();
        //refresh the target position
        v4DPoint tar_v4dpoint;
        tar_v4dpoint.Position.Px = cur_v4dpoint.Position.Px + this->TargetVelocity.Velocity.Vx * CTL_PERIOD;
        tar_v4dpoint.Position.Py = cur_v4dpoint.Position.Py + this->TargetVelocity.Velocity.Vy * CTL_PERIOD;
        tar_v4dpoint.Position.Pz = cur_v4dpoint.Position.Pz + this->TargetVelocity.Velocity.Vz * CTL_PERIOD;
        tar_v4dpoint.YawAngle = cur_v4dpoint.YawAngle + this->TargetVelocity.YawAngleRate * CTL_PERIOD;
        // control
        if(!this->PositionControlMinor(tar_v4dpoint))
        {
            return(false);
        }
        //return
        return(true);
    }

    /**
     * @brief set the robot velocity
     * @param vel -> <v4DVelocity> target velocity, Units: (m/s)
     * @param accelrate -> <double> accelerate ratio, range: [0,1], default: 0.5
     * @return bool, true is successful, flase is failed
     * @warning 
    **/
    bool DeltaRobotControl::VelocityControl(v4DVelocity vel)
    {
        this->TargetVelocity = vel;
        //return
        return(true);
    }




    /**
     * @brief printf robot infomation
     * @param void
     * @return void
     * @warning 
    **/
    void DeltaRobotControl::PrintfRobotInfomation(void)
    {
        cout << "\n" << endl;
        cout << "----------------------- printf the robot information ----------------------- " << endl;
        //printf actuators ID
        cout << "actuators ID: " << "\t";
        for(auto uID : UnifiedidArray)
        {
            cout << uID.actuatorID << "\t";
        }
        cout << endl;
        
    }
    
    /**
     * @brief printf robot status
     * @param void
     * @return void
     * @warning 
    **/
    void DeltaRobotControl::PrintfRobotStatus(void)
    {
        cout << "\n" << endl;
        cout << "----------------------- printf the robot status ----------------------- " << endl;
        //printf robot control mode
        ROBOT_MODE mode = this->GetControlMode();
        cout << "(1) robot control mode: " << "\t";
        switch(mode)
        {
            case ROBOT_MODE_CURRENT:
                cout << "CURRENT MODE";
                break;
            case ROBOT_MODE_VELOCITY:
                cout << "VELOCITY MODE";
                break;
            case ROBOT_MODE_POSITION:
                cout << "POSITION MODE";
                break;
            case ROBOT_MODE_ERROR:
                cout << "ERROR MODE";
                break;
            default:
                break;
        }
        cout << endl;
        //printf robot joint angles
        v4DJoint v4djoint = this->GetJointAngles();
        cout << "(2) joint angles <rad>: " << "\t";
        cout << fixed << setprecision(4) << v4djoint.Joint.th1 << "\t" << v4djoint.Joint.th2 << "\t" << v4djoint.Joint.th3 << "\t" << v4djoint.YawAngle << endl;
        //printf robot position coordinate
        v4DPoint v4dpoint = this->GetPosition();
        cout << "(3) end position <m,rad>: " << "\t";
        cout << fixed << setprecision(4) << v4dpoint.Position.Px << "\t" << v4dpoint.Position.Py << "\t" << v4dpoint.Position.Pz << "\t" << v4dpoint.YawAngle << endl;

    }

}