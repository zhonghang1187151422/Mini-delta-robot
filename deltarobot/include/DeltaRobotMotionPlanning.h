#ifndef DELTAROBOT_MOTIONPLANNING_H
#define DELTAROBOT_MOTIONPLANNING_H

#include <vector>
#include <iostream>
#include "DeltaRobotDataType.h"


using namespace std;


namespace DeltaRobot
{
    class MotionPlanning
    {
    private:
        //planning parameters
        double Af;      // 0 < Af < 1, accelerate time and decelerate time
        double a;       // average timne: 100*(1-4a)%, range: 0.05 <= a <= 0.25
        double dT;      // the time period of the points

    public:
        //default construct function 
        MotionPlanning();
        //construct function 
        MotionPlanning(double iAf, double ia, double idT);
        //set motion planning parameters
        void SetMotionPlanningPara(double iAf, double ia, double idT);
        //mod sin cam motion planning
        bool ModSinMotionPlanning(int pointnum, vector<double>& modsindata);
        //joint motion planning
        bool JointMotionPlanning(v4DJoint startjoint, v4DJoint endjoint, double v, vector<v4DJoint>& jointbuff);
        //3D Line motion planning
        bool LineMotionPlanning(v4DPoint startpoint, v4DPoint endpoint, double v, vector<v4DPoint>& pointbuff);

    };
}


#endif //DELTAROBOT_MOTIONPLANNING_H