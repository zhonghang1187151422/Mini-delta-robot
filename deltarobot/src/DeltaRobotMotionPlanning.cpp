#include <cmath>
#include "DeltaRobotMotionPlanning.h"
#include "DeltaRobotControl.h"


using namespace std;


namespace DeltaRobot
{
    //default construct function 
    MotionPlanning::MotionPlanning(): Af(0.5), a(0.25), dT(0.005) {}

    //construct function 
    MotionPlanning::MotionPlanning(double iAf, double ia, double idT)
    {
        Af = iAf;
        a = ia;
        dT = idT;
    }

    //set motion planning parameters
    void MotionPlanning::SetMotionPlanningPara(double iAf, double ia, double idT)
    {
        Af = iAf;
        a = ia;
        dT = idT;
    }

    /**
     * @brief mod sin cam motion planning
     * @param pointnum -> <int> the point number
     * @param modsindata -> <vector<double>&> reference of the return mod sin data
     * @return true means successful, false means failed
     * @warning ref: Masey R J M , Gray J O , Dodd T J , et al. Elliptical point to point trajectory planning using electronic cam motion profiles for high speed industrial pick and place robots[J]. 2009.
    **/
    bool MotionPlanning::ModSinMotionPlanning(int pointnum, vector<double>& modsindata)
    {
        short error=0;
        double Ca, u, du;
        int i;
        double temp;

        //check the parameters
        if((Af<0) || (Af>1) || (a<0.05) || (a>0.25))
        {
            return(false);
        }else
        {
            //calculate Ca
            Ca = 1/(4*a/M_PI + 16*a*a/M_PI/M_PI - 12*a*a/M_PI);
            //calculate time
            u = 0;
            du = 1/(static_cast<double>(pointnum-1));
            //motion planning
            for(i=0; i<pointnum; i++)
            {
                if(u <= Af*a)
                    temp = Ca*(a*u/M_PI - 2*Af*a*a/M_PI/M_PI*sin(M_PI*u/2/Af/a));
                else if(u < 4*Af*a)
                    temp = Ca*(a*u/M_PI + 2*Af*a*a/M_PI/M_PI*(8-9*sin(M_PI*u/6/Af/a + M_PI/3)));
                else if(u < (1-4*a*(1-Af)))
                    temp = 4*Ca*(a*u/M_PI + Af*a*a*(4-3*M_PI)/M_PI/M_PI);
                else if(u < (1-a*(1-Af)))
                    temp = 1 - Ca*(a*(1-u)/M_PI + 2*a*a*(1-Af)/M_PI/M_PI*(8-9*sin(M_PI*(1-u)/6/a/(1-Af) + M_PI/3)));
                else
                    temp = 1 - Ca*(a*(1-u)/M_PI - 2*a*a*(1-Af)/M_PI/M_PI*sin(M_PI*(1-u)/2/a/(1-Af)));
                //push back
                modsindata.push_back(temp);
                //referesh time
                u += du;
            }
            //return
            return(true);
        }
    }

    /**
     * @brief joint motion planning
     * @param startjoint -> <v4DJoint> the start joint
     * @param endjoint -> <v4DJoint> the end joint
     * @param v -> <double> the motion velocity, units: (m/s)
     * @param jointbuff -> <vector<v4DJoint>&> the reference of the motion planning joint buff
     * @return true means successful, false means failed
     * @warning none
    **/
    bool MotionPlanning::JointMotionPlanning(v4DJoint startjoint, v4DJoint endjoint, double v, vector<v4DJoint>& jointbuff)
    {
        //find the maxmum joint range
        v4DJoint diffjoint = startjoint - endjoint;
        double max_joint = abs(diffjoint.Joint.th1);
        if(abs(diffjoint.Joint.th2) > max_joint)
            max_joint = abs(diffjoint.Joint.th2);
        if(abs(diffjoint.Joint.th3) > max_joint)
            max_joint = abs(diffjoint.Joint.th3);
        if(abs(diffjoint.YawAngle) > max_joint)
            max_joint = abs(diffjoint.YawAngle);
        //calculate the joint bumber
        int joint_num = (int)(max_joint / (v * dT)) + 1;
        //check the munber
        if(joint_num <= 3)
        {
            joint_num = 3;
        }
        //mod sin motion planning
        vector<double> modsindata;
        if(!ModSinMotionPlanning(joint_num, modsindata))
        {
            return(false);
        }
        //joint motion planning
        v4DJoint joint;
        for(int i=0; i<joint_num; i++)
        {
            joint.Joint.th1 = startjoint.Joint.th1 + (endjoint.Joint.th1 - startjoint.Joint.th1) * modsindata.at(i);
            joint.Joint.th2 = startjoint.Joint.th2 + (endjoint.Joint.th2 - startjoint.Joint.th2) * modsindata.at(i);
            joint.Joint.th3 = startjoint.Joint.th3 + (endjoint.Joint.th3 - startjoint.Joint.th3) * modsindata.at(i);
            joint.YawAngle = startjoint.YawAngle + (endjoint.YawAngle - startjoint.YawAngle) * modsindata.at(i);
            jointbuff.push_back(joint);
        }
        // add the last point
        jointbuff.push_back(endjoint);
        //return
        return(true);
    }

    /**
     * @brief 3D Line motion planning
     * @param startpoint -> <v3DPoint> the start point
     * @param endpoint -> <v3DPoint> the end point
     * @param v -> <double> the motion velocity, units: (m/s)
     * @param pointbuff -> <vector<v3DPoint>&> the reference of the motion planning point buff
     * @return true means successful, false means failed
     * @warning none
    **/
    bool MotionPlanning::LineMotionPlanning(v4DPoint startpoint, v4DPoint endpoint, double v, vector<v4DPoint>& pointbuff)
    {
        // line length
        double linelength = sqrt(pow((startpoint.Position.Px - endpoint.Position.Px), 2) + pow((startpoint.Position.Py - endpoint.Position.Py), 2) + pow((startpoint.Position.Pz - endpoint.Position.Pz), 2));
        //calculate the point number
        int pointnum;
        int pointnum_line = (int)(linelength / (v * dT)) + 1;
        int pointnum_theta = (int)(abs(endpoint.YawAngle - startpoint.YawAngle) / (10 * v * dT)) + 1;
        //select the maximum number
        if(pointnum_line >= pointnum_theta)
        {
            pointnum = pointnum_line;
        }else
        {
            pointnum = pointnum_theta;
        }
        //check the munber
        if(pointnum <= 3)
        {
            pointnum = 3;
        }
        //mod sin motion planning
        vector<double> modsindata;
        if(!ModSinMotionPlanning(pointnum, modsindata))
        {
            return(false);
        }
        //line motion planning
        v4DPoint point;
        for(int i=0; i<pointnum; i++)
        {
            point.Position.Px = startpoint.Position.Px + (endpoint.Position.Px - startpoint.Position.Px) * modsindata.at(i);
            point.Position.Py = startpoint.Position.Py + (endpoint.Position.Py - startpoint.Position.Py) * modsindata.at(i);
            point.Position.Pz = startpoint.Position.Pz + (endpoint.Position.Pz - startpoint.Position.Pz) * modsindata.at(i);
            point.YawAngle = startpoint.YawAngle + (endpoint.YawAngle - startpoint.YawAngle) * modsindata.at(i);
            pointbuff.push_back(point);
        }
        // add the last point
        pointbuff.push_back(endpoint);
        //return
        return(true);
    }


}


