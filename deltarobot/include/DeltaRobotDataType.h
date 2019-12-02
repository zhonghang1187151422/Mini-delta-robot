#ifndef DELTAROBOT_DATATYPE_H
#define DELTAROBOT_DATATYPE_H


#include <vector>


namespace DeltaRobot
{
    //struct, 3D point
    struct v3DPoint
    {
    public:
        double Px;
        double Py;
        double Pz;
    public:
        v3DPoint():Px(0), Py(0), Pz(0){}
        v3DPoint(double px, double py, double pz)
        {
            this->Px = px; this->Py = py; this->Pz = pz;
        }
    public:
        v3DPoint operator+(const v3DPoint &others)
        {
            v3DPoint temp;
            temp.Px = this->Px + others.Px;
            temp.Py = this->Py + others.Py;
            temp.Pz = this->Pz + others.Pz;
            return(temp);
        }
        v3DPoint operator-(const v3DPoint &others)
        {
            v3DPoint temp;
            temp.Px = this->Px - others.Px;
            temp.Py = this->Py - others.Py;
            temp.Pz = this->Pz - others.Pz;
            return(temp);
        }
    };

    //struct, 3D velocity
    struct v3DVelocity
    {
    public:
        double Vx;
        double Vy;
        double Vz;
    public:
        v3DVelocity():Vx(0), Vy(0), Vz(0){}
        v3DVelocity(double vx, double vy, double vz)
        {
            this->Vx = vx; this->Vy = vy; this->Vz = vz;
        }
    public:
        v3DVelocity operator+(const v3DVelocity &others)
        {
            v3DVelocity temp;
            temp.Vx = this->Vx + others.Vx;
            temp.Vy = this->Vy + others.Vy;
            temp.Vz = this->Vz + others.Vz;
            return(temp);
        }
        v3DVelocity operator-(const v3DVelocity &others)
        {
            v3DVelocity temp;
            temp.Vx = this->Vx - others.Vx;
            temp.Vy = this->Vy - others.Vy;
            temp.Vz = this->Vz - others.Vz;
            return(temp);
        }
    };

    //struct, 3D joint
    struct v3DJoint
    {
    public:
        double th1;
        double th2;
        double th3;
    public:
        v3DJoint():th1(0), th2(0), th3(0){}
        v3DJoint(double th1, double th2, double th3)
        {
            this->th1 = th1; this->th2 = th2; this->th3 = th3;
        }
    public:
        v3DJoint operator+(const v3DJoint &others)
        {
            v3DJoint temp;
            temp.th1 = this->th1 + others.th1;
            temp.th2 = this->th2 + others.th2;
            temp.th3 = this->th3 + others.th3;
            return(temp);
        }
        v3DJoint operator-(const v3DJoint &others)
        {
            v3DJoint temp;
            temp.th1 = this->th1 - others.th1;
            temp.th2 = this->th2 - others.th2;
            temp.th3 = this->th3 - others.th3;
            return(temp);
        }
    };

    //struct, 3D joint rate
    struct v3DJointRate
    {
    public:
        double vth1;
        double vth2;
        double vth3;
    public:
        v3DJointRate():vth1(0), vth2(0), vth3(0){}
        v3DJointRate(double vth1, double vth2, double vth3)
        {
            this->vth1 = vth1; this->vth2 = vth2; this->vth3 = vth3;
        }
    public:
        v3DJointRate operator+(const v3DJointRate &others)
        {
            v3DJointRate temp;
            temp.vth1 = this->vth1 + others.vth1;
            temp.vth2 = this->vth2 + others.vth2;
            temp.vth3 = this->vth3 + others.vth3;
            return(temp);
        }
        v3DJointRate operator-(const v3DJointRate &others)
        {
            v3DJointRate temp;
            temp.vth1 = this->vth1 - others.vth1;
            temp.vth2 = this->vth2 - others.vth2;
            temp.vth3 = this->vth3 - others.vth3;
            return(temp);
        }
    };


    //delta robot v4DPoint struct
    struct v4DPoint
    {
    public:
        v3DPoint Position;
        double YawAngle;
    public:
        v4DPoint():Position(0,0,0), YawAngle(0){}
        v4DPoint(double px, double py, double pz, double yawangle)
        {
            this->Position.Px = px;
            this->Position.Py = py;
            this->Position.Pz = pz;
            this->YawAngle = yawangle;
        }
        v4DPoint(v3DPoint point, double yawangle)
        {
            this->Position = point; this->YawAngle = yawangle;
        }
    public:
        v4DPoint operator+(const v4DPoint &others)
        {
            v4DPoint temp;
            temp.Position = this->Position + others.Position;
            temp.YawAngle = this->YawAngle + others.YawAngle;
            return(temp);
        }
        v4DPoint operator-(const v4DPoint &others)
        {
            v4DPoint temp;
            temp.Position = this->Position - others.Position;
            temp.YawAngle = this->YawAngle - others.YawAngle;
            return(temp);
        }
    };
    //robot v4DVelocity struct
    struct v4DVelocity
    {
    public:
        v3DVelocity Velocity;
        double YawAngleRate;
    public:
        v4DVelocity():Velocity(0,0,0), YawAngleRate(0){}
        v4DVelocity(double vx, double vy, double vz, double anglerate)
        {
            this->Velocity.Vx = vx;
            this->Velocity.Vy = vy;
            this->Velocity.Vz = vz;
            this->YawAngleRate = anglerate;
        }
        v4DVelocity(v3DVelocity velocity, double yawanglerate)
        {
            this->Velocity = velocity; this->YawAngleRate = yawanglerate;
        }
    public:
        v4DVelocity operator+(const v4DVelocity &others)
        {
            v4DVelocity temp;
            temp.Velocity = this->Velocity + others.Velocity;
            temp.YawAngleRate = this->YawAngleRate + others.YawAngleRate;
            return(temp);
        }
        v4DVelocity operator-(const v4DVelocity &others)
        {
            v4DVelocity temp;
            temp.Velocity = this->Velocity - others.Velocity;
            temp.YawAngleRate = this->YawAngleRate - others.YawAngleRate;
            return(temp);
        }
    };

    //delta robot v4DJoint struct
    struct v4DJoint
    {
    public:
        v3DJoint Joint;
        double YawAngle;
    public:
        v4DJoint():Joint(0,0,0), YawAngle(0){}
        v4DJoint(double th1, double th2, double th3, double yawangle)
        {
            this->Joint.th1 = th1;
            this->Joint.th2 = th2;
            this->Joint.th3 = th3;
            this->YawAngle = yawangle;
        }
        v4DJoint(v3DJoint joint, double yawangle)
        {
            this->Joint = joint; this->YawAngle = yawangle;
        }
    public:
        v4DJoint operator+(const v4DJoint &others)
        {
            v4DJoint temp;
            temp.Joint = this->Joint + others.Joint;
            temp.YawAngle = this->YawAngle + others.YawAngle;
            return(temp);
        }
        v4DJoint operator-(const v4DJoint &others)
        {
            v4DJoint temp;
            temp.Joint = this->Joint - others.Joint;
            temp.YawAngle = this->YawAngle - others.YawAngle;
            return(temp);
        }
    };

    //delta robot v4DJoint struct
    struct v4DJointRate
    {
    public:
        v3DJointRate JointRate;
        double YawAngleRate;
    public:
        v4DJointRate():JointRate(0,0,0), YawAngleRate(0){}
        v4DJointRate(double vth1, double vth2, double vth3, double anglerate)
        {
            this->JointRate.vth1 = vth1;
            this->JointRate.vth2 = vth2;
            this->JointRate.vth3 = vth3;
            this->YawAngleRate = anglerate;
        }
        v4DJointRate(v3DJointRate jointrate, double yawanglerate)
        {
            this->JointRate = jointrate; this->YawAngleRate = yawanglerate;
        }
    public:
        v4DJointRate operator+(const v4DJointRate &others)
        {
            v4DJointRate temp;
            temp.JointRate = this->JointRate + others.JointRate;
            temp.YawAngleRate = this->YawAngleRate + others.YawAngleRate;
            return(temp);
        }
        v4DJointRate operator-(const v4DJointRate &others)
        {
            v4DJointRate temp;
            temp.JointRate = this->JointRate - others.JointRate;
            temp.YawAngleRate = this->YawAngleRate - others.YawAngleRate;
            return(temp);
        }
    };

}

#endif //DELTAROBOT_DATATYPE_H