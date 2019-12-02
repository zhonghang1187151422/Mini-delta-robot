#include <cmath>
#include "DeltaRobotKinematics.h"
#include "DeltaRobotMath.h"


using namespace std;


namespace DeltaRobot
{
    //default construct function
    DeltaRobotKinematics::DeltaRobotKinematics():alpha1(0), alpha2(2*M_PI/3), alpha3(4*M_PI/3), R(DEFAULT_R), r(DEFAULT_r), L1(DEFAULT_L1), L2(DEFAULT_L2), H(DEFAULT_H) {}

    //construct function
    DeltaRobotKinematics::DeltaRobotKinematics(double iR, double ir, double iL1, double iL2, double iH)
    {
        this->alpha1 = 0;
        this->alpha2 = 2*M_PI/3;
        this->alpha3 = 4*M_PI/3;
        this->H = iH;
        this->R = iR;
        this->r = ir;
        this->L1 = iL1;
        this->L2 = iL2;
    }

    //destructor function
    DeltaRobotKinematics::~DeltaRobotKinematics() {}

    /**
     * @brief delta robot forward kinematics
     * @param th -> <v3DJoint> the joint variables
     * @param pos -> <v3DPoint&> reference of the end position
     * @return return true is success, false is failed
     * @warning none
    **/
    bool DeltaRobotKinematics::ForwardKinematics(v3DJoint th, v3DPoint& pos)
    {
        //check the input joint in limit
        if((th.th1 < JOINT_MIN) || (th.th1 > JOINT_MAX) || (th.th2 < JOINT_MIN) || (th.th2 > JOINT_MAX) || (th.th3 < JOINT_MIN) || (th.th3 > JOINT_MAX))
        {
            return(false);
        }
        //calculate B matrix
        double b11 = (r - R - L1*std::cos(th.th1)) * std::cos(alpha1);
        double b21 = (r - R - L1*std::cos(th.th2)) * std::cos(alpha2);
        double b31 = (r - R - L1*std::cos(th.th3)) * std::cos(alpha3);
        double b12 = (r - R - L1*std::cos(th.th1)) * std::sin(alpha1);
        double b22 = (r - R - L1*std::cos(th.th2)) * std::sin(alpha2);
        double b32 = (r - R - L1*std::cos(th.th3)) * std::sin(alpha3);
        double b13 = L1*std::sin(th.th1);
        double b23 = L1*std::sin(th.th2);
        double b33 = L1*std::sin(th.th3);
        //calculate M
        double M1 = b11*b11 + b12*b12 + b13*b13 - b21*b21 - b22*b22 - b23*b23;
        double M2 = b11*b11 + b12*b12 + b13*b13 - b31*b31 - b32*b32 - b33*b33;
        //calculate W
        double W1 = ((b32-b12)*M1-(b22-b12)*M2) / (2*(b21-b11)*(b32-b12)-2*(b31-b11)*(b22-b12));
        double W2 = ((b23-b13)*(b32-b12)-(b33-b13)*(b22-b12)) / ((b21-b11)*(b32-b12)-(b31-b11)*(b22-b12));
        double W3 = ((b31-b11)*M1-(b21-b11)*M2) / (2*(b22-b12)*(b31-b11)-2*(b32-b12)*(b21-b11));
        double W4 = ((b23-b13)*(b31-b11)-(b33-b13)*(b21-b11)) / ((b22-b12)*(b31-b11)-(b32-b12)*(b21-b11));
        //calculate T
        double T1 = W2*W2 + W4*W4 + 1;
        double T2 = W1*W2 + W3*W4 + b11*W2 + b12*W4 - b13;
        double T3 = W1*W1 + W3*W3 + 2*b11*W1 + 2*b12*W3 + b11*b11 + b12*b12 + b13*b13 - L2*L2;
        //calculate Pz
        double Pz1 = (T2 + std::sqrt(T2*T2 - T1*T3)) / T1;
        double Pz2 = (T2 - std::sqrt(T2*T2 - T1*T3)) / T1;
        if(Pz1 < 0)
        {
            pos.Pz = Pz1;
            
        }else
        {
            pos.Pz = Pz2;
        }
        //calculate Px and Py
        pos.Px = W1 - W2*pos.Pz;
        pos.Py = W3 - W4*pos.Pz;
        //convert to the top plane coordinate
        pos.Pz = -pos.Pz + H;
        //return
        return(true);
    }

    /**
     * @brief delta robot inverse kinematics
     * @param pos -> <v3DPoint> the end position
     * @param th -> <v3DJoint&> reference of the joint angles
     * @return return true is success, false is failed
     * @warning none
    **/
    bool DeltaRobotKinematics::InverseKinematics(v3DPoint pos, v3DJoint& th)
    {
        //variables declare
        double I1, I2, I3;
        double J1, J2, J3;
        double K1, K2, K3;

        //convert to the top plane coordinate
        pos.Pz = -(pos.Pz - H);
        //calculate I
        I1 = 2*L1*((pos.Px + r*std::cos(alpha1) - R*std::cos(alpha1))*std::cos(alpha1) + (pos.Py + r*std::sin(alpha1) - R*std::sin(alpha1))*std::sin(alpha1));
        I2 = 2*L1*((pos.Px + r*std::cos(alpha2) - R*std::cos(alpha2))*std::cos(alpha2) + (pos.Py + r*std::sin(alpha2) - R*std::sin(alpha2))*std::sin(alpha2));
        I3 = 2*L1*((pos.Px + r*std::cos(alpha3) - R*std::cos(alpha3))*std::cos(alpha3) + (pos.Py + r*std::sin(alpha3) - R*std::sin(alpha3))*std::sin(alpha3));
        //calculate J
        J1 = 2*L1*pos.Pz;
        J2 = J1;
        J3 = J1;
        //calculate K
        K1 = pow(pos.Px+r*cos(alpha1)-R*cos(alpha1), 2) + pow(pos.Py+r*sin(alpha1)-R*sin(alpha1), 2) + pow(pos.Pz, 2) + pow(L1, 2) - pow(L2, 2);
        K2 = pow(pos.Px+r*cos(alpha2)-R*cos(alpha2), 2) + pow(pos.Py+r*sin(alpha2)-R*sin(alpha2), 2) + pow(pos.Pz, 2) + pow(L1, 2) - pow(L2, 2);
        K3 = pow(pos.Px+r*cos(alpha3)-R*cos(alpha3), 2) + pow(pos.Py+r*sin(alpha3)-R*sin(alpha3), 2) + pow(pos.Pz, 2) + pow(L1, 2) - pow(L2, 2);
        //check the point if or not in workspace
        if(((J1*J1+I1*I1-K1*K1) < 0) || ((J2*J2+I2*I2-K2*K2) < 0) || ((J3*J3+I3*I3-K3*K3) < 0))
        {
            //out the workspace
            return(false);
        }else
        {
            th.th1 = 2*atan((-J1-sqrt(J1*J1+I1*I1-K1*K1))/(K1+I1));
            th.th2 = 2*atan((-J2-sqrt(J2*J2+I2*I2-K2*K2))/(K2+I2));
            th.th3 = 2*atan((-J3-sqrt(J3*J3+I3*I3-K3*K3))/(K3+I3));
            //check the joint variables if or not in limit
            if((th.th1 < JOINT_MIN) || (th.th1 > JOINT_MAX) || (th.th2 < JOINT_MIN) || (th.th2 > JOINT_MAX) || (th.th3 < JOINT_MIN) || (th.th3 > JOINT_MAX))
            {
                return(false);
            }
        }
        //return
        return(true);
    }

    /**
     * @brief delta robot forward singular kinematics
     * @param th -> <v3DJoint> the joint angles
     * @param vth -> <v3DJointRate> the joint rate
     * @param th -> <v3DVelocity&> the reference of the velocity
     * @return return true is success, false is failed
     * @warning none
    **/
    bool DeltaRobotKinematics::ForwardSingularKinematics(v3DJoint th, v3DJointRate vth, v3DVelocity& vel)
    {
        double a11, a12, a13, a21, a22, a23, a31, a32, a33;
        double m1, m2, m3;
        double J[3][3], InvJ[3][3];
        //forward kinematics
        v3DPoint pos;
        bool ret = ForwardKinematics(th, pos);
        //check
        if(ret)
        {
            // A*d_P = B*d_theta
            // A = [a11 a12 a13;a21 a22 a23;a31 a32 a33];
            a11 = pos.Px + r*cos(alpha1) - R*cos(alpha1) - L1*cos(alpha1)*cos(th.th1);
            a21 = pos.Px + r*cos(alpha2) - R*cos(alpha2) - L1*cos(alpha2)*cos(th.th2);
            a31 = pos.Px + r*cos(alpha3) - R*cos(alpha3) - L1*cos(alpha3)*cos(th.th3);
            a12 = pos.Py + r*sin(alpha1) - R*sin(alpha1) - L1*sin(alpha1)*cos(th.th1);
            a22 = pos.Py + r*sin(alpha2) - R*sin(alpha2) - L1*sin(alpha2)*cos(th.th2);
            a32 = pos.Py + r*sin(alpha3) - R*sin(alpha3) - L1*sin(alpha3)*cos(th.th3);
            a13 = pos.Pz + L1*sin(th.th1);
            a23 = pos.Pz + L1*sin(th.th2);
            a33 = pos.Pz + L1*sin(th.th3);
            // B = [m1 0 0;0 m2 0;0 0 m3];
            m1 = -a11*L1*cos(alpha1)*sin(th.th1) - a12*L1*sin(alpha1)*sin(th.th1) - a13*L1*cos(th.th1);
            m2 = -a21*L1*cos(alpha2)*sin(th.th2) - a22*L1*sin(alpha2)*sin(th.th2) - a23*L1*cos(th.th2);
            m3 = -a31*L1*cos(alpha3)*sin(th.th3) - a32*L1*sin(alpha3)*sin(th.th3) - a33*L1*cos(th.th3);
            // J= inv(B)*A
            J[0][0]=a11/m1; J[0][1]=a12/m1; J[0][2]=a13/m1;
            J[1][0]=a21/m2; J[1][1]=a22/m2; J[1][2]=a23/m2;
            J[2][0]=a31/m3; J[2][1]=a32/m3; J[2][2]=a33/m3;
            // InvJ = inv(J)
            if(DeltaRobot::MatrixInverse(J[0], InvJ[0], 3))
            {
                vel.Vx = InvJ[0][0]*vth.vth1 + InvJ[0][1]*vth.vth2 + InvJ[0][2]*vth.vth3;
                vel.Vy = InvJ[1][0]*vth.vth1 + InvJ[1][1]*vth.vth2 + InvJ[1][2]*vth.vth3;
                vel.Vz = InvJ[2][0]*vth.vth1 + InvJ[2][1]*vth.vth2 + InvJ[2][2]*vth.vth3;
            }else
            {
                return(false);
            }
        }else
        {
            return(false);
        }
        //return
        return(true);
    }

    /**
     * @brief delta robot inverse singular kinematics
     * @param th -> <v3DJoint> the joint angles
     * @param vel -> <v3DVelocity> the end effector velocity
     * @param vth -> <v3DJointRate&> the reference of the joint rate
     * @return return true is success, false is failed
     * @warning none
    **/
    bool DeltaRobotKinematics::InverseSingularKinematics(v3DJoint th, v3DVelocity vel, v3DJointRate& vth)
    {
        double a11, a12, a13, a21, a22, a23, a31, a32, a33;
        double m1, m2, m3;
        double J[3][3];
        //forward kinematics
        v3DPoint pos;
        bool ret = ForwardKinematics(th, pos);
        //check
        if(ret)
        {
            // A*d_P = B*d_theta
            // A
            a11 = pos.Px + r*cos(alpha1) - R*cos(alpha1) - L1*cos(alpha1)*cos(th.th1);
            a21 = pos.Px + r*cos(alpha2) - R*cos(alpha2) - L1*cos(alpha2)*cos(th.th2);
            a31 = pos.Px + r*cos(alpha3) - R*cos(alpha3) - L1*cos(alpha3)*cos(th.th3);
            a12 = pos.Py + r*sin(alpha1) - R*sin(alpha1) - L1*sin(alpha1)*cos(th.th1);
            a22 = pos.Py + r*sin(alpha2) - R*sin(alpha2) - L1*sin(alpha2)*cos(th.th2);
            a32 = pos.Py + r*sin(alpha3) - R*sin(alpha3) - L1*sin(alpha3)*cos(th.th3);
            a13 = pos.Pz + L1*sin(th.th1);
            a23 = pos.Pz + L1*sin(th.th2);
            a33 = pos.Pz + L1*sin(th.th3);
            //A = [a11 a12 a13;a21 a22 a23;a31 a32 a33];
            // B
            m1 = -a11*L1*cos(alpha1)*sin(th.th1) - a12*L1*sin(alpha1)*sin(th.th1) - a13*L1*cos(th.th1);
            m2 = -a21*L1*cos(alpha2)*sin(th.th2) - a22*L1*sin(alpha2)*sin(th.th2) - a23*L1*cos(th.th2);
            m3 = -a31*L1*cos(alpha3)*sin(th.th3) - a32*L1*sin(alpha3)*sin(th.th3) - a33*L1*cos(th.th3);
            //B = [m1 0 0;0 m2 0;0 0 m3];
            J[0][0]=a11/m1; J[0][1]=a12/m1; J[0][2]=a13/m1;
            J[1][0]=a21/m2; J[1][1]=a22/m2; J[1][2]=a23/m2;
            J[2][0]=a31/m3; J[2][1]=a32/m3; J[2][2]=a33/m3;
            //calculate
            vth.vth1 = J[0][0]*vel.Vx + J[0][1]*vel.Vy + J[0][2]*vel.Vz;
            vth.vth2 = J[1][0]*vel.Vx + J[1][1]*vel.Vy + J[1][2]*vel.Vz;
            vth.vth3 = J[2][0]*vel.Vx + J[2][1]*vel.Vy + J[2][2]*vel.Vz;
        }else
        {
            return(false);
        }
        //return
        return(true);
    }


}