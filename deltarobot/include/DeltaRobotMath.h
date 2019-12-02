#ifndef DELTAROBOT_MATH_H
#define DELTAROBOT_MATH_H

#include <cmath>


using namespace std;


namespace DeltaRobot
{
    //calculate the matrix inverse
    bool MatrixInverse(double *iM, double *oM, int N);

}



#endif //DELTAROBOT_MATH_H