#include "DeltaRobotMath.h"


using namespace std;


namespace DeltaRobot
{
    /**
     * @brief delta robot forward singular kinematics
     * @param iM -> <double *> the input matrix address
     * @param oM -> <double *> the output matrix address
     * @param N ->  <int> matrix size
     * @return return true is success, false is failed
     * @warning none
    **/
    bool MatrixInverse(double *iM, double *oM, int N)
    {
        double a[10][10], b[10][20], t;
        int i, j, m;

        //limit the matrix size
        if(N>10)
        {
            return(false);
        }
        //set value
        for(i=0; i<N; i++)
        {
            for(j=0; j<N; j++)
            {
                a[i][j] = *(iM + i*N + j);
                b[i][j] = a[i][j];
            }
        }
        //calculate the argumation matrix
        for(i=0; i<N; i++)
            for(j=N; j<2*N; j++)
                b[i][j] = 0;
        for(i=0; i<N; i++)
            b[i][N+i] = 1;
        for(m=0; m<N; m++)
        {
            t = b[m][m];
            i = m;
            while(b[m][m] == 0)
            {
                b[m][m] = b[i+1][m];
                i++;
            }
            if(i > m)
            {
                b[i][m] = t;
                for(j=0; j<m; j++)
                {
                    t = b[m][j];
                    b[m][j] = b[i][j];
                    b[i][j] = t;
                }
                for(j=m+1; j<2*N; j++)
                {
                    t = b[m][j];
                    b[m][j] = b[i][j];
                    b[i][j] = t;
                }
            }
            for(i=m+1; i<N; i++)
                for(j=2*N-1; j>=m; j--)
                    b[i][j] -= b[i][m]*b[m][j]/b[m][m];
            for(j=2*N-1; j>=m; j--)
                b[m][j] /= b[m][m];
        }
        m = N-1;
        while(m>0)
        {
            for(i=0; i<m; i++)
                for(j=2*N-1; j>=m; j--)
                    b[i][j] -= b[i][m]*b[m][j];
            m--;
        }
        for(i=0; i<N; i++)
            for(j=0; j<N; j++)
                *(oM + i*N + j) = b[i][N+j];
        
        //return
        return(true);
    }
    

}