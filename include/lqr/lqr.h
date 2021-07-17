#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <memory>

template<int numberOfInputs, int numberOfOutputs, int numberOfStates>
struct LinearStateSpace
{

    /*
        Linear State Space representation: 

        X' = AX + BU; Y = KX;

        @X (numberOfStates, 1): Vector
        @Y (numberOfOutputs, 1): Vector
        @U (numberOfInputs, 1): Vector

        @A weights/penalties of X
        @B weights/penalties of U
        @K System Gain for Full State Feedback Controller
    */

    Eigen::Matrix<double, numberOfInputs, 1> U;  
    Eigen::Matrix<double, numberOfOutputs, 1> Y;
    Eigen::Matrix<double, numberOfStates, 1> X;

    Eigen::Matrix<double, numberOfOutputs, numberOfStates> A;  
    Eigen::Matrix<double, numberOfOutputs, numberOfInputs> B;  
    Eigen::Matrix<double, numberOfOutputs, numberOfStates> K;  

};

template<int numberOfInputs, int numberOfOutputs, int numberOfStates>
class lqr
{

    //Defaults.
    lqr();

    private:
        
        //Plant Information.
        int numberOfInputs_, numberOfOutputs_, numberOfStates_;

        //System init.
        struct LinearStateSpace<numberOfInputs, numberOfOutputs, numberOfStates> System;

        //Cost Function Q and R Matrices.

        //Positive Definite.
        Eigen::Matrix<double, numberOfInputs, numberOfInputs> R;
        //Positive Semi Definite.
        Eigen::Matrix<double, numberOfStates, numberOfStates> Q;

        //Initial Cost.
        double cost = 0;

    public:

        /*
        
            Cost Function Algorithm:

                J += X.T @ Q @ X + U.T @ R @ U

        */
        void costFunc();

        Eigen::Matrix<double, numberOfInputs, numberOfStates> result();

};

#endif
