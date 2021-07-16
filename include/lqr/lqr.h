#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/src/Core/Matrix.h>

template<int numberOfInputs, int numberOfOutputs, int numberOfStates>
struct LinearStateSpace
{

    /*
        Linear State Space representation: 

        X' = AX + BU; Y = KX;

        X: numberOfStateVariables_ * 1 Vector
        Y: numberOfOutputs_ * 1 Vector
        U: numberOfInputs_ * 1 Vector

        A: weights/penalties of X
        B: weights/penalties of U
        K: weights/penalties of X with respect to the output equation.
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

    //Default Constructor.
    lqr();

    private:
        
        //Plant Information.
        int numberOfInputs_, numberOfOutputs_, numberOfStates_;

        //System init.
        LinearStateSpace<numberOfInputs, numberOfOutputs, numberOfStates>* System;

        //Cost Function Q and R Matrices.
            

};

#endif
