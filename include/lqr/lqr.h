#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>

template<int numberOfInputs, int numberOfStates>
struct LinearStateSpace
{

    /*
        Linear State Space representation: 

        X' = AX + BU;
        U = -KX;

        Dimensions:
        
            @X (numberOfStates, 1): Vector
            @U (numberOfInputs, 1): Vector
            @X' = X : Vector
                                        
            @A weights/penalties of X: (numberOfStates, numberOfStates): Vector
            @B weights/penalties of U: (numberOfStates, numberOfInputs): Vector
            @K System Gain for Full State Feedback Controller: (numberOfInputs, numberOfStates): Vector
    

    */

    Eigen::MatrixXd U = Eigen::MatrixXd::Identity(numberOfInputs, 1);
    Eigen::MatrixXd X = Eigen::MatrixXd::Identity(numberOfStates, 1);

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(numberOfStates, numberOfStates);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(numberOfStates, numberOfInputs);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(numberOfInputs, numberOfStates);

};

template<int numberOfInputs, int numberOfStates>
class lqr
{

    //Defaults.
    lqr();

    private:
        
        //Plant Information.
        int numberOfInputs_, numberOfStates_;

    public:

        //System init.
        struct LinearStateSpace<numberOfInputs, numberOfStates> System;

        //Cost Function Q and R Matrices.

        //Positive Definite.
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(numberOfInputs, numberOfInputs);
        //Positive Semi Definite.
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(numberOfStates, numberOfStates);

    private:

        //Initial Cost.
        double cost = 0;

    public:

        /*
        
            Cost Function Algorithm:

                J += X.T @ Q @ X + U.T @ R @ U

        */
        
        void costFunc();

        Eigen::MatrixXd result();

};

#endif
