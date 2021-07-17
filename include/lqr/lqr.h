#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <memory>
#include "eigen3/Eigen/Eigenvalues"

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
    lqr(Eigen::MatrixXd A_, Eigen::MatrixXd B_, Eigen::MatrixXd Q_, Eigen::MatrixXd R_);

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

        //Sets the Gain of the System.
        //Arimoto-Potter Algorithm @TakaHoribe/Riccati_Solver
        void setK();

        //Compute the Required Control Sequence
        void computeU();

};

#endif
