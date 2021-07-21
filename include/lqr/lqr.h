#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory>
#include "eigen3/Eigen/Eigenvalues"
#include <vector>

struct LinearStateSpace
{

    /*
        Linear State Space representation: 

        X' = AX + BU;
        U = -KX;

        Dimensions:
        
            @X (numberOfStates, 1): Vector
            @U (numberOfInputs, 1): Vector
            @X' = X.size() : Vector
                                        
            @A weights/penalties of X: (numberOfStates, numberOfStates): Vector
            @B weights/penalties of U: (numberOfStates, numberOfInputs): Vector
            @K System Gain for Full State Feedback Controller: (numberOfInputs, numberOfStates): Vector
    */
   

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;

    Eigen::MatrixXd K;

};

typedef struct LinearStateSpace LinearStateSpace;

class lqr
{

    public:

        // Defualts.
        
        lqr(LinearStateSpace& System_, std::vector<double> Q_, std::vector<double> R_);

    private:

        /*
            @Q is the Error that the System state has. 

                Q is a Positive Semi Definitve Matrix. 
                Q.size() = (numberOfStates, numberOfStates)

            @R is the Effort to put on the System.
                
                R is a Positive Definitve Matrix.
                R.size() = (numberOfInputs, numberOfInputs)

            J += x.T @ Q @ x + u.T @ R @ u
            
            where J is always positive and is the cost function to minimize.
        */
        
        Eigen::MatrixXd A, B, Q, R;

        int numberOfStates, numberOfInputs;

    public:

        // K is the Gain matrix of the System.
        void setK(LinearStateSpace &System_);

};

#endif
