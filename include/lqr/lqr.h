#ifndef LQR_H
#define LQR_H

#include "eigen3/Eigen/Core"
#include <iostream>
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

    Eigen::Matrix<double, numberOfInputs, 1> U;
    Eigen::Matrix<double, numberOfInputs, 1> X;

    Eigen::Matrix<double, numberOfStates, numberOfStates> A;
    Eigen::Matrix<double, numberOfStates, numberOfInputs> B;
    Eigen::Matrix<double, numberOfInputs, numberOfStates> K;

};

template<int numberOfInputs, int numberOfStates>
class lqr
{
    
    public:
        //Defaults.
            lqr(Eigen::Matrix<double, numberOfStates, numberOfStates>A_,
            Eigen::Matrix<double, numberOfStates, numberOfInputs>B_, 
            Eigen::Matrix<double, numberOfStates, numberOfStates>Q_,
            Eigen::Matrix<double, numberOfInputs, numberOfInputs>R_);

        //System init.
        struct LinearStateSpace<numberOfInputs, numberOfStates> System;

        //Cost Function Q and R Matrices.

            //Positive Definite.
        Eigen::Matrix<double, numberOfInputs, numberOfInputs> R;
            //Positive Semi Definite.
        Eigen::Matrix<double, numberOfStates, numberOfStates> Q;

        //Sets the Gain of the System.
        //Arimoto-Potter Algorithm @TakaHoribe/Riccati_Solver
        void setK();

        //Compute the Required Control Sequence
        void computeU();

};

template<int numberOfInputs, int numberOfStates>
lqr<numberOfInputs, numberOfStates>::lqr(Eigen::Matrix<double, numberOfStates, numberOfStates>A_,
                                        Eigen::Matrix<double, numberOfStates, numberOfInputs>B_, 
                                        Eigen::Matrix<double, numberOfStates, numberOfStates>Q_,
                                        Eigen::Matrix<double, numberOfInputs, numberOfInputs>R_)

{

    System.A = A_;
    System.B = B_;

    Q = Q_;
    R = R_;

    
}

template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::setK()
{
    Eigen::Matrix<double, 2*numberOfStates, 2*numberOfStates> Hamiltonian;

    Hamiltonian << System.A, -System.B * R.inverse() * System.B.transpose(), -Q, -System.A.transpose();

    Eigen::EigenSolver<Eigen::Matrix<double, 2*numberOfStates, 2*numberOfStates>> EigenValues(Hamiltonian);
    
    Eigen::MatrixXcd EigenVector = Eigen::MatrixXcd::Zero(2 * numberOfStates, numberOfStates);
    
    int j = 0;
    for (int i = 0; i < 2 * numberOfStates; ++i)
    {
        if (EigenValues.eigenvalues()[i].real() < 0.)
        {
            EigenVector.col(j) = EigenValues.eigenvectors().block(0, i, 2 * numberOfStates, 1);
            ++j;
        }
    }

    Eigen::MatrixXcd t1, t2;

    t1 = EigenVector.block(0,0, numberOfStates, numberOfStates);
    t2 = EigenVector.block(numberOfStates, 0, numberOfStates,numberOfStates);

    auto P = (t2*t1.inverse()).real();

    System.K = R.inverse() * System.B.transpose() * P;

}


template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::computeU()
{
    System.U = - System.K * System.X;
}

#endif
