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

    Eigen::Matrix<float, numberOfInputs, 1> U;
    Eigen::Matrix<float, numberOfInputs, 1> X;

    Eigen::Matrix<float, numberOfStates, numberOfStates> A;
    Eigen::Matrix<float, numberOfStates, numberOfInputs> B;
    Eigen::Matrix<float, numberOfInputs, numberOfStates> K;

};

template<int numberOfInputs, int numberOfStates>
class lqr
{
    
    public:
        //Defaults.
            lqr(Eigen::Matrix<float, numberOfStates, numberOfStates>A_,
            Eigen::Matrix<float, numberOfStates, numberOfInputs>B_, 
            Eigen::Matrix<float, numberOfStates, numberOfStates>Q_,
            Eigen::Matrix<float, numberOfInputs, numberOfInputs>R_);

        //System init.
        struct LinearStateSpace<numberOfInputs, numberOfStates> System;

        //Cost Function Q and R Matrices.

            //Positive Definite.
        Eigen::Matrix<float, numberOfInputs, numberOfInputs> R;
            //Positive Semi Definite.
        Eigen::Matrix<float, numberOfStates, numberOfStates> Q;

        //Sets the Gain of the System.
        //Arimoto-Potter Algorithm @TakaHoribe/Riccati_Solver
        void setK();

        //Compute the Required Control Sequence
        void computeU();

        
};

template<int numberOfInputs, int numberOfStates>
lqr<numberOfInputs, numberOfStates>::lqr(Eigen::Matrix<float, numberOfStates, numberOfStates>A_,
                                        Eigen::Matrix<float, numberOfStates, numberOfInputs>B_, 
                                        Eigen::Matrix<float, numberOfStates, numberOfStates>Q_,
                                        Eigen::Matrix<float, numberOfInputs, numberOfInputs>R_)

{

    System.A = A_;
    System.B = B_;

    Q = Q_;
    R = R_;

    
}

template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::setK()
{

    Eigen::Matrix<float, 2*numberOfStates, 2*numberOfStates> Hamiltonian;

    Hamiltonian << System.A, -System.B * R.inverse() * System.B.transpose(), -Q, -System.A.transpose();

    Eigen::EigenSolver<Eigen::Matrix<float, 2*numberOfStates, 2*numberOfStates>> EigenValues(Hamiltonian);

    Eigen::Matrix<float, 2*numberOfStates, 1> EigenVector;


    int j = 0;
    for(int i = 0; i < 2*numberOfStates; ++i)
    {
        if(EigenValues.eigenvalues()[i].real() < 0.)
        {
            EigenVector.col(j) = EigenValues.eigenvectors().block(0, i, 2*numberOfStates, 1);
            ++j;
        }
    }

    System.K = (EigenVector.block(numberOfStates, 0, numberOfStates, numberOfStates) 
                    * EigenVector.block(0,0,numberOfStates,numberOfStates).inverse()).real();

}


template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::computeU()
{
    System.U = - System.K * System.X;
}

#endif
