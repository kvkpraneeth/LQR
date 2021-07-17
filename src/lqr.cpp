#include "lqr/lqr.h"

typedef Eigen::MatrixXd matrix;

template<int numberOfInputs, int numberOfStates>
lqr<numberOfInputs, numberOfStates>::lqr(matrix A_, matrix B_, matrix Q_, matrix R_)
{
    
    this->numberOfInputs_ = numberOfInputs;
    this->numberOfStates_ = numberOfStates;

    System.A = A_;
    System.B = B_;
    Q = Q_;
    R = R_;

}

template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::setK()
{

    matrix Hamiltonian = matrix::Zero(2*numberOfStates, 2*numberOfInputs);

    Hamiltonian << System.A, -System.B * R.inverse() * System.B.transpose(), -Q, -System.A.transpose();

    Eigen::EigenSolver<matrix> EigenValues(Hamiltonian);

    Eigen::MatrixXcd EigenVector = Eigen::MatrixXcd::Zero(2*numberOfStates);

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
