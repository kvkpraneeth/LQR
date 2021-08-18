#include "lqr/lqr.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Eigenvalues/ComplexEigenSolver.h>
#include <eigen3/Eigen/src/Eigenvalues/EigenSolver.h>

lqr::lqr(LinearStateSpace& System_, std::vector<double> Q_, std::vector<double> R_)
{

    this->numberOfInputs = int(std::pow(R_.size(), 0.5));
    this->numberOfStates = int(std::pow(Q_.size(), 0.5));

    this->A = Eigen::MatrixXd(numberOfStates, numberOfStates);

    A = System_.A;

	std::cout << "A" << std::endl;
	
	std::cout << A << std::endl;

	std::cout << "======" << std::endl;

    this->B = Eigen::MatrixXd(numberOfStates, numberOfInputs);

    B = System_.B;
	
	std::cout << "B" << std::endl;
	
	std::cout << B << std::endl;
	
	std::cout << "======" << std::endl;

    this->Q = Eigen::MatrixXd(numberOfStates, numberOfStates);

    for(int x = 0; x < numberOfStates; x++)
    {
        for(int y = 0; y < numberOfStates; y++)
        {
            Q(x,y) = Q_[x*numberOfStates + y];
        }
        
    }

	std::cout << "Q" << std::endl;
	
	std::cout << Q << std::endl;
	
	std::cout << "======" << std::endl;
    
	this->R = Eigen::MatrixXd(numberOfInputs, numberOfInputs);

    for(int x = 0; x < numberOfInputs; x++)
    {
        for(int y = 0; y < numberOfInputs; y++)
        {
            R(x,y) = R_[numberOfStates*x + y];
        }
    }

	std::cout << "R" << std::endl;
	
	std::cout << R << std::endl;
	
	std::cout << "======" << std::endl;
    
	this->setK(System_);

}

void lqr::setK(LinearStateSpace &System_)
{

    Eigen::MatrixXd Hamiltonian(2*numberOfStates, 2*numberOfStates);

    Hamiltonian << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    Eigen::EigenSolver<Eigen::MatrixXd> EigenValues(Hamiltonian);

	std::cout << "Eigenvalues" << std::endl;
	
	std::cout << EigenValues.eigenvalues() << std::endl;
	
	std::cout << "======" << std::endl;
    
	Eigen::MatrixXcd EigenVector = Eigen::MatrixXcd::Zero(2*numberOfStates, numberOfStates);

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
    
    auto K = R.inverse() * B.transpose() * P;

    System_.K = Eigen::MatrixXd(numberOfInputs, numberOfStates);

    System_.K = K;

	std::cout << "K" << std::endl;
	
	std::cout << K << std::endl;
	
	std::cout << "======" << std::endl;
}
