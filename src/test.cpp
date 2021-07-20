#include "lqr/lqr.h"

int main()
{
    
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 1, 1> R;

    A << 0, 1, 0, -1.0/5.0;

    B << 0, 1;

    Q << 1, 0, 0, 1;

    R << 0.01;
    
    std::shared_ptr<lqr<1,2>> BlockSystemController = std::make_shared<lqr<1,2>>(A,B,Q,R);

    BlockSystemController->setK();

    std::cout << BlockSystemController->System.K << std::endl;

    // Solution can be verified from https://www.youtube.com/watch?v=wEevt2a4SKI&t=3656s

    return 0;
}
