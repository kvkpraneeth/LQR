#include "lqr/lqr.h"
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <iostream>

int main()
{
    
    LinearStateSpace System;

    System.A = Eigen::Matrix<double, 2, 2>();
    System.B = Eigen::Matrix<double, 2, 1>();

    System.A << 0, 1, 0, -1.0/5.0;

    System.B << 0, 1;

    // Row Major Ordering
    std::vector<double> Q{1,0,0,1}, R{0.01};

    lqr x(System, Q, R);

    // Solution can be verified from https://www.youtube.com/watch?v=wEevt2a4SKI&t=3656s
    
    std::cout << System.K;

    return 0;
}
