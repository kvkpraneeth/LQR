#include "lqr/lqr.h"

template<int numberOfInputs, int numberOfStates>
lqr<numberOfInputs, numberOfStates>::lqr()
{
    this->numberOfInputs_ = numberOfInputs;
    this->numberOfStates_ = numberOfStates;
}

template<int numberOfInputs, int numberOfStates>
void lqr<numberOfInputs, numberOfStates>::costFunc()
{
    cost += (System.X.transpose() * Q * System.X) + (System.U.transpose() * R * System.U);
}

