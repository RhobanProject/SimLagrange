#include <iostream>
#include "Model.hpp"
#include "FlyModel.hpp"

int main()
{
    Leph::FlyModel::ParameterContainer parameters;
    parameters["mass"] = 1.0;
    parameters["g"] = 9.81;
    parameters["l"] = 1.0;

    Leph::FlyModel flyModel(parameters);
    flyModel.initialization();
    flyModel.test();

    Leph::FlyModel::EigenVector state(2, 1);
    state(0) = 1.5;
    state(1) = 0.0;
    Leph::FlyModel::EigenVector torque = Leph::FlyModel::EigenVector::Zero(1, 1);
    torque(0) = 6.0;

    for (int i=0;i<100;i++) {
        state = flyModel.runStep(state, torque, 0.1);
        std::cout << state(0) << std::endl;
    }

    return 0;
}

