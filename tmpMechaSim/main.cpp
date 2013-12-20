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

    return 0;
}

