#include <iostream>
#include <cassert>
#include "SymOptim/src/GradientDescent.hpp"

using namespace std;
using namespace Leph::SymOptim;

int main()
{
    GradientDescent<double>::VariableContainer variables;
    variables.push("x", 
        GradientDescent<double>::Symbol::create("x"));
    variables.push("y", 
        GradientDescent<double>::Symbol::create("y"));

    GradientDescent<double>::TermPtr target = 
        Leph::Symbolic::Add<double>::create(
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[0], variables[0]),
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[1], variables[1]));

    GradientDescent<double> gradientDecent(target, variables);
    gradientDecent.state("x") = 10.0;
    gradientDecent.state("y") = 5.0;
    gradientDecent.runOptimization();
    assert(gradientDecent.state("x") > -0.0001 && 
        gradientDecent.state("x") < 0.0001);
    assert(gradientDecent.state("y") > -0.0001 && 
        gradientDecent.state("y") < 0.0001);

    return 0;
}

