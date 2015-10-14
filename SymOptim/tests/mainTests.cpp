#include <iostream>
#include <stdexcept>
#include <cassert>
#include "SymOptim/src/GradientDescent.hpp"
#include "SymOptim/src/PenaltyMethod.hpp"
#include "Symbolic/src/Constant.hpp"

using namespace std;
using namespace Leph::SymOptim;

int main()
{
    //Gradient Descent
    GradientDescent<double>::VariableContainer variables;
    variables.push("x", 
        GradientDescent<double>::Symbol::create("x"));
    variables.push("y", 
        GradientDescent<double>::Symbol::create("y"));

    GradientDescent<double>::TermPtr targetConvex = 
        Leph::Symbolic::Add<double>::create(
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[0], variables[0]),
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[1], variables[1]));

    GradientDescent<double>::TermPtr targetSaddle = 
        Leph::Symbolic::Sub<double>::create(
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[0], variables[0]),
            Leph::Symbolic::Mult<double,double,double>::
                create(variables[1], variables[1]));

    GradientDescent<double> descentConvex(targetConvex, variables);
    descentConvex.state("x") = 10.0;
    descentConvex.state("y") = 5.0;
    descentConvex.runOptimization(false);
    assert(descentConvex.state("x") > -0.0001 && 
        descentConvex.state("x") < 0.0001);
    assert(descentConvex.state("y") > -0.0001 && 
        descentConvex.state("y") < 0.0001);
    
    GradientDescent<double> descentSaddle(targetSaddle, variables);
    descentSaddle.state("x") = 10.0;
    descentSaddle.state("y") = 5.0;
    try {
        descentSaddle.runOptimization();
        assert(false);
    } catch (const std::runtime_error& error) {
        assert(true);
    }

    //Penalty Method
    PenaltyMethod<double> penalty(targetConvex, variables);
    PenaltyMethod<double>::TermPtr constraint =
        Leph::Symbolic::Sub<double>::create(
            variables["x"], 
            Leph::Symbolic::Constant<double>::create(2.0));
    penalty.addEqualityConstraint(constraint);
    penalty.state("x") = 1.0;
    penalty.state("y") = 1.0;
    penalty.runOptimization();
    //cout << "Gradient Iteration: " << penalty.getDescentCount() << endl;
    //cout << "Penalty Iteration: " << penalty.getPenaltyCount() << endl;
    assert(penalty.state("x") > 1.99 && 
        penalty.state("x") < 2.01);
    assert(penalty.state("y") > -0.0001 && 
        penalty.state("y") < 0.0001);

    return 0;
}

