#ifndef LEPH_SYMOPTIM_PENALTYMETHOD_HPP
#define LEPH_SYMOPTIM_PENALTYMETHOD_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include "VectorMap/src/VectorMap.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Constant.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/terms.h"
#include "SymOptim/src/GradientDescent.hpp"

namespace Leph {
namespace SymOptim {

/**
 * PenaltyMethod
 *
 * Use a Gradient Descent for
 * searching the target function minimum
 * point under a set of given equalities 
 * constraints
 * Use constrainted optimization Penaly Method
 */
template <class scalar>
class PenaltyMethod
{
    public:

        /**
         * Typedef of GradientDescent
         */
        typedef typename GradientDescent<scalar>::TermPtr 
            TermPtr;
        typedef typename GradientDescent<scalar>::VariableContainer 
            VariableContainer;
        typedef typename GradientDescent<scalar>::ValueContainer 
            ValueContainer;

        /**
         * Configuration parameters
         */
        scalar INITIAL_EQUALITY_COEF;
        scalar EQUALITY_COEF_GAIN;
        scalar EQUALITY_EPSILON;
        unsigned int MAX_LOOP; 

        /**
         * Initialization with Symbolic target expression
         * to minimized and a container of Symbolic variables
         */
        PenaltyMethod(TermPtr targetFunction, 
            const VariableContainer& variables) :
            INITIAL_EQUALITY_COEF(0.001),
            EQUALITY_COEF_GAIN(10.0),
            EQUALITY_EPSILON(0.001),
            MAX_LOOP(100),
            _targetFunction(targetFunction),
            _variables(variables),
            _equalityConstraints(),
            _equalityCoefficients(),
            _countGradientIteration(0),
            _countPenaltyIteration(0)
        {
            //Initialize state value
            for (size_t i=0;i<_variables.size();i++) {
                _values.push(_variables.getKey(i), scalar(0.0));
            }
        }

        /**
         * Access to current point value of 
         * given name variable
         */
        inline const scalar& state(const std::string& name) const
        {
            return _values[name];
        }
        inline scalar& state(const std::string& name)
        {
            return _values[name];
        }

        /**
         * Return the number of gradient descent and 
         * penalty iterations
         */
        inline unsigned int getDescentCount() const
        {
            return _countGradientIteration;
        }
        inline unsigned int getPenaltyCount() const
        {
            return _countPenaltyIteration;
        }

        /**
         * Add a Symbolic equality constraint
         * The constraint is meet when the expression
         * equals zero
         */
        inline void addEqualityConstraint(TermPtr constraint)
        {
            _equalityConstraints.push_back(constraint);
            _equalityCoefficients.push_back(INITIAL_EQUALITY_COEF);
        }

        /**
         * Start from current given point and use Gradient 
         * Descent to optimize the target function along with
         * enforcing given constraints
         * Return the founded point
         * If debug is true, print debug information
         */
        inline ValueContainer runOptimization(bool debug = false)
        {
            //Verbose debug
            if (debug) {
                std::cout << "Start Penalty Method for ";
                std::cout << _targetFunction->toString() << std::endl;
                std::cout << "Equality constraints:" << std::endl;
                for (size_t i=0;i<_equalityConstraints.size();i++) {
                    std::cout << _equalityConstraints[i]->toString();
                    std::cout << std::endl;
                }
            }
            bool isConverged = false;
            _countGradientIteration = 0;
            _countPenaltyIteration = 0;
            while (!isConverged) {
                //Verbose debug
                if (debug) {
                    std::cout << "Start penalty iteration ";
                    std::cout << _countPenaltyIteration << std::endl;
                }
                //Build unconstrainted target function
                TermPtr target = buildUnconstraintedFunction();
                //Gradient descent
                GradientDescent<scalar> gradientDescent(target, _variables);
                //Set starting point with current approximation
                for (size_t i=0;i<_values.size();i++) {
                    gradientDescent.state(_values.getKey(i)) = _values[i];
                }
                //Do optimization
                _values = gradientDescent.runOptimization(debug);
                //Check if constrainted are enforced
                isConverged = true;
                for (size_t i=0;i<_equalityConstraints.size();i++) {
                    //If no enforced, penality coefficient is increased
                    if (evalConstraint(
                        penaltyEqualityFunction(_equalityConstraints[i])) 
                        > EQUALITY_EPSILON*EQUALITY_EPSILON
                    ) {
                        _equalityCoefficients[i] *= EQUALITY_COEF_GAIN;
                        isConverged = false;
                    }
                }
                //Check for convergence
                if (_countPenaltyIteration > MAX_LOOP) {
                    throw std::runtime_error(
                        "PenaltyMethod unable to converge");
                }
                _countGradientIteration += 
                    gradientDescent.getIterationCount();
                _countPenaltyIteration++;
                //Verbose debug
                if (debug) {
                    std::cout << "Penalty state:" << std::endl;
                    for (size_t i=0;i<_values.size();i++) {
                        std::cout << _values.getKey(i);
                        std::cout << ": " << _values[i] << std::endl;
                    }
                    std::cout << "Constraints state:" << std::endl;
                    for (size_t i=0;i<_equalityConstraints.size();i++) {
                        std::cout << _equalityConstraints[i]->toString();
                        std::cout << " " << _equalityCoefficients[i];
                        std::cout << " (";
                        std::cout << evalConstraint(
                            penaltyEqualityFunction(_equalityConstraints[i]));
                        std::cout << ")" << std::endl;
                    }
                }
            }

            return _values;
        }

    private:

        /**
         * Initial Symbolic Target function to minimize
         */
        TermPtr _targetFunction;

        /**
         * Symbolic variables container
         */
        VariableContainer _variables;

        /**
         * Current variables value
         */
        ValueContainer _values;

        /**
         * The Symbolic equality constraints 
         * container to enforce (equals to zero)
         */
        std::vector<TermPtr> _equalityConstraints;

        /**
         * Positive penalty coefficients 
         * container enforcing the associated 
         * equality constraint
         */
        std::vector<scalar> _equalityCoefficients;

        /**
         * Count the number of Gradient Descent and
         * Penalty iteration
         */
        unsigned int _countGradientIteration;
        unsigned int _countPenaltyIteration;

        /**
         * Build the Symbolic penalty function for 
         * equality constraints with given constraints
         */
        TermPtr penaltyEqualityFunction(TermPtr constraint)
        {
            return Symbolic::Pow<scalar>::create(constraint, 2);
        }

        /**
         * Build the unconstrainted target function
         * to optimize with gradient descent
         * Use current penalty coefficients
         */
        TermPtr buildUnconstraintedFunction()
        {
            TermPtr target = _targetFunction;
            for (size_t i=0;i<_equalityConstraints.size();i++) {
                target = Symbolic::Add<scalar>::create(
                    target,
                    Symbolic::Mult<scalar,scalar,scalar>::create(
                        Symbolic::Constant<scalar>::
                            create(_equalityCoefficients[i]),
                        penaltyEqualityFunction(_equalityConstraints[i])));
            }

            return target;
        }

        /**
         * Evaluate the given constraint with 
         * current state point
         */
        scalar evalConstraint(TermPtr constraint)
        {
            Symbolic::Bounder bounder;
            for (size_t i=0;i<_values.size();i++) {
                bounder.setValue(_variables[i], _values[i]);
            }

            constraint->reset();
            return constraint->evaluate(bounder);
        }
};

}
}

#endif

