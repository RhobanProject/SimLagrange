#ifndef LEPH_SYMOPTIM_GRADIENTDESCENT_HPP
#define LEPH_SYMOPTIM_GRADIENTDESCENT_HPP

#include <string>
#include <stdexcept>
#include "VectorMap/src/VectorMap.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/terms.h"

namespace Leph {
namespace SymOptim {

/**
 * GradientDescent
 *
 * Process a Gradient Descent
 * search on Symbolic expressions
 */
template <class scalar>
class GradientDescent
{
    public:

        /**
         * Typedef for Symbolic expressions
         */
        typedef typename Symbolic::Symbol<scalar> 
            Symbol;
        typedef typename Symbolic::Symbol<scalar>::SymbolPtr 
            SymbolPtr;
        typedef typename Symbolic::Term<scalar>::TermPtr 
            TermPtr;

        /**
         * Typedef for Symbolic searched variables container
         */
        typedef VectorMap::VectorMap<std::string, SymbolPtr> 
            VariableContainer;

        /**
         * Typedef for searched variables values container
         */
        typedef VectorMap::VectorMap<std::string, scalar> 
            ValueContainer;

        /**
         * Typedef for container of target function
         * derivatives with respect to each Symbolic variable
         */
        typedef VectorMap::VectorMap<std::string, TermPtr> 
            DerivativeContainer;

        /**
         * Configuration parameters
         */
        scalar INITIAL_STEP;
        scalar STEP_ADAPTIVE_GAIN;
        scalar STOP_EPSILON;

        /**
         * Initialization with Symbolic target function
         * and Symbolic variables
         */
        GradientDescent
            (TermPtr targetFunction, const VariableContainer& variables) :
            INITIAL_STEP(0.001),
            STEP_ADAPTIVE_GAIN(1.5),
            STOP_EPSILON(0.0001),
            _targetFunction(targetFunction),
            _targetDerivatives(),
            _variables(variables),
            _currentState()
        {
            if (_variables.size() == 0) {
                throw std::logic_error("GradientDescent empty variables");
            }

            //Initialize target derivatives and current
            //point values
            for (size_t i=0;i<_variables.size();i++) {
                _targetDerivatives.push(
                    _variables.getKey(i),
                    _targetFunction->derivate(_variables[i]));
                _currentState.push(
                    _variables.getKey(i),
                    scalar(0.0));
            }
        }

        /**
         * Access to variables state point values
         * by name
         */
        inline const scalar& state(const std::string& name) const
        {
            return _currentState[name];
        }
        inline scalar& state(const std::string& name)
        {
            return _currentState[name];
        }

        /**
         * Start from current given initial point and 
         * do a gradient descent until stop condition is meet
         * Return the founded minimal point
         */
        inline ValueContainer runOptimization()
        {
            //Descent initial step
            scalar step = INITIAL_STEP;
            //Compute gradient at current state point
            ValueContainer gradient = computeGradient(_currentState);
            do {
                //Try to adapt the step size with simple heuristic
                scalar valueOld = computeTarget(_currentState);
                scalar valueNew = computeTarget(
                    applyStep(_currentState, gradient, step));
                //Decrease the step if target value increase
                while (valueNew > valueOld) {
                    step = step/STEP_ADAPTIVE_GAIN;
                    valueNew = computeTarget(
                        applyStep(_currentState, gradient, step));
                }
                //Increase the step while target value decrease
                while (valueNew < valueOld) {
                    step = step*STEP_ADAPTIVE_GAIN;
                    valueOld = valueNew;
                    valueNew = computeTarget(
                        applyStep(_currentState, gradient, step));
                }
                step = step/STEP_ADAPTIVE_GAIN;
                //Update current state point with founded step
                _currentState = applyStep(_currentState, gradient, step);
                //Recompute gradient
                gradient = computeGradient(_currentState);
            } while (infinityNorm(gradient) > STOP_EPSILON);

            return _currentState;
        }

    private:

        /**
         * The Symbolic multivariate
         * function to be optimized
         */
        TermPtr _targetFunction;

        /**
         * The Symbolic multivariate 
         * derivatives of target function with
         * respect to each Symbolic variable
         */
        DerivativeContainer _targetDerivatives;

        /**
         * Symbolic variables container
         */
        VariableContainer _variables;

        /**
         * Current variables state value
         */
        ValueContainer _currentState;

        /**
         * Set up and return Symbolic bounder for
         * gradient evaluation with given state
         */
        inline Symbolic::Bounder setUpBounder
            (const ValueContainer& values) const
        {
            Symbolic::Bounder bounder;
            for (size_t i=0;i<_variables.size();i++) {
                bounder.setValue(
                    (SymbolPtr)_variables[i], 
                    values[_variables.getKey(i)]);
            }

            return bounder;
        }

        /**
         * Return the infinity norm of given
         * Value container
         */
        inline scalar infinityNorm
            (const ValueContainer& values) const
        {
            scalar norm(-1.0);
            for (size_t i=0;i<values.size();i++) {
                scalar abs = values[i];
                if (values[i] < 0.0) {
                    abs = -values[i];
                }
                if (norm < 0.0 || abs > norm) {
                    norm = abs;
                }
            }

            return norm;
        }

        /**
         * Compute and return gradient value container
         * with given state point container
         */
        inline ValueContainer computeGradient
            (const ValueContainer& values)
        {
            ValueContainer gradient;
            Symbolic::Bounder bounder = setUpBounder(values);
            for (size_t i=0;i<_variables.size();i++) {
                _targetDerivatives[i]->reset();
                gradient.push(
                    _variables.getKey(i),
                    _targetDerivatives[i]->evaluate(bounder));
            }

            return gradient;
        }

        /**
         * Evaluate the target function value
         * with given state point
         */
        inline scalar computeTarget(const ValueContainer& values)
        {
            Symbolic::Bounder bounder = setUpBounder(values);
            _targetFunction->reset();

            return _targetFunction->evaluate(bounder);
        }

        /**
         * Apply the given step grandient descent to given
         * state point and return it
         */
        inline ValueContainer applyStep(
            const ValueContainer& state, 
            const ValueContainer& gradient, scalar step) const
        {
            ValueContainer nexState = state;
            for (size_t i=0;i<state.size();i++) {
                nexState[i] += -step*gradient[i];
            }

            return nexState;
        }
};

}
}
#endif

