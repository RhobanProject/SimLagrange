#ifndef LEPH_SYMOPTIM_GRADIENTDESCENT_HPP
#define LEPH_SYMOPTIM_GRADIENTDESCENT_HPP

#include <string>
#include <stdexcept>
#include <iostream>
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
        unsigned int MAX_INCR_STEP;
        unsigned int MAX_GRADIENT_STEP;

        /**
         * Initialization with Symbolic target function
         * and Symbolic variables
         */
        GradientDescent
            (TermPtr targetFunction, const VariableContainer& variables) :
            INITIAL_STEP(0.0001),
            STEP_ADAPTIVE_GAIN(1.5),
            STOP_EPSILON(0.01), //TODO
            MAX_INCR_STEP(100),
            MAX_GRADIENT_STEP(500), //TODO
            _targetFunction(targetFunction),
            _variables(variables),
            _countIteration(0),
            _targetDerivatives(),
            _currentState()
        {
            if (_variables.size() == 0) {
                throw std::logic_error("GradientDescent empty variables");
            }

            //Initialize target derivatives and current
            //point values
            initDerivatives();
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
         * Return the number of run iteration
         */
        inline unsigned int getIterationCount() const
        {
            return _countIteration;
        }

        /**
         * Return current state
         */
        inline const ValueContainer& state() const
        {
            return _currentState;
        }

        /**
         * Start from current given initial point and 
         * do a gradient descent until stop condition is meet
         * Return the founded minimal point
         * If debug is true, print debug information
         */
        inline ValueContainer runOptimization(bool debug = false)
        {
            //Verbose debug
            if (debug) {
                std::cout << "Start Gradient Descent of ";
                std::cout << _targetFunction->toString() << std::endl;
                std::cout << "Derivatives:" << std::endl;
                for (size_t i=0;i<_variables.size();i++) {
                    std::cout << _variables.getKey(i) << ": ";
                    std::cout << _targetDerivatives[i]->toString();
                    std::cout << std::endl;
                }
            }
            //Descent initial step
            scalar step = INITIAL_STEP;
            //Compute gradient at current state point
            ValueContainer gradient = computeGradient(_currentState);
            _countIteration = 0;
            do {
                //Verbose debug
                if (debug) {
                    std::cout << "Start gradient iteration ";
                    std::cout << _countIteration << std::endl;
                }
                //Try to adapt the step size with simple heuristic
                scalar valueOld = computeTarget(_currentState);
                scalar valueNew = computeTarget(
                    applyStep(_currentState, gradient, step));
                //Decrease the step if target value increase
                while (valueNew > valueOld) {
                    step = step/STEP_ADAPTIVE_GAIN;
                    valueNew = computeTarget(
                        applyStep(_currentState, gradient, step));
                    //Verbose debug
                    if (debug) {
                        std::cout << "Decr step=" << step << std::endl;
                    }
                }
                //Increase the step while target value decrease
                unsigned int countStep = 0;
                while (valueNew < valueOld) {
                    step = step*STEP_ADAPTIVE_GAIN;
                    valueOld = valueNew;
                    valueNew = computeTarget(
                        applyStep(_currentState, gradient, step));
                    //Verbose debug
                    if (debug) {
                        std::cout << "Incr step=" << step << std::endl;
                    }
                    //Check for non convex target function
                    if (countStep > MAX_INCR_STEP) {
                        throw std::runtime_error(
                            "GradientDescent unable to find minimum");
                    }
                    countStep++;
                }
                step = step/STEP_ADAPTIVE_GAIN;
                //Update current state point with founded step
                _currentState = applyStep(_currentState, gradient, step);
                //Recompute gradient
                gradient = computeGradient(_currentState);
                //Verbose debug
                if (debug) {
                    std::cout << "Gradient iteration " << _countIteration;
                    std::cout << " with step: " << step << std::endl;
                    std::cout << "State:" << std::endl;
                    for (size_t i=0;i<_variables.size();i++) {
                        std::cout << _variables.getKey(i) << "=";
                        std::cout << _currentState[i] << " (gradient=";
                        std::cout << gradient[i] << ")" << std::endl;
                    }
                }
                //Check for non convergence
                if (_countIteration > MAX_GRADIENT_STEP) {
                    throw std::runtime_error(
                        "GradientDescent unable to converge");
                }
                _countIteration++;
            } while (infinityNorm(gradient) > STOP_EPSILON);

            return _currentState;
        }

    protected:
        
        /**
         * The Symbolic multivariate
         * function to be optimized
         */
        TermPtr _targetFunction;

        /**
         * Symbolic variables container
         */
        VariableContainer _variables;

        /**
         * Compute and initialize
         * target derivatites and reset current state
         */
        inline void initDerivatives()
        {
            _targetDerivatives.clear();
            _currentState.clear();
            for (size_t i=0;i<_variables.size();i++) {
                _targetDerivatives.push(
                    _variables.getKey(i),
                    _targetFunction->derivate(_variables[i]));
                _currentState.push(
                    _variables.getKey(i),
                    scalar(0.0));
            }
        }

    private:

        /**
         * Count the number of descent iteration
         */
        unsigned int _countIteration;

        /**
         * The Symbolic multivariate 
         * derivatives of target function with
         * respect to each Symbolic variable
         */
        DerivativeContainer _targetDerivatives;

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

