#ifndef LEPH_SIMMECHA_SIMULATION_HPP
#define LEPH_SIMMECHA_SIMULATION_HPP

#include <vector>
#include <string>
#include <stdexcept>
#include "Eigen/Dense"
#include "VectorMap/src/VectorMap.hpp"
#include "Vector/src/Vector2D.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Constant.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/terms.h"

namespace Leph {
namespace SimMecha {

/**
 * Scalar type
 */
typedef double scalar;

/**
 * Typedef for Vector
 */
typedef Vector::Vector2D<scalar> Vector2D;

/**
 * Typedef for Symbol type (degree of freedom)
 */
typedef Symbolic::Symbol<scalar> Symbol;
typedef Symbolic::Symbol<scalar>::SymbolPtr SymbolPtr;
typedef Symbolic::Symbol<Vector2D> SymbolVector;
typedef Symbolic::Symbol<Vector2D>::SymbolPtr SymbolVectorPtr;

/**
 * Typedef for Term expression
 */
typedef Symbolic::Term<scalar> Term;
typedef Symbolic::Term<scalar>::TermPtr TermPtr;
typedef Symbolic::Term<Vector2D> TermVector;
typedef Symbolic::Term<Vector2D>::TermPtr TermVectorPtr;

/**
 * Typedef for Constant expression
 */
typedef Symbolic::Constant<scalar> Constant;
typedef Symbolic::Constant<Vector2D> ConstantVector;

/**
 * Typedef for Eigen vector
 */
typedef Eigen::Matrix
    <scalar, Eigen::Dynamic, 1> EigenVector;
typedef Eigen::Matrix
    <scalar, Eigen::Dynamic, Eigen::Dynamic> EigenMatrix;

/**
 * Typedef for degrees of freedom container
 */
typedef VectorMap::VectorMap<std::string, SymbolPtr> 
    DofContainer;

/**
 * Typedef for Symbolic dynamic container
 */
typedef VectorMap::VectorMap<std::string, TermPtr>
    TermContainer;

/**
 * Fill up and return Symbolic bounder
 * with given position, velocity and acceleration values
 * using given degree of freedom container
 */
inline Symbolic::Bounder simulationSetUpBounder(
    const EigenVector& position, 
    const EigenVector& velocity, 
    const EigenVector& acceleration, 
    const DofContainer& dofs, SymbolPtr time)
{
    Symbolic::Bounder bounder;
    for (size_t i=0;i<dofs.size();i++) {
        SymbolPtr sym = dofs[i];
        SymbolPtr sym_dt = sym->derivate(time);
        SymbolPtr sym_ddt = sym_dt->derivate(time);
        bounder.setValue(sym, position[i]);
        bounder.setValue(sym_dt, velocity[i]);
        bounder.setValue(sym_ddt, acceleration[i]);
    }

    return bounder;
}

/**
 * Compute and return the torques vector for each
 * degrees of freedom given position, 
 * velocity and acceleration values
 */
inline EigenVector simulationComputeTorques(
    const EigenVector& position,
    const EigenVector& velocity,
    const EigenVector& acceleration,
    const DofContainer& dofs, 
    TermContainer& dynamics, SymbolPtr time)
{
    Symbolic::Bounder bounder = simulationSetUpBounder(
        position, velocity, acceleration, dofs, time);
    
    EigenVector torques(dofs.size(), 1);
    for (size_t i=0;i<dofs.size();i++) {
        dynamics[dofs.getKey(i)]->reset();
        torques[i] = dynamics[dofs.getKey(i)]
            ->evaluate(bounder);
    }

    return torques;
}

/**
 * Compute and return the acceleration 
 * for each degrees of freedom given their position
 * velocity and applied torque using given dynamics
 */
inline EigenVector simulationComputeAccelerations(
    const EigenVector& position,
    const EigenVector& velocity,
    const EigenVector& torques,
    const DofContainer& dofs, 
    TermContainer& dynamics, SymbolPtr time)
{
    EigenVector forces = simulationComputeTorques(
        position, velocity, 
        EigenVector::Zero(dofs.size(), 1), 
        dofs, dynamics, time);
    EigenMatrix inertia = EigenMatrix::
        Zero(dofs.size(), dofs.size());
    EigenVector acceleration = EigenVector::
        Zero(dofs.size(), 1);
    for (size_t i=0;i<dofs.size();i++) {
        acceleration(i) = 1.0;
        inertia.col(i) = 
            simulationComputeTorques(position, velocity, 
                acceleration, dofs, dynamics, time) - forces;
        acceleration(i) = 0.0;
    }

    //Using LU decomposition for inversing inertia matrix
    Eigen::FullPivLU<EigenMatrix> inertiaLU(inertia);
    if (inertiaLU.isInvertible()) {
        return inertiaLU.inverse()*(torques - forces);
    } else {
        return EigenVector::Zero(dofs.size(), 1);
    }
}

/**
 * Compute and return the given state derivative applying 
 * given torques against given dynamics
 */
inline EigenVector simulationDifferentialEquations(
    const EigenVector& state, 
    const EigenVector& torques, 
    const DofContainer& dofs, 
    TermContainer& dynamics, SymbolPtr time)
{
    EigenVector position(dofs.size(), 1);
    EigenVector velocity(dofs.size(), 1);
    for (size_t i=0;i<dofs.size();i++) {
        position[i] = state[i];
    }
    for (size_t i=0;i<dofs.size();i++) {
        velocity[i] = state[i+dofs.size()];
    }

    EigenVector acceleration = simulationComputeAccelerations(
        position, velocity, torques, dofs, dynamics, time);

    EigenVector nextState(2*dofs.size(), 1);
    for (size_t i=0;i<dofs.size();i++) {
        nextState[i] = velocity[i];
    }
    for (size_t i=0;i<dofs.size();i++) {
        nextState[i+dofs.size()] = acceleration[i];
    }

    return nextState;
}
        
/**
 * Integrate the given state over dt given time. The given
 * torques vector is applied to degrees of freedom
 */
inline EigenVector simulationRungeKutta4(
    const EigenVector& state,
    const EigenVector& torques, 
    scalar dt, const DofContainer& dofs,
    TermContainer& dynamics, SymbolPtr time)
{
    EigenVector k1 = simulationDifferentialEquations
        (state, torques, dofs, dynamics, time);
    EigenVector k2 = simulationDifferentialEquations
        (state + (dt/2.0)*k1, torques, dofs, dynamics, time);
    EigenVector k3 = simulationDifferentialEquations
        (state + (dt/2.0)*k2, torques, dofs, dynamics, time);
    EigenVector k4 = simulationDifferentialEquations
        (state + dt*k3, torques, dofs, dynamics, time);

    return state + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

/**
 * Compute the next simulation step and 
 * update the given state using given
 * applied torques and time step
 */
inline void simulationComputeStep(
    std::vector<scalar>& statePosition,
    std::vector<scalar>& stateVelocity,
    const std::vector<scalar>& stateTorque,
    scalar dt, const DofContainer& dofs,
    TermContainer& dynamics, SymbolPtr time)
{
    if (
        statePosition.size() != dofs.size() ||
        stateVelocity.size() != dofs.size() ||
        stateTorque.size() != dofs.size()
    ) {
        throw std::logic_error("Simulation invalid state size");
    }

    EigenVector state(2*dofs.size(), 1);
    EigenVector torques(dofs.size(), 1);
    for (size_t i=0;i<dofs.size();i++) {
        state(i) = statePosition[i];
    }
    for (size_t i=0;i<dofs.size();i++) {
        state(i+dofs.size()) = stateVelocity[i];
    }
    for (size_t i=0;i<dofs.size();i++) {
        torques(i) = stateTorque[i];
    }

    state = simulationRungeKutta4(
        state, torques, dt, dofs, dynamics, time);
    
    for (size_t i=0;i<dofs.size();i++) {
        statePosition[i] = state(i);
    }
    for (size_t i=0;i<dofs.size();i++) {
        stateVelocity[i] = state(i+dofs.size());
    }
}

}
}

#endif

