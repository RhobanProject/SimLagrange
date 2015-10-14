#ifndef LEPH_SIMMECHA_UNARYCONSTRAINT_HPP
#define LEPH_SIMMECHA_UNARYCONSTRAINT_HPP

#include <stdexcept>
#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/System.hpp"
#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/Constraint.hpp"
#include "SymOptim/src/PenaltyMethod.hpp"

namespace Leph {
namespace SimMecha {

/**
 * UnaryConstraint
 *
 * Base class for representing
 * mechanical constraint between one Body
 * and one static Body
 * (Collision response do not preserve 
 * linear and angular momentum, only one
 * Body state is update)
 */
class UnaryConstraint : public Constraint
{
    public:

        /**
         * Initialization with the Body and System involved
         * and collision parameters
         */
        UnaryConstraint(Body& body, System& system, 
            scalar restitutionCoef, bool isFriction) :
            Constraint(restitutionCoef, isFriction),
            _body(&body),
            _system(&system)
        {
        }

        /**
         * TODO TODO TODO
         */
        inline void handle()
        {
            Vector2D point;
            Vector2D dir;
            Vector2D posInBody;

            if (!computeCheckConstraint(point, dir, posInBody)) {
                return;
            }

            //TODO Bijection
            scalar timeMin = 0.0;
            scalar timeMax = 0.01;
            scalar currentTime = 0.0;
            for (int i=0;i<10;i++) {
                scalar t = (timeMax-timeMin)/2.0;
                _system->runSimulationStep(currentTime-t);
                currentTime = t;
                if (computeCheckConstraint(point, dir, posInBody)) {
                    timeMin = t;
                } else {
                    timeMax = t;
                }
            }
            std::cout << "TTT> " << currentTime << " " << timeMax << std::endl;
            _system->runSimulationStep(currentTime-timeMax);
            computeCheckConstraint(point, dir, posInBody);
            //

            std::cout << "pouet" << std::endl;
            std::cout << point << std::endl;

            Vector2D impulse = computeImpulsion(
                point, dir);
            std::cout << impulse << std::endl;
            propagateImpulse(impulse, point, posInBody);
        }

        /**
         * TODO
         */
        inline void propagateImpulse(const Vector2D& impulse, 
            const Vector2D& point, const Vector2D& posInBody)
        {
            //Get global coordinate Symbolic velocity of
            //collision point
            TermVectorPtr currentVelSym = 
                _body->buildSymPositionVel(posInBody);
            //Bind position Symbol
            currentVelSym = 
                _system->bindStatePosition(currentVelSym);
            //Get binded system lagrangian
            TermPtr lagrangian = 
                _system->getBindedLagrangian();
            //Get velocity degrees of freedom Symbols
            const DofContainer& dofs = 
                _system->getVelocityDofs();

            //Build impulse constraint
            TermVectorPtr constraint = Symbolic::Sub<Vector2D>::create(
                currentVelSym,
                ConstantVector::create(impulse));
            TermPtr constraintX = Symbolic::X<scalar,Vector2D>::
                create(constraint);
            TermPtr constraintY = Symbolic::Y<scalar,Vector2D>::
                create(constraint);

            std::cout << currentVelSym->toString() << std::endl;
            //std::cout << lagrangian->toString() << std::endl;
            for (size_t i=0;i<dofs.size();i++) {
                std::cout << dofs.getKey(i) << std::endl;
            }

            //Initialize Optimization target function and
            //constraints
            SymOptim::PenaltyMethod<scalar> penaltyOptim(
                lagrangian, dofs);
            penaltyOptim.addEqualityConstraint(constraintX);
            penaltyOptim.addEqualityConstraint(constraintY);
            //Initialize point
            for (size_t i=0;i<dofs.size();i++) {
                penaltyOptim.state(dofs.getKey(i)) = 0.0;
            }
            //Run optimization
            std::cout << "Start Optim" << std::endl;
            penaltyOptim.runOptimization(true);
            //Get result TODO
            for (size_t i=0;i<dofs.size();i++) {
                std::cout << dofs.getKey(i) << " " <<
                    penaltyOptim.state(dofs.getKey(i)) << std::endl;
                _system->stateVelocity(dofs.getKey(i)) += 
                    penaltyOptim.state(dofs.getKey(i));
            }
        }

    protected:
        
        /**
         * The Body involved in the
         * Constraint
         */
        Body* _body;

        /**
         * The System, the Body
         * is belonging to
         */
        System* _system;
        
        /**
         * Check if the constraint against 
         * current system state.
         * Return false is the constraint is tied
         * Return true in case of violation of the 
         * constraint and set for the Body 
         * the position and normal direction of
         * detected collision
         * The position is given in global frame
         * The direction is normal to collision surface
         * and upward to Body face and is normalized
         * PosInBody is the position of collision
         * in the Body coordinate
         */
        virtual bool computeCheckConstraint(
            Vector2D& point, Vector2D& dir, Vector2D& posInBody) = 0;

        /**
         * Compute and return the velocity impulsion
         * given the contact point (in global frame)
         * and collision normal direction
         */
        inline Vector2D computeImpulsion
            (const Vector2D& point, const Vector2D& dir) const
        {
            if (dir.norm() < 0.999 || dir.norm() > 1.001) {
                throw std::logic_error(
                    "UnaryConstraint not normalized dir");
            }

            //Compute velocity of collisioning point
            Vector2D centerPos = _system->evalPosition(*_body);
            Vector2D centerVel = _system->evalPositionVel(*_body);
            scalar centerAngleVel = _system->evalAngleVel(*_body);
            Vector2D vel = centerVel 
                + centerAngleVel*Vector2D::normal(point - centerPos);

            //Declare normal and tangent unit vector
            Vector2D n = dir;
            Vector2D t = Vector2D::normal(dir);
            
            //Build impulse
            Vector2D impulse(0.0, 0.0);
            //Tangent (friction) component
            if (Constraint::isFriction()) {
                impulse += -Vector2D::dot(vel, t)*t;
            }
            //Normal (response) component
            scalar e = Constraint::getRestitutionCoef();
            impulse += -(e+1.0)*Vector2D::dot(vel, n)*n;

            return impulse;
        }
};

}
}

#endif

