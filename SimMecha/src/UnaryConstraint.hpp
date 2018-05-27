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
         * Detect if the constraint is violated.
         * In case of collision, a bijection search is used to 
         * back in past and to refine the collision time
         * and system state at this time.
         * Internal System state is updated.
         * WARNING: therefore, the simulation time step is 
         * NOT CONSTANT when a collision occurs.
         *
         * @param point Assigned collision point in world frame in case of collision.
         * @param dir Assigned contact normal in body frame in case of collision.
         * @param posInBody Assigned collision point in body frame in case of collision.
         * @param dt Normal simulation time step (for bijection search).
         * @return true if a constraint violation is detected.
         */
        inline bool getConstraint(Vector2D& point, Vector2D& dir, Vector2D& posInBody, double dt)
        {
            //Check if the constraint is currently violated
            if (!computeCheckConstraint(point, dir, posInBody)) {
                return false;
            }

            //Retrieve state before last simulation step
            std::vector<scalar> vectPos = _system->getLastPositions();
            std::vector<scalar> vectVel = _system->getLastVelocities();

            //Check that the constraint was not violated at last iteration
            _system->setState(vectPos, vectVel);
            _system->runSimulationStep(0.0);
            bool isCollision1 = computeCheckConstraint(point, dir, posInBody);
            _system->setState(vectPos, vectVel);
            _system->runSimulationStep(dt);
            bool isCollision2 = computeCheckConstraint(point, dir, posInBody);
            if (isCollision1 || !isCollision2) {
                std::cerr << "Constraint bijection sanity check failed" << std::endl;
                return true;
            }

            //Bijection to refine collision time
            scalar stepMin = 0.0;
            scalar stepMax = dt; 
            scalar step;
            for (int i=0;i<50;i++) { //50 is enough to reach double precision limit
                step = (stepMax+stepMin)/2.0;
                _system->setState(vectPos, vectVel);
                _system->runSimulationStep(step);
                if (computeCheckConstraint(point, dir, posInBody)) {
                    stepMax = step;
                } else {
                    stepMin = step;
                }
            }

            //Go back to collision time
            _system->setState(vectPos, vectVel);
            _system->runSimulationStep(step);
            computeCheckConstraint(point, dir, posInBody);
            return true;
        }

        /**
         * Check if the constraint is violated.
         * In case of collision, accurate time of collision
         * is refine and impulsion response is computed.
         */
        inline virtual void handle(double dt)
        {
            //Check for constraint violation
            Vector2D point;
            Vector2D dir;
            Vector2D posInBody;
            if (!getConstraint(point, dir, posInBody, dt)) {
                return;
            }

            //Compute expected contact velocity in 
            //world after collision impulsion
            Vector2D targetVel = computeTargetContactVelocity(posInBody, dir);

            //Compute system Jacobian matrix at 
            //contact point and system inertia mass matrix
            EigenMatrix matJac = _system->evalJacobian(*_body, posInBody);
            EigenMatrix matH = _system->evalInertiaMatrix();
            unsigned int ndof = matH.rows();
            
            //Retrieve in Eigen vector current system 
            //DoFs velocity vector
            EigenVector velOld(ndof);
            const DofContainer& dofs =
                _system->getVelocityDofs();
            for (size_t i=0;i<dofs.size();i++) {
                velOld(i) = _system->stateVelocity(dofs.getKey(i));
            }

            /**
             * When a collision occurs, the response is computed 
             * by applying an impulsion to the system (immediate
             * modification of degrees of freedom velocity).
             * Given the old (before collision) DoF velocities, 
             * the old Cartesian velocity at contact point and 
             * the new expected velocity at contact point, the
             * new DoF velocities under momentum conservation.
             * J: Jacobian at contact point.
             * H: Inertia matrix.
             * dq-: Old DoF velocities.
             * dq+: New DoF velocities.
             * v-:  Old Cartesian contact point velocity.
             * v+:  New Cartesian contact point velocity.
             * i:   Cartesian impulsion at contact point.
             * J.dq- = v-
             * J.dq+ = v+
             * H.dq+ = H.dq- + J'.i
             *
             * The following linear problem is solved:
             * |H J'|.|dq+| = |H.dq-|
             * |J 0 | |-i |   |v+|
             */

            EigenMatrix problemMat;
            EigenVector problemVec;
            if (!Constraint::isFriction()) {
                //When tangent friction is disabled,
                //only normal final velocity is enforced
                problemMat = EigenMatrix::Zero(ndof+1, ndof+1);
                problemMat.block(0, 0, ndof, ndof) = matH;
                problemMat.block(ndof, 0, 1, ndof) = matJac.block(1, 0, 1, ndof);
                problemMat.block(0, ndof, ndof, 1) = matJac.block(1, 0, 1, ndof).transpose();
                problemVec = EigenVector::Zero(ndof+1);
                problemVec.segment(0, ndof) = matH*velOld;
                problemVec(ndof+0) = targetVel.y();
            } else {
                //When tangent friction is enabled,
                //tangent final velocity is enforced to zero.
                //Note that partial tangent friction is possible.
                problemMat = EigenMatrix::Zero(ndof+2, ndof+2);
                problemMat.block(0, 0, ndof, ndof) = matH;
                problemMat.block(ndof, 0, 2, ndof) = matJac;
                problemMat.block(0, ndof, ndof, 2) = matJac.transpose();
                problemVec = EigenVector::Zero(ndof+2);
                problemVec.segment(0, ndof) = matH*velOld;
                problemVec(ndof+0) = 0.0;
                problemVec(ndof+1) = targetVel.y();
            }
            
            //Solve linear equation
            EigenVector velNew = problemMat.partialPivLu().solve(problemVec);

            //Assign back DoF velocities to system
            for (size_t i=0;i<dofs.size();i++) {
                _system->stateVelocity(dofs.getKey(i)) = velNew(i);
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
         * Compute and return the expected velocity of given 
         * contact point in body frame after the collision impulsion
         */
        inline Vector2D computeTargetContactVelocity
            (const Vector2D& posInBody, const Vector2D& dir) const
        {
            if (dir.norm() < 0.999 || dir.norm() > 1.001) {
                throw std::logic_error(
                    "UnaryConstraint not normalized dir");
            }

            //Compute the current of velocity 
            //of contacting point in word frame
            Vector2D vel = _system->evalPointVelocity(*_body, posInBody);

            //Build expected contact velocity
            Vector2D targetVel(0.0, 0.0);
            scalar e = Constraint::getRestitutionCoef();

            //Declare normal and tangent unit vector
            Vector2D n = dir;
            Vector2D t = Vector2D::normal(dir);
            //Tangent (friction) component
            if (!Constraint::isFriction()) {
                targetVel += Vector2D::dot(vel, t)*t;
            }
            //Normal (response) component
            targetVel += -e*Vector2D::dot(vel, n)*n;

            return targetVel;
        }
};

}
}

#endif
