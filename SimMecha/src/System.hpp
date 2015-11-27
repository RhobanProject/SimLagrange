#ifndef LEPH_SIMMECHA_SYSTEM_HPP
#define LEPH_SIMMECHA_SYSTEM_HPP

#include <vector>
#include <stdexcept>
#include <sstream>
#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/AngularJoint.hpp"
#include "SimMecha/src/AngularSpring.hpp"
#include "SimMecha/src/LinearJoint.hpp"
#include "SimMecha/src/LinearSpring.hpp"
#include "SimMecha/src/CamJoint.hpp"
#include "SimMecha/src/CamJointInverted.hpp"
#include "SimMecha/src/RoundFoot.hpp"
#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Base.hpp"
#include "SimMecha/src/FixedBase.hpp"
#include "SimMecha/src/FloatingBase.hpp"
#include "SimViewer/src/SimViewer.hpp"
#include <functional>

namespace Leph {
namespace SimMecha {

/**
 * System
 *
 * Represent a mechanical system
 * A chain of links and rigid bodies
 */
class System
{
    public:

        /**
         * Initialization with Fixed Base of
         * given position
         */
        System(const Vector2D& pos) :
            _base(NULL),
            _bodies(),
            _joints(),
            _time(Symbol::create("t")),
            _dofs(),
            _velDofs(),
            _lagrangian(),
            _dynamics(),
            _statePosition(),
            _stateVelocity(),
            _stateTorque(),
            _is_init(true)
        {
            //Init lagrangian to null
            _lagrangian = Symbol::create(
                Symbolic::BaseSymbol::zero());
            //Initialize Fixed Base with constant position
            _base = new FixedBase(pos, _time);
        }

        /**
         * Initialization with Floating Base of
         * given initial position and velocity
         * (translation movement)
         */
        System(const Vector2D& pos, const Vector2D& vel) :
            _base(NULL),
            _bodies(),
            _joints(),
            _time(Symbol::create("t")),
            _dofs(),
            _velDofs(),
            _lagrangian(),
            _dynamics(),
            _statePosition(),
            _stateVelocity(),
            _stateTorque(),
            _is_init(true)
        {
            //Init lagrangian to null
            _lagrangian = Symbol::create(
                Symbolic::BaseSymbol::zero());
            //Initialize Floating Base with moving
            //position degree of freedom
            createDof("x");
            createDof("y");
            _base = new FloatingBase(_dofs["x"], _dofs["y"], _time);
            _statePosition.push_back(pos.x());
            _statePosition.push_back(pos.y());
            _stateVelocity.push_back(vel.x());
            _stateVelocity.push_back(vel.y());
            _stateTorque.push_back(0.0);
            _stateTorque.push_back(0.0);
        }

        /**
         * Destructor
         */
        virtual ~System()
        {
            //Desallocation
            if (_base != NULL) {
                delete _base;
            }
            for (size_t i=0;i<_bodies.size();i++) {
                delete _bodies[i];
            }
            for (size_t i=0;i<_joints.size();i++) {
                delete _joints[i];
            }
        }

        /**
         * Return the System Base
         */
        inline const Base& getBase() const
        {
            return *_base;
        }
        inline Base& getBase()
        {
            return *_base;
        }

        inline std::vector<Body*> getBodies()
        {
            return _bodies;
        }


        /**
         * Create and return a new Body linked to the given
         * Body with a rotational Joint
         * Joint position and angle are given within Bodies
         * coordinates system
         * Initial position and velocity are given
         */
        inline Body& addAngularJoint(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new AngularJoint(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof());
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }

        /**
         * Create and return a new Body linked to the given
         * Body with a linear Joint
         * Joint position and angle are given within Bodies
         * coordinates system
         * Initial position and velocity are given
         */
        inline Body& addLinearJoint(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new LinearJoint(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof());
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }


       inline Body& addLinearSpring(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
                                    std::function<TermPtr(TermPtr)> F,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new LinearSpring(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(),F);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }


           inline Body& addLinearSpring(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
                                        scalar K, scalar l0,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new LinearSpring(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(),K, l0);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }



       inline Body& addAngularSpring(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
                                    std::function<TermPtr(TermPtr)> F,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new AngularSpring(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(),F);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }


       inline Body& addAngularSpring(Body& root,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
                                        scalar K, scalar l0,
            scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new AngularSpring(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(),K, l0);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }



        /**
         * Create and return a new Body linked to the given
         * Body with a cam Joint
         * Joint position and angle are given within Bodies
         * coordinates system
         * Initial position and velocity are given
         */
        inline Body& addCamJoint(Body& root,
            const Vector2D& posRoot, scalar angleRoot,const Vector2D& posLeaf, scalar angleLeaf, std::function<TermPtr(TermPtr)> F, scalar H, scalar phi,
                                 scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new CamJoint(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(), F, H, phi);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }

        inline Body& addCamJointInverted(Body& root,
            const Vector2D& posRoot, scalar angleRoot,const Vector2D& posLeaf, scalar angleLeaf, std::function<TermPtr(TermPtr)> F, scalar H, scalar phi,
                                 scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new CamJointInverted(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(), F, H, phi);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }

        inline Body& addRoundFoot(Body& root,
            const Vector2D& posRoot, scalar angleRoot,const Vector2D& posLeaf, scalar angleLeaf, scalar r, scalar gamma,
                                 scalar statePos, scalar stateVel)
        {
            Body* leaf = new Body(_time);
            Joint* joint = new RoundFoot(
                root, posRoot, angleRoot,
                *leaf, posLeaf, angleLeaf,
                createDof(), r, gamma, statePos);
            leaf->setJointRoot(joint);
            root.addLeafJoint(joint);

            _bodies.push_back(leaf);
            _joints.push_back(joint);
            _statePosition.push_back(statePos);
            _stateVelocity.push_back(stateVel);
            _stateTorque.push_back(0.0);

            return *leaf;
        }


        /**
         * Build for all Bodies
         * position and velocity Symbolic expressions
         * Compute lagrangian expression and compute
         * all dynamic equations
         * (Must to be called right after System
         * structure declaration)
         */
        inline void initSymbols()
        {

            _lagrangian = Symbol::create(
                Symbolic::BaseSymbol::zero()); //reset Lagrangian


            _Ec=Symbol::create(
                Symbolic::BaseSymbol::zero());
            _Ep=Symbol::create(
                Symbolic::BaseSymbol::zero());


            //We assume that Joints in the container are ordered
            //by dependencies (true by construction)
            for (size_t i=0;i<_joints.size();i++) {
                _joints[i]->computeSymTransformation(_time);
            }
            //Recompute Base lagrangian (new masses added)
            _base->initSymbols();
            //Compute lagrangian expression
            _lagrangian = _base->getLagrangian();
            for (size_t i=0;i<_bodies.size();i++) {
                _lagrangian = Symbolic::Add<scalar>::create(
                    _lagrangian,
                    _bodies[i]->getLagrangian());

                _Ec = Symbolic::Add<scalar>::create(
                    _Ec,
                    _bodies[i]->getKinetic());
                _Ep = Symbolic::Add<scalar>::create(
                    _Ep,
                    _bodies[i]->getPotential());

            }

                //Add Joint's Lagrangian (springs)
            for (size_t i=0;i<_joints.size();i++) {
                _joints[i]->initSymbols();
                // std::cout<<"DEBUG: "<<_joints[i]->getLagrangian()->toString()<<std::endl;

                _lagrangian = Symbolic::Add<scalar>::create(
                    _lagrangian,
                    _joints[i]->getLagrangian());

                _Ec = Symbolic::Add<scalar>::create(
                    _Ec,
                    _bodies[i]->getKinetic());
                _Ep = Symbolic::Add<scalar>::create(
                    _Ep,
                    _bodies[i]->getPotential());

            }

                //TODO optimize with Ec and Ep

            _dynamics.clear();
            _velDofs.clear();

            //Compute dynamic equations
            for (size_t i=0;i<_dofs.size();i++) {
                TermPtr dL_dq = _lagrangian
                    ->derivate(_dofs[i]);
                TermPtr dL_ddq = _lagrangian
                    ->derivate(_dofs[i]->derivate(_time));
                TermPtr dL_ddq_dt = dL_ddq
                    ->derivate(_time);
                TermPtr dynamic =
                    Symbolic::Sub<scalar>::create(dL_ddq_dt, dL_dq);

                _dynamics.push(_dofs[i]->toString(), dynamic);
            }
            //Build velocity degree of freedom container
            for (size_t i=0;i<_dofs.size();i++) {
                _velDofs.push(_dofs.getKey(i),
                    _dofs[i]->derivate(_time));
            }
        }

        /**
         * Run mechanical dynamic simulation for a
         * time step of given dt
         * Update state position and velocity accordingly with
         * internal dynamics and applied torques
         */
        inline void runSimulationStep(scalar dt)
        {
            if(_is_init)
            {
                _init_statePosition=_statePosition;
                _init_stateVelocity=_stateVelocity;
                _init_stateTorque=_stateTorque;
                _init_basePosition= evalPosition(getBase()); //TODO

                _is_init=false;
            }

            simulationComputeStep(
                _statePosition,
                _stateVelocity,
                _stateTorque,
                dt, _dofs,
                _dynamics, _time);
        }

        /**
         * Yep, just reset the system...
         *
         */

        inline void stateReset()
        {
            _statePosition=_init_statePosition;
            _stateVelocity=_init_stateVelocity;
            _stateTorque=_init_stateTorque;
            getBase().setPos(_init_basePosition);
            initSymbols();
        }


        inline std::vector<Joint*>& getJoints()
        {
            return _joints;
        }

        /**
         * Access the given degree of freedom
         * name the given state value for position
         * and velocity
         */
        inline const scalar& statePosition
            (const std::string& name) const
        {
            return _statePosition[_dofs.getIndex(name)];
        }
        inline scalar& statePosition(const std::string& name)
        {
            return _statePosition[_dofs.getIndex(name)];
        }
        inline const scalar& stateVelocity
            (const std::string& name) const
        {
            return _stateVelocity[_dofs.getIndex(name)];
        }
        inline scalar& stateVelocity(const std::string& name)
        {
            return _stateVelocity[_dofs.getIndex(name)];
        }

        /**
         * Access to degree of freedom applied torque
         */
        inline const scalar& stateTorque
            (const std::string& name) const
        {
            return _stateTorque[_dofs.getIndex(name)];
        }
        inline scalar& stateTorque(const std::string& name)
        {
            return _stateTorque[_dofs.getIndex(name)];
        }

        /**
         * Return true if the given degree of freedom
         * name exists
         */
        inline bool isDof(const std::string& name) const
        {
            return _dofs.isKey(name);
        }

        inline DofContainer getDof() const
        {
            return _dofs;
        }

        /**
         * Evaluate the position, angle and their velocity
         * with current system state
         */
        inline const Vector2D& evalPosition(Body& body)
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            body.getSymPosition()->reset();
            return body.getSymPosition()->evaluate(bounder);
        }
        inline scalar evalAngle(Body& body)
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            body.getSymAngle()->reset();
            return body.getSymAngle()->evaluate(bounder);
        }
        inline const Vector2D& evalPositionVel(Body& body)
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            body.getSymPositionVel()->reset();
            return body.getSymPositionVel()->evaluate(bounder);
        }
        inline scalar evalAngleVel(Body& body)
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            body.getSymAngleVel()->reset();
            return body.getSymAngleVel()->evaluate(bounder);
        }


        inline TermPtr getPotential()
        {
            return _Ep;
        }
        inline TermPtr getKinetic()
        {
            return _Ec;
        }

        inline scalar evalPotential()
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            _Ep->reset();
            return _Ep->evaluate(bounder);
        }
        inline scalar evalKinetic()
        {
            Symbolic::Bounder bounder = bounderFromCurrentState();
            _Ec->reset();
            return _Ec->evaluate(bounder);
        }


        /**
         * Return the Symbolic derivates degrees
         * of freedom container
         */
        inline const DofContainer& getVelocityDofs() const
        {
            return _velDofs;
        }

        /**
         * Return a Symbolic lagrangian with all degrees
         * of freedom (not theirs derivatives) substituate
         * with current state position
         */
        inline TermPtr getBindedLagrangian()
        {
            return bindStatePosition(_lagrangian);
        }

        /**
         * Return the given Symbolic expression with all
         * position degrees of freedom Symbol substituated
         * by current state values
         */
        inline TermPtr bindStatePosition(TermPtr term)
        {
            TermPtr termTmp = term;
            for (size_t i=0;i<_dofs.size();i++) {
                termTmp = termTmp->substitute<scalar>(_dofs[i],
                    Constant::create(_statePosition[i]));
            }

            return termTmp;
        }
        inline TermVectorPtr bindStatePosition(TermVectorPtr term)
        {
            TermVectorPtr termTmp = term;
            for (size_t i=0;i<_dofs.size();i++) {
                termTmp = termTmp->substitute<scalar>(_dofs[i],
                    Constant::create(_statePosition[i]));
            }

            return termTmp;
        }

        /**
         * Draw the system on given SimViewer
         */
        inline void draw(SimViewer::SimViewer& viewer)
        {
            //Draw Base
            Vector2D posBase = evalPosition(*_base);
            scalar angleBase = evalAngle(*_base);
            _base->draw(viewer, posBase, angleBase);
            //Draw all Bodies
            for (size_t i=0;i<_bodies.size();i++) {
                Vector2D pos = evalPosition(*_bodies[i]);
                scalar angle = evalAngle(*_bodies[i]);
                _bodies[i]->draw(viewer, pos, angle);
            }
            //Draw all Joints
            for (size_t i=0;i<_joints.size();i++) {
                Vector2D posRoot = evalPosition(
                    _joints[i]->getBodyRoot());
                scalar angleRoot = evalAngle(
                    _joints[i]->getBodyRoot());
                Vector2D posLeaf = evalPosition(
                    _joints[i]->getBodyLeaf());
                scalar angleLeaf = evalAngle(
                    _joints[i]->getBodyLeaf());
                size_t index = _dofs.getIndex(
                    _joints[i]->getName());
                _joints[i]->draw(viewer,
                    posRoot, angleRoot, posLeaf, angleLeaf,
                    _statePosition[index]);
            }
        }

    inline void plot(SimViewer::SimViewer& viewer, std::vector<Vector2D> data)
        // inline void plot(SimViewer::SimViewer& viewer)
        {

            viewer.plot(data, 0,100);
        }

    private:

        /**
         * System Base
         * (root of kinematic chain)
         */
        Base* _base;

        /**
         * Bodies container
         */
        std::vector<Body*> _bodies;

        /**
         * Joints container
         */
        std::vector<Joint*> _joints;

        /**
         * Time Symbol
         */
        SymbolPtr _time;

        /**
         * Degree of freedom container
         * The index is associated with Joint index
         */
        DofContainer _dofs;

        /**
         * Symbolic Derivates of degrees of freedom
         * container with respect to time
         */
        DofContainer _velDofs;

        /**
         * Total lagrangian Symbolic expression
         * for System
         */
        TermPtr _lagrangian;

            /**
         * Keep track of potential and kinetic energy
         */

    TermPtr _Ep;
    TermPtr _Ec;

        /**
         * Symbolic dynamic equation container
         * Indexed by degree of freedom derivative
         */
        TermContainer _dynamics;

        /**
         * State position and velocity values
         * of current mechanical System
         */
        std::vector<scalar> _statePosition;
        std::vector<scalar> _stateVelocity;

        /**
         * Applied torque values to associated
         * degree of freeodm (at each simulation step)
         */
        std::vector<scalar> _stateTorque;

        /**
         * Create a new degree of freedom
         * and return it
         */


        /**
         * Keep memory of the initial state
         */

        std::vector<scalar> _init_statePosition;
        std::vector<scalar> _init_stateVelocity;
        std::vector<scalar> _init_stateTorque;
        Vector2D _init_basePosition;

        bool _is_init;



    // DofContainer _init_dofs;
    // TermContainer _init_dynamics;
    // SymbolPtr _init_time;

        inline SymbolPtr createDof()
        {
            size_t index = _joints.size() + 1;
            std::ostringstream oss;
            oss << "q" << index;

            _dofs.push(oss.str(), Symbol::create(oss.str()));
            _dofs.last()->depend(_time);

            return _dofs.last();
        }
        inline SymbolPtr createDof(const std::string& name)
        {
            _dofs.push(name, Symbol::create(name));
            _dofs.last()->depend(_time);

            return _dofs.last();
        }

        /**
         * Return a bounder using current state position
         * and velocity
         */
        inline Symbolic::Bounder bounderFromCurrentState()
        {
            Symbolic::Bounder bounder;
            for (size_t i=0;i<_dofs.size();i++) {
                bounder.setValue(
                    _dofs[i], _statePosition[i]);
                bounder.setValue(
                    _dofs[i]->derivate(_time), _stateVelocity[i]);
            }

            return bounder;
        }
};

}
}

#endif
