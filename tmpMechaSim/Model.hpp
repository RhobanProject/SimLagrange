#ifndef MODEL_HPP
#define MODEL_HPP

#include <vector>
#include <map>
#include <string>
#include <stdexcept>
#include "Eigen/Dense"
#include "Vector/src/Vector2D.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/Bounder.hpp"
#include "Symbolic/src/terms.h"

namespace Leph {

/**
 * Model
 *
 * Base class for mechanic simulation framework
 * using lagrangian physic
 * Represent one Model of simulated system
 */
class Model
{
    public:

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
        typedef Symbolic::Symbol<Model::Vector2D> SymbolVector;
        typedef Symbolic::Symbol<Model::Vector2D>::SymbolPtr SymbolVectorPtr;

        /**
         * Typedef for Term expression
         */
        typedef Symbolic::Term<scalar> Term;
        typedef Symbolic::Term<scalar>::TermPtr TermPtr;
        typedef Symbolic::Term<Model::Vector2D> TermVector;
        typedef Symbolic::Term<Model::Vector2D>::TermPtr TermVectorPtr;

        /**
         * Typedef for Eigen vector
         */
        typedef Eigen::Matrix<scalar, 
            Eigen::Dynamic, 1> EigenVector;
        typedef Eigen::Matrix<scalar, 
            Eigen::Dynamic, Eigen::Dynamic> EigenMatrix;

        /**
         * Typedef for degrees of freedom container
         */
        typedef std::map<std::string, SymbolPtr> DofContainer;

        /**
         * Typedef for dynamic model container
         */
        typedef std::map<std::string, TermPtr> ModelContainer;

        /**
         * Typedef for parameters container
         */
        typedef std::map<std::string, scalar> ParameterContainer;

        /**
         * Initialization with Model parameters
         */
        Model(const ParameterContainer& parameters) :
            _time(Symbol::create("t")),
            _dofs(),
            _lagrangian(),
            _parameters(parameters)
        {
        }

        /**
         * Virtual destructor
         */
        virtual ~Model()
        {
        }

        /**
         * Initialize the model
         */
        inline void initialization()
        {
            //Initialization of degree of freedom
            //and Lagrangian symbolic expression
            doInit();
            //The Lagrangian is now initialized
            //Computing Symbolic dynamic model expression
            DofContainer::iterator it;
            for (it=_dofs.begin();it!=_dofs.end();it++) {
                computeDynamicModel(it->second);
            }
        }

        /**
         * TODO
         */
        inline void test()
        {
            EigenVector position = EigenVector::Zero(_dofs.size(), 1);
            EigenVector velocity = EigenVector::Zero(_dofs.size(), 1);
            EigenVector torque = EigenVector::Zero(_dofs.size(), 1);
            position(0) = -0.5;
            std::cout << computeAcceleration(position, velocity, torque) << std::endl;
        }

    protected:

        /**
         * Symbolic time symbol
         */
        SymbolPtr _time;

        /**
         * Symbolic degrees of freedom container
         */
        DofContainer _dofs;

        /**
         * Lagrangian symbolic
         * expression
         */
        TermPtr _lagrangian;

        /**
         * Symbolic Dynamic model for all
         * degrees of freedom
         */
        ModelContainer _dynamic;
        
        /**
         * Model Parameters container
         */
        ParameterContainer _parameters;

        /**
         * Add the given Symbol name to degree of freedom container
         */
        inline void addDof(const std::string& name)
        {
            _dofs[name] = Model::Symbol::create(name);
            _dofs[name]->depend(_time);
        }

        /**
         * Initialization of the model
         * Init degrees of freedom
         * Compute kinetic and potential energy
         */
        virtual void doInit() = 0;

    private:

        /**
         * Build and save the dynamical symbolic expression
         * of torque accordingly with the model lagrangian
         * with respect to given degree of freedom
         */
        inline void computeDynamicModel(SymbolPtr dof)
        {
            TermPtr dL_dq = 
                _lagrangian->derivate(dof);
            TermPtr dL_ddq = 
                _lagrangian->derivate(dof->derivate(_time));
            TermPtr dL_ddq_dt = 
                dL_ddq->derivate(_time);
            TermPtr dynamic = 
                Symbolic::Sub<scalar>::create(dL_ddq_dt, dL_dq);

            _dynamic[dof->toString()] = dynamic;
        }

        /**
         * Compute according to the model the dynamic torque
         * with respect to given symbolic degree of freedom
         * The model state (position, velocity and acceleration)
         * of all degrees of freedom is given
         */
        inline scalar computeTorque(SymbolPtr dof, 
            const EigenVector& position, 
            const EigenVector& velocity, 
            const EigenVector& acceleration)
        {
            //Dofs state size check
            if (
                position.rows() != (int)_dofs.size() ||
                velocity.rows() != (int)_dofs.size() ||
                acceleration.rows() != (int)_dofs.size()
            ) {
                throw std::logic_error("Model invalid dofs state size");
            }

            //Set bounder values
            Symbolic::Bounder bounder;
            DofContainer::iterator it;
            int index = 0;
            for (it=_dofs.begin();it!=_dofs.end();it++) {
                SymbolPtr sym = it->second;
                SymbolPtr sym_dt = sym->derivate(_time);
                SymbolPtr sym_ddt = sym_dt->derivate(_time);
                bounder.setValue(sym, position[index]);
                bounder.setValue(sym_dt, velocity[index]);
                bounder.setValue(sym_ddt, acceleration[index]);
                index++;
            }

            //Reset dynamic model
            _dynamic.at(dof->toString())->reset();
            //Evaluate the dynamic equation
            return _dynamic.at(dof->toString())->evaluate(bounder);
        }

        /**
         * Call computeTorque for all degrees of freedom
         * and return torque vector
         */
        inline EigenVector computeVectorTorque(
            const EigenVector& position, 
            const EigenVector& velocity, 
            const EigenVector& acceleration)
        {
            EigenVector torque = EigenVector::Zero(_dofs.size(), 1);

            int index = 0;
            DofContainer::iterator it;
            for (it=_dofs.begin();it!=_dofs.end();it++) {
                torque(index) = computeTorque(
                    it->second, position, 
                    velocity, acceleration);
                index++;
            }

            return torque;
        }

        /**
         * Compute the acceleration of degrees 
         * of freedom given all degrees of freedom 
         * position and velocity and torque applied to it
         */
        inline EigenVector computeAcceleration(
            const EigenVector& position, 
            const EigenVector& velocity,
            const EigenVector& torque)
        {
            EigenVector force = computeVectorTorque(
                position, velocity, EigenVector::Zero(_dofs.size(), 1));
            EigenMatrix inertia = EigenMatrix::Zero(
                _dofs.size(), _dofs.size());
            EigenVector acceleration = EigenVector::Zero(_dofs.size(), 1);
            for (size_t i=0;i<_dofs.size();i++) {
                acceleration(i) = 1.0;
                inertia.col(i) = 
                    computeVectorTorque(position, velocity, acceleration)
                    - force;
                acceleration(i) = 0.0;
            }

            //Using LU decomposition for inversing inertia matrix
            Eigen::FullPivLU<EigenMatrix> inertiaLU(inertia);
            if (inertiaLU.isInvertible()) {
                return inertiaLU.inverse()*(torque - force);
            } else {
                return EigenVector::Zero(_dofs.size(), 1);
            }
        }
};

}

#endif

