#ifndef MODEL_HPP
#define MODEL_HPP

#include <vector>
#include <map>
#include <string>
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
            //Create symbol for gravity (not depending on time)
            _dofs["g"] = Symbol::create("g");
        }

        /**
         * Virtual destructor
         */
        virtual ~Model()
        {
        }

        /**
         * Initialize the model
         * Init degrees of freedom
         * Compute kinetic and potential energy
         */
        virtual void initialization() = 0;

        /**
         * TODO
         */
        inline void test()
        {
        }

        /**
         * TODO
         * Compute according to the model the dynamic torque
         * with respect to given symbolic degree of freedom
         * (derivation variable)
         * The model state (position, velocity and acceleration)
         * of all degrees of freedom is given
         */
        inline scalar computeTorque(SymbolPtr dof, 
            const EigenVector& position, 
            const EigenVector& velocity, 
            const EigenVector& acceleration, 
            bool withGravity)
        {
            TermPtr dL_dq = 
                _lagrangian->derivate(dof);
            TermPtr dL_ddq = 
                _lagrangian->derivate(dof->derivate(_time));
            TermPtr dL_ddq_dt = 
                dL_ddq->derivate(_time);
            TermPtr dynamic = 
                Symbolic::Sub<scalar>::create(dL_ddq_dt, dL_dq);

            Symbolic::Bounder;
        }

        /**
         * Access to time symbolic expression
         */
        /*
        inline SymbolPtr& getTime()
        {
            return _time;
        }
        */

        /**
         * Access to internal degree of freedom container
         */
        /*
        inline DofContainer& getDofs()
        {
            return _dofs;
        }
        */

        /**
         * Return Lagrangian symbolic 
         * expression for this model
         */
        /*
        inline TermPtr& getLagrangian()
        {
            return _lagrangian;
        }
        */

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
};

}

#endif

