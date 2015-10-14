#ifndef LEPH_SIMMECHA_CONSTRAINT_HPP
#define LEPH_SIMMECHA_CONSTRAINT_HPP

#include <stdexcept>
#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/System.hpp"
#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * Constraint
 *
 * Base class for both Unary 
 * and Binary Constraints
 * (Collision involving one or two Bodies)
 */
class Constraint
{
    public:

        /**
         * Initialization of constraint with
         * the coefficient of restitution between 0 and 1
         * and if the collision response should handle friction
         */
        Constraint(scalar restitutionCoef, bool isFriction) :
            _restitutionCoef(restitutionCoef),
            _isFriction(isFriction)
        {
            if (_restitutionCoef < 0.0 || _restitutionCoef > 1.0) {
                throw std::logic_error(
                    "Constraint invalid restitution coef");
            }
        }

        /**
         * Virtual destructor
         */
        virtual ~Constraint()
        {
        }

    protected:

        /**
         * Access to collision parameters
         */
        inline scalar getRestitutionCoef() const
        {
            return _restitutionCoef;
        }
        inline bool isFriction() const
        {
            return _isFriction;
        }

    private:

        /**
         * The coefficient of restitution 
         * between 0 and 1
         * (1 means perfect elastic collision and
         * 0 means perfect inelastic collision)
         */
        scalar _restitutionCoef;

        /**
         * False means no friction
         * True means full friction (set 
         * tangent velocity response to zero)
         */
        bool _isFriction;
};

}
}

#endif

