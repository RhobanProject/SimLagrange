#ifndef LEPH_SIMMECHA_BINARYCONSTRAINT_HPP
#define LEPH_SIMMECHA_BINARYCONSTRAINT_HPP

#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/System.hpp"
#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/Constraint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * BinaryConstraint
 *
 * Base class for representing
 * mechanical constraint between two Bodies
 * (Both Bodies are update by collision
 * response)
 */
class BinaryConstraint : public Constraint
{
    public:

    private:
        
        /**
         * The two Bodies involved in the
         * Constraint
         */
        Body* _body1;
        Body* _body2;

        /**
         * The two Systems, the two Bodies
         * are belonging to (can be the same)
         */
        System* _system1;
        System* _system2;
        
        /**
         * Check if the constraint against 
         * current systems states.
         * Return false is the constraint is tied
         * Return true in case of violation of the 
         * constraint and set for the two Bodies 
         * the position and normal direction of
         * detected collision
         */
        virtual bool computeCheckConstraint(
            Vector2D& pos1, Vector2D& norm1, 
            Vector2D& pos2, Vector2D& norm2) = 0;
};

}
}

#endif

