#ifndef LEPH_SIMMECHA_FIXEDBASE_HPP
#define LEPH_SIMMECHA_FIXEDBASE_HPP

#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/Base.hpp"

namespace Leph {
namespace SimMecha {

/**
 * FixedBase
 *
 * Base implementation for fixed static Base
 */
class FixedBase : public Base
{
    public:

        /**
         * Initialization
         */
        FixedBase(const Vector2D& pos, SymbolPtr time) :
            Base(time)
        {
            //Initiating Body Symbolic 
            //position and velocity
            TermVectorPtr symPos = 
                ConstantVector::create(pos);
            TermPtr symAngle = Symbol::create(
                Symbolic::BaseSymbol::zero());
            TermVectorPtr symPosVel = SymbolVector::create(
                Symbolic::BaseSymbol::zero());
            TermPtr symAngleVel = Symbol::create(
                Symbolic::BaseSymbol::zero());
            
            //Initalize Symbols and compute 
            //lagrangian symbolic expression
            Body::initSymbols(
                symPos, symAngle, symPosVel, symAngleVel);
        }
};

}
}

#endif

