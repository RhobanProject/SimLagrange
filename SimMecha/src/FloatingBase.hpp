#ifndef LEPH_SIMMECHA_FLOATINGBASE_HPP
#define LEPH_SIMMECHA_FLOATINGBASE_HPP

#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/Base.hpp"

namespace Leph {
namespace SimMecha {

/**
 * FloatingBase
 *
 * Base implementation for moving Base
 * with translation movement
 */
class FloatingBase : public Base
{
    public:

        /**
         * Initialization
         */
        FloatingBase(SymbolPtr baseX, SymbolPtr baseY, 
            SymbolPtr time) :
            Base(time)
        {
            //Initiating Body Symbolic 
            //position and velocity
            TermVectorPtr symPos = Symbolic::Vect<Vector2D, scalar>::
                create(baseX, baseY);
            TermPtr symAngle = Symbol::create(
                Symbolic::BaseSymbol::zero());
            TermVectorPtr symPosVel = 
                symPos->derivate(time);
            TermPtr symAngleVel = Symbol::create(
                Symbolic::BaseSymbol::zero());

            //Initalize Symblos and compute 
            //lagrangian symbolic expression
            Body::initSymbols(
                symPos, symAngle, symPosVel, symAngleVel);
        }
};

}
}

#endif

