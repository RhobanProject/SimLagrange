#ifndef LEPH_SIMMECHA_BASE_HPP
#define LEPH_SIMMECHA_BASE_HPP

#include "SimMecha/src/Body.hpp"

namespace Leph {
namespace SimMecha {

class System;

/**
 * Base
 *
 * Base class for pseudo rigid body
 * used as base of kinematic chain
 */
class Base : public Body
{
    public:

        /**
         * @Inherit
         */
        Base(SymbolPtr time) :
            Body(time)
        {
        }

        /**
         * @Inherit
         */
        inline virtual void draw(SimViewer::SimViewer& viewer, 
            const Vector2D& pos, scalar angle)
        {
            Body::draw(viewer, pos, angle);
            viewer.drawBase(pos.x(), pos.y());
        }
};

}
}

#endif

