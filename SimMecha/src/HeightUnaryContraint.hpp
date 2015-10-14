#ifndef LEPH_SIMMECHA_HEIGHTUNARYCONSTRAINT_HPP
#define LEPH_SIMMECHA_HEIGHTUNARYCONSTRAINT_HPP

#include "SimMecha/src/UnaryConstraint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * HeightUnaryContraint
 */
class HeightUnaryContraint : public UnaryConstraint
{
    public:

        /**
         * Initialization
         */
        HeightUnaryContraint(Body& body, System& system, 
            scalar restitutionCoef, bool isFriction,
            scalar heightGround, const Vector2D& posInBody) :
            UnaryConstraint(body, system, restitutionCoef, isFriction),
            _heightGround(heightGround),
            _posInBody(posInBody)
        {
        }

    protected:
        
        /**
         * @Inherit
         */
        inline virtual bool computeCheckConstraint(
            Vector2D& point, Vector2D& dir, Vector2D& posInBody)
        {
            Vector2D centerPos = UnaryConstraint::_system
                ->evalPosition(*UnaryConstraint::_body);
            scalar centerAngle = UnaryConstraint::_system
                ->evalAngle(*UnaryConstraint::_body);

            Vector2D pos = centerPos 
                + Vector2D::rotate(_posInBody, centerAngle);

            point = pos;
            dir = Vector2D(0.0, 1.0);
            posInBody = _posInBody;
            if (pos.y() <= _heightGround) {
                return true;
            } else {
                return false;
            }
        }

    private:

        /**
         * Height minimum limit 
         * value (in global frame)
         */
        scalar _heightGround;

        /**
         * Position in Body coordinate
         * system of refence point used 
         * to check the Constraint
         */
        Vector2D _posInBody;
};

}
}

#endif

