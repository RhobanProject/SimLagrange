#ifndef LEPH_SIMMECHA_HEIGHTUNARYCONSTRAINT_HPP
#define LEPH_SIMMECHA_HEIGHTUNARYCONSTRAINT_HPP

#include "SimMecha/src/UnaryConstraint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * HeightUnaryConstraint
 */
class HeightUnaryConstraint : public UnaryConstraint
{
    public:

        /**
         * Initialization
         */
        HeightUnaryConstraint(Body& body, System& system,
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




class Ground: public HeightUnaryConstraint
{
    public:

        /**
         * Function describing the ground (in global frame)
         *
         */

        std::function<float(float)> _F;
        /**
         * Position in Body coordinate
         * system of refence point used
         * to check the Constraint
         */

        Vector2D _posInBody;
        Vector2D _currentpos;
        bool _contact;

        /**
         * Initialization
         */
        Ground(Body& body, System& system,
            scalar restitutionCoef, bool isFriction,
            std::function<float(float)> F, const Vector2D& posInBody) :
            HeightUnaryConstraint(body, system, restitutionCoef, isFriction, 0.0, posInBody)
        {
            _F=F;
            _currentpos=Vector2D(0, 0);
            _contact=false;
            _posInBody=posInBody;
        }

        virtual void draw(Leph::SimViewer::SimViewer& viewer)
        {
            Vector2D pos_r=_currentpos;
            Vector2D pos_l=_currentpos;

            for(double xi=0.0; xi<5.0;xi+=0.01)
            {
                viewer.drawSegmentByEnd(pos_r.x(),_F(pos_r.x()), xi+pos_r.x(),_F(xi+pos_r.x()) ,0.02,sf::Color(255,255,255,255));
                pos_r=Vector2D(pos_r.x()+xi, _F(xi+pos_r.x()));
                viewer.drawSegmentByEnd(pos_l.x(),_F(pos_l.x()), pos_l.x()-xi,_F(pos_l.x()-xi) ,0.02,sf::Color(255,255,255,255));
                pos_l=Vector2D(pos_l.x()-xi, _F(pos_l.x()-xi));
            }

            if(_contact) //FIXME
            {
                viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,255));
            }
            else
                viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,100));

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
            _currentpos=pos;


            if (pos.y() <= _F(pos.x())) {
                _contact=true;
                return true;
            } else {
                _contact=false;
                return false;
            }
        }

    private:

};




}
}

#endif
