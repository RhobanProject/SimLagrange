#ifndef LEPH_SIMMECHA_ANGULARSPRING_HPP
#define LEPH_SIMMECHA_ANGULARPRING_HPP

#include <cmath>
#include "SimMecha/src/AngularJoint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * AngularSpring
 */
class AngularSpring : public AngularJoint
{
    public:

        /**
         * Custom Spring law function
         */

        std::function<TermPtr(TermPtr)> _F;

        /**
         * Constant for Hooke's law
         */
        TermPtr _K, _l0;


        /**
         * Initialization with Joint position on the two
         * given Bodies and reference orientation angles
         * and Symbolic degree of freedom
         */
        AngularSpring(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, std::function<TermPtr(TermPtr)> F) :
            AngularJoint(bodyRoot, posRoot, angleRoot,
                        bodyLeaf, posLeaf, angleLeaf, dof)
        {
            _F=F;
            _K=Constant::create(0.0);
            _l0=Constant::create(0.0);
        }

        AngularSpring(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, scalar K, scalar l0) :
            AngularJoint(bodyRoot, posRoot, angleRoot,
                        bodyLeaf, posLeaf, angleLeaf, dof)
        {
            _K=Constant::create(K);
            _l0=Constant::create(l0);

            _F= [this](TermPtr x) -> TermPtr
                {
                        //Hooke law
                    return Symbolic::Mult<scalar, scalar, scalar>::create(Symbolic::Frac<scalar>::create(this->_K,Constant::create(2.0)), Symbolic::Pow<scalar>::create(Symbolic::Sub<scalar>::create(x,this->_l0) , 2.0));

                };
        }


        inline virtual void computeLagrangian()
        {
                //_F() returns the potential energy
                //Here the Lagrangian is only L=-Ep
            setLagrangian(Symbolic::Minus<scalar>::create(_F(getDof())));
        }


        /**
         * @Inherit
         */

        inline virtual void draw(SimViewer::SimViewer& viewer,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
            scalar value)
        {
            Joint::draw(viewer, posRoot, angleRoot,
                posLeaf, angleLeaf, value);

            Vector2D pos = posRoot + Vector2D::rotate(
                Joint::getPosRoot(), angleRoot);

            viewer.drawJoint(pos.x(), pos.y(),
                (Joint::getAngleRoot()+angleRoot)*180.0/M_PI,
                (value)*180.0/M_PI);


                //let's draw a rotational spring
            Vector2D tmppos=pos;
            Vector2D tmpnew=tmppos;
            scalar angle=Joint::getAngleRoot()+angleRoot+M_PI/2.0;
            scalar r=0.0;

            for(int i=0;i<100;i++)
            {
                r+=0.25/100.0;
                angle+=2.0*(2.0*M_PI+value/2.0)/100.0;
                tmpnew=Vector2D(pos.x()+r*cos(angle),pos.y()+r*sin(angle));
                viewer.drawSegmentByEnd(tmppos.x(),tmppos.y(), tmpnew.x(),tmpnew.y(),0.01,sf::Color(200,200,200,100));
                tmppos=tmpnew;
            }
        }


};

}
}

#endif
