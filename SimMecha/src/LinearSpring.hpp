#ifndef LEPH_SIMMECHA_LINEARSPRING_HPP
#define LEPH_SIMMECHA_LINEARSPRING_HPP

#include <cmath>
#include "SimMecha/src/LinearJoint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * LinearSpring
 */
class LinearSpring : public LinearJoint
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
        LinearSpring(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, std::function<TermPtr(TermPtr)> F) :
            LinearJoint(bodyRoot, posRoot, angleRoot,
                        bodyLeaf, posLeaf, angleLeaf, dof)
        {
            _F=F;
            _K=Constant::create(0.0);
            _l0=Constant::create(0.0);
        }

        LinearSpring(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, scalar K, scalar l0) :
            LinearJoint(bodyRoot, posRoot, angleRoot,
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

            Vector2D pos1 = posRoot + Vector2D::rotate(
                Joint::getPosRoot(), angleRoot);
            Vector2D pos2 = posLeaf + Vector2D::rotate(
                Joint::getPosLeaf(), angleLeaf);

            viewer.drawLinearJoint(pos1.x(), pos1.y(),
                pos2.x(), pos2.y(), value);
        }



};

}
}

#endif
