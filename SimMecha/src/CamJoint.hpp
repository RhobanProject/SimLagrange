#ifndef LEPH_SIMMECHA_CAMJOINT_HPP
#define LEPH_SIMMECHA_CAMJOINT_HPP

#include "SimMecha/src/Joint.hpp"

//Let's annoy Leph
#define COS(x) Symbolic::Polar<scalar, scalar>::create(x)
#define SIN(x) Symbolic::PolarInv<scalar, scalar>::create(x)

namespace Leph {
namespace SimMecha {

/**
 * CamJoint
 */
class CamJoint : public Joint
{
    public:


    SymbolPtr _a, _b, _phi, _H;

        /**
         * Initialization with Joint position on the two
         * given Bodies and reference (zero) angles
         * and Symbolic degree of freedom
         */
        CamJoint(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, scalar a, scalar b, scalar H, scalar phi) :
            Joint(bodyRoot, posRoot, angleRoot,
                bodyLeaf, posLeaf, angleLeaf, dof)
        {
            _a=Constant::create(a);
            _b=Constant::create(b);
            _H=Constant::create(H);
            _phi=Constant::create(phi);
        }


        /**
         *
         * Function F describing the cam shape in cartesian
         *
         */

        SymbolPtr F(SymbolPtr x)
        {
            return Symbolic::Add<scalar>::create(Symbolic::Mult<scalar, scalar, scalar>::create(_a,x),Symbolic::Mult<scalar, scalar, scalar>::create(_b,Symbolic::Pow<scalar>::create(x,2)));


        }

        /**
         * @Inherit
         */
        inline virtual void computeSymTransformation(SymbolPtr time)
        {
            //Get linked Bodies
            Body& root = Joint::getBodyRoot();
            Body& leaf = Joint::getBodyLeaf();

            //Get Joint structure
            const Vector2D& posRoot = Joint::getPosRoot();
            const Vector2D& posLeaf = Joint::getPosLeaf();
            scalar angleRoot = Joint::getAngleRoot();
            scalar angleLeaf = Joint::getAngleLeaf();

            //Build Joint structure Constants
            TermPtr angleRootSym = Constant::create(angleRoot);
            TermPtr angleLeafSym = Constant::create(angleLeaf);
            TermVectorPtr posRootSym = ConstantVector::create(posRoot);
            TermVectorPtr posLeafSym = ConstantVector::create(posLeaf);

            //Build unity vector
            TermVectorPtr unitySym = ConstantVector::
                create(Vector2D(1.0, 0.0));

            //Get Joint degree of freedom
            SymbolPtr dof = Joint::getDof();

            //Sum up angle
            TermPtr angle1 = Symbolic::Add<scalar>::create(
                root.getSymAngle(),
                Symbolic::Add<scalar>::create(
                    dof,
                    angleRootSym));

                //Build up Leaf Body orientation
            TermPtr resultAngle = Symbolic::Add<scalar>::create(
                angle1,
                Symbolic::Add<scalar>::create(Constant::create(-angleLeaf),_phi)); //Offset

            //Joint anchor on Root Body and Leaf Body
            TermVectorPtr p1 = Symbolic::Add<Vector2D>::create(
                root.getSymPosition(),
                Symbolic::Rotation<Vector2D, scalar>::create(
                    posRootSym,
                    root.getSymAngle()));

            SymbolPtr z=Symbolic::Add<scalar>::create(F(Symbolic::Mult<scalar, scalar, scalar>::create(_H,SIN(dof))),Symbolic::Mult<scalar, scalar, scalar>::create(_H,COS(dof)));


                //Joint anchor on Leaf Body
            TermVectorPtr p2 = Symbolic::Add<Vector2D>::create(
                p1,
                Symbolic::Mult<Vector2D, scalar, Vector2D>::create(
                    z,
                    Symbolic::Rotation<Vector2D, scalar>::create(
                        unitySym,
                        angle1)));

            //Build up center of Leaf Body
            TermVectorPtr resultPos = Symbolic::Add<Vector2D>::create(
                p2,
                Symbolic::Rotation<Vector2D, scalar>::create(
                    Symbolic::Minus<Vector2D>::create(posLeafSym),
                    resultAngle));

            //Set up Leaf Symbols
            leaf.initSymbols(
                resultPos,
                resultAngle,
                resultPos->derivate(time),
                resultAngle->derivate(time));
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
        }
};

}
}

#endif
