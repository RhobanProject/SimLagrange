#ifndef LEPH_SIMMECHA_LINEARJOINT_HPP
#define LEPH_SIMMECHA_LINEARJOINT_HPP

#include <cmath>
#include "SimMecha/src/Joint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * LinearJoint
 */
class LinearJoint : public Joint
{
    public:
        
        /**
         * Initialization with Joint position on the two
         * given Bodies and reference orientation angles
         * and Symbolic degree of freedom
         */
        LinearJoint(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof) :
            Joint(bodyRoot, posRoot, angleRoot, 
                bodyLeaf, posLeaf, angleLeaf, dof)
        {
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
                angleRootSym);
            //Build up the Leaf Body orientation
            TermPtr resultAngle = Symbolic::Add<scalar>::create(
                angle1,
                Constant::create(M_PI-angleLeaf));

            //Joint anchor on Root Body
            TermVectorPtr p1 = Symbolic::Add<Vector2D>::create(
                root.getSymPosition(),
                Symbolic::Rotation<Vector2D, scalar>::create(
                    posRootSym,
                    root.getSymAngle()));
            //Joint anchor on Leaf Body
            TermVectorPtr p2 = Symbolic::Add<Vector2D>::create(
                p1,
                Symbolic::Mult<Vector2D, scalar, Vector2D>::create(
                    dof,
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
                resultAngle->derivate(time),
                time);
        }
        
        /**
         * @Inherit
         */
        inline virtual void draw(SimViewer::SimViewer& viewer, 
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
            scalar value)
        {
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

