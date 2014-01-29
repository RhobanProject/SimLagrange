#ifndef LEPH_SIMMECHA_ANGULARJOINT_HPP
#define LEPH_SIMMECHA_ANGULARJOINT_HPP

#include "SimMecha/src/Joint.hpp"

namespace Leph {
namespace SimMecha {

/**
 * AngularJoint
 */
class AngularJoint : public Joint
{
    public:
        
        /**
         * Initialization with Joint position on the two
         * given Bodies and reference (zero) angles
         * and Symbolic degree of freedom
         */
        AngularJoint(
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
                Symbolic::Add<scalar>::create(
                    dof, 
                    angleRootSym));
            //Build up Leaf Body orientation
            TermPtr resultAngle = Symbolic::Add<scalar>::create(
                angle1,
                Constant::create(-angleLeaf));
            
            //Joint anchor on Root Body and Leaf Body
            TermVectorPtr p1 = Symbolic::Add<Vector2D>::create(
                root.getSymPosition(),
                Symbolic::Rotation<Vector2D, scalar>::create(
                    posRootSym,
                    root.getSymAngle()));
            //Build up center of Leaf Body 
            TermVectorPtr resultPos = Symbolic::Add<Vector2D>::create(
                p1,
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
            (void)posLeaf;
            (void)angleLeaf;

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

