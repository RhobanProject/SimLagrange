#ifndef LEPH_SIMMECHA_CAMJOINTINVERTED_HPP
#define LEPH_SIMMECHA_CAMJOINTINVERTED_HPP

#include "SimMecha/src/Joint.hpp"
#include "Symbolic/src/Bounder.hpp"
#include <functional>

#include <stdio.h>

//Let's annoy Leph
#define COS(x) Symbolic::Polar<scalar, scalar>::create(x)
#define SIN(x) Symbolic::PolarInv<scalar, scalar>::create(x)

namespace Leph {
namespace SimMecha {

/**
 * CamJointInverted
 */
class CamJointInverted : public Joint
{
    public:


    TermPtr _phi, _H;
    scalar _sphi, _sH;
    std::function<TermPtr(TermPtr)> _F;

        /**
         * Initialization with Joint position on the two
         * given Bodies and reference (zero) angles
         * and Symbolic degree of freedom
         */
        CamJointInverted(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, std::function<TermPtr(TermPtr)> F, scalar H, scalar phi) :
            Joint(bodyRoot, posRoot, angleRoot,
                bodyLeaf, posLeaf, angleLeaf, dof)
        {

            // _a=Constant::create(a);
            // _b=Constant::create(b);
            _H=Constant::create(H);
            _phi=Constant::create(phi);

            _F=F;
            // _sa=a;
            // _sb=b;
            _sH=H;
            _sphi=phi;
        }


        /**
         *
         * Function F describing the cam shape in cartesian
         *
         */

    /*
        TermPtr F(TermPtr x)
        {
            return Symbolic::Add<scalar>::create(Symbolic::Mult<scalar, scalar, scalar>::create(_a,x),Symbolic::Mult<scalar, scalar, scalar>::create(_b,Symbolic::Pow<scalar>::create(x,2)));


        }

        scalar F(scalar x)
        {
            return _sa*x+_sb*x*x;
        }
*/

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
                create(Vector2D(0.0, 1.0));

            //Get Joint degree of freedom
            SymbolPtr dof = Joint::getDof();

            //Sum up angle
            TermPtr angle1 = Symbolic::Add<scalar>::create(
                root.getSymAngle(),
                Symbolic::Add<scalar>::create(
                    dof,
                    angleRootSym));

            TermPtr angle_root = Symbolic::Add<scalar>::create(root.getSymAngle(),angleRootSym);

                //Build up Leaf Body orientation
            // TermPtr resultAngle = Symbolic::Add<scalar>::create(
            //     angle1,
            //     Symbolic::Add<scalar>::create(Constant::create(-angleLeaf),_phi)); //Offset

            TermPtr resultAngle = Symbolic::Add<scalar>::create(
                angle1,
                Constant::create(-angleLeaf)); //Offset

            //Joint anchor on Root Body and Leaf Body
            TermVectorPtr p1 = Symbolic::Add<Vector2D>::create(
                root.getSymPosition(),
                Symbolic::Rotation<Vector2D, scalar>::create(
                    posRootSym,
                    root.getSymAngle()));


            TermPtr z=Symbolic::Add<scalar>::create(_F(Symbolic::Mult<scalar, scalar, scalar>::create(Symbolic::Minus<scalar>::create(_H),SIN(Symbolic::Add<scalar>::create(dof,_phi)))),Symbolic::Mult<scalar, scalar, scalar>::create(_H,COS(Symbolic::Add<scalar>::create(dof,_phi))));


                //Joint anchor on Leaf Body
            // TermVectorPtr p2 = Symbolic::Add<Vector2D>::create(
            //     p1,
            //     Symbolic::Mult<Vector2D, scalar, Vector2D>::create(
            //         z,
            //         Symbolic::Rotation<Vector2D, scalar>::create(
            //             unitySym,
            //             angle_root)));

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
                // Symbolic::Add<scalar>::create(resultAngle,_phi),
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

            Vector2D pos_j = posRoot + Vector2D::rotate(
                Joint::getPosRoot(), angleRoot);

            viewer.drawJoint(pos_j.x(), pos_j.y(),
                (Joint::getAngleRoot()+angleRoot)*180.0/M_PI,
                (value)*180.0/M_PI);

            Vector2D pos = posLeaf + Vector2D::rotate(
                Joint::getPosLeaf(), angleLeaf);

            // viewer.drawJoint(pos.x(), pos.y(),
            //     (Joint::getAngleRoot()+angleRoot)*180.0/M_PI,
            //     (value)*180.0/M_PI);


            double ori=(Joint::getAngleLeaf()+angleLeaf);

            Vector2D rot;

                //Draw the cam
            Vector2D pos_r=pos;
            Vector2D pos_l=pos;
            double x=0.0;
            double y=0.0;

            Leph::Symbolic::Bounder bounder;
            SymbolPtr xs = Symbol::create("x");


            for(int i=0;i<100;i++)
            {
                x+=.3/100.0;
                xs->reset();
                bounder.setValue(xs,x);
                // y=F(x);
                y=_F(xs)->evaluate(bounder);



                rot=Vector2D::rotate(Vector2D(x,y),ori+M_PI);

                viewer.drawSegmentByEnd(pos_r.x(),pos_r.y(), pos.x()+rot.x(),pos.y()+rot.y() ,0.01,sf::Color(200,200,200,100));
                pos_r=Vector2D(pos.x()+rot.x(), pos.y()+rot.y());
                xs->reset();
                bounder.setValue(xs,-x);
                y=_F(xs)->evaluate(bounder);
                // y=F(-x);
                rot=Vector2D::rotate(Vector2D(-x,y),ori+M_PI);

                viewer.drawSegmentByEnd(pos_l.x(),pos_l.y(), pos.x()+rot.x(),pos.y()+rot.y() ,0.01,sf::Color(200,200,200,100));
                pos_l=Vector2D(pos.x()+rot.x(), pos.y()+rot.y());


            }



            viewer.drawSegment(pos.x(),pos.y(),_sH*2.0,(Joint::getAngleLeaf()+angleLeaf-M_PI/2.0)*180.0/M_PI,0.05,sf::Color(255,255,255,100));


            viewer.drawSegment(pos_j.x(),pos_j.y(),_sH,((Joint::getAngleRoot()+angleRoot)+M_PI/2.0-_sphi)*180.0/M_PI,0.01,sf::Color(255,0,255,100));

            viewer.drawCircle(pos_j.x()+_sH*cos((Joint::getAngleRoot()+angleRoot)+M_PI/2.0-_sphi),pos_j.y()+_sH*sin((Joint::getAngleRoot()+angleRoot)+M_PI/2.0-_sphi),0.03,sf::Color(255,0,255,100));

            viewer.drawCircle(pos.x(),pos.y(),0.03,sf::Color(255,255,255,255));

        }
};

}
}

#endif
