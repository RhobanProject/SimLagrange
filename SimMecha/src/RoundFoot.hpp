#ifndef LEPH_SIMMECHA_ROUNDFOOT_HPP
#define LEPH_SIMMECHA_ROUNDFOOT_HPP

#include "SimMecha/src/Joint.hpp"
#include <stdio.h>
#include "Symbolic/src/Bounder.hpp"
#include <functional>

//Let's annoy Leph
#define COS(x) Symbolic::Polar<scalar, scalar>::create(x)
#define SIN(x) Symbolic::PolarInv<scalar, scalar>::create(x)



namespace Leph {
namespace SimMecha {

/**
 * RoundFoot
 */
class RoundFoot : public Joint
{
    public:

    scalar _sr; //radius
    scalar _sgamma; //ground slope
    TermPtr _r;
    TermPtr _gamma;
    scalar _scontactPos; //coordinate of the contact point (angle/leg orientation)
    TermPtr _contactPos;

    TermPtr _initposX; //TODO find a way to reset
    TermPtr _initposY;


        /**
         * Initialization with Joint position on the two
         * given Bodies and reference (zero) angles
         * and Symbolic degree of freedom
         */

        RoundFoot(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof, scalar r, scalar gamma, scalar contactPos) :
            Joint(bodyRoot, posRoot, angleRoot,
                bodyLeaf, posLeaf, angleLeaf, dof)
        {

            _r=Constant::create(r);
            _gamma=Constant::create(gamma);
            _contactPos=Constant::create(contactPos);
            _sr=r;
            _sgamma=gamma;
            _scontactPos=contactPos;
            _initposX = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(_contactPos),COS(_gamma));
            _initposY = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(_contactPos),SIN(_gamma));




        }


    void setCustomScalar(scalar val)
        {

            // std::cout<<"debug"<<std::endl;
            TermPtr contactPos=Constant::create(val);
            // std::cout<<"debug1"<<std::endl;
            // _scontactPos=val;
            // std::cout<<"debug2"<<std::endl;
            _initposX = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(contactPos),COS(_gamma));
            _initposY = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(contactPos),SIN(_gamma));

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
                create(Vector2D(0.0, 1.0));

            //Get Joint degree of freedom
            SymbolPtr dof = Joint::getDof();

            TermPtr alpha=Symbolic::Frac<scalar>::create(dof,_r);



            TermPtr angle_root = Symbolic::Add<scalar>::create(root.getSymAngle(),angleRootSym);

                //Build up Leaf Body orientation
            // TermPtr resultAngle = Symbolic::Add<scalar>::create(
            //     angle1,
            //     Symbolic::Add<scalar>::create(Constant::create(-angleLeaf),_phi)); //Offset

            TermPtr resultAngle = Symbolic::Add<scalar>::create(
                alpha,
                Constant::create(-angleLeaf)); //Offset

            //Joint anchor on Root Body and Leaf Body
            TermVectorPtr p1 = Symbolic::Add<Vector2D>::create(
                root.getSymPosition(),
                Symbolic::Rotation<Vector2D, scalar>::create(
                    posRootSym,
                    Symbolic::Add<scalar>::create(root.getSymAngle(),alpha)));

            // scalar init_x=_system._statePosition.at(0);
            TermPtr dx = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(dof),COS(_gamma));

            dx=Symbolic::Add<scalar>::create(dx,Symbolic::Minus<scalar>::create(_initposX));

            TermPtr dy = Symbolic::Mult<scalar,scalar,scalar>::create(Symbolic::Minus<scalar>::create(dof),SIN(_gamma));
            dy=Symbolic::Add<scalar>::create(dy,Symbolic::Minus<scalar>::create(_initposY));
            // dy=Symbolic::Add<scalar>::create(dy,Symbolic::Minus<scalar>::create(SIN(resultAngle)));
            // dy=Symbolic::Add<scalar>::create(dy,SIN(resultAngle));


            TermVectorPtr dxdy=Symbolic::Vect<Vector2D, scalar>::create(dx,dy);
            p1=Symbolic::Add<Vector2D>::create(p1,dxdy); //old

                                                               // ConstantVector::create(Vector2D(Symbolic::Mult<scalar,scalar,scalar>::create(dof,COS(_gamma)), Symbolic::Mult<scalar,scalar,scalar>::create(dof,SIN(_gamma)) )));


            TermVectorPtr radvect = ConstantVector::
                create(Vector2D(0.0, _sr));


                //Joint anchor on Leaf Body
            TermVectorPtr p2 = Symbolic::Add<Vector2D>::create(
                p1,
                    Symbolic::Rotation<Vector2D, scalar>::create(
                        radvect,//unitySym,
                        angle_root));

            // p2=Symbolic::Add<Vector2D>::create(p2,dxdy);
            //Build up center of Leaf Body
            TermVectorPtr resultPos = Symbolic::Add<Vector2D>::create(
                p2, //p2
                Symbolic::Rotation<Vector2D, scalar>::create(
                    Symbolic::Minus<Vector2D>::create(posLeafSym),
                    resultAngle));


            //Set up Leaf Symbols
            leaf.initSymbols(
                resultPos, //p1
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

            // Vector2D pos = posRoot + Vector2D::rotate(
            //     Joint::getPosRoot(), angleRoot);

            Vector2D pos = posLeaf + Vector2D::rotate(
                Joint::getPosLeaf(), angleLeaf);

            // std::cout<<"posleaf: "<<posLeaf<<" "<<pos<<std::endl;

            // viewer.drawJoint(pos.x(), pos.y(),
            //     (Joint::getAngleRoot()+angleRoot)*180.0/M_PI,
            //     (value)*180.0/M_PI);

            // pos = posRoot + Vector2D::rotate(
            //     Joint::getPosRoot(), angleRoot);

            double ori=(Joint::getAngleRoot()+angleRoot);

            Vector2D rot;

                //Draw the cam
            Vector2D pos_r=pos+Vector2D(_sr,0);
            // Vector2D pos_l=pos;
            // double x=0.0;
            // double y=0.0;

            double theta=0.0;

            // Leph::Symbolic::Bounder bounder;
            // SymbolPtr xs = Symbol::create("x");


            viewer.drawCircle(pos.x(),pos.y(),0.03,sf::Color(255,255,255,255)); //center


            for(int i=0;i<100;i++)
            {


                theta+=2.0*M_PI/100.0;
                viewer.drawSegmentByEnd(pos_r.x(),pos_r.y(), pos.x()+_sr*cos(theta),pos.y()+_sr*sin(theta),0.01,sf::Color(200,200,200,100));
                pos_r=Vector2D(pos.x()+_sr*cos(theta),pos.y()+_sr*sin(theta));

                    /*
                xs->reset();
                bounder.setValue(xs,x);
                // y=F(x);
                y=_F(xs)->evaluate(bounder);

                // std::cout<<"x: "<<x<<" "<<xs->evaluate(bounder)<<" y: "<<y<<std::endl;


                rot=Vector2D::rotate(Vector2D(x,y),ori);

                viewer.drawSegmentByEnd(pos_r.x(),pos_r.y(), pos.x()+rot.x(),pos.y()+rot.y() ,0.01,sf::Color(200,200,200,100));
                pos_r=Vector2D(pos.x()+rot.x(), pos.y()+rot.y());

                xs->reset();
                bounder.setValue(xs,-x);
                y=_F(xs)->evaluate(bounder);
                // y=F(-x);
                rot=Vector2D::rotate(Vector2D(-x,y),ori);

                viewer.drawSegmentByEnd(pos_l.x(),pos_l.y(), pos.x()+rot.x(),pos.y()+rot.y() ,0.01,sf::Color(200,200,200,100));
                pos_l=Vector2D(pos.x()+rot.x(), pos.y()+rot.y());
                    */

            }


                /*
            viewer.drawSegment(pos.x(),pos.y(),_sH*2.0,((Joint::getAngleRoot()+angleRoot)+M_PI/2.0)*180.0/M_PI,0.05,sf::Color(255,255,255,100));

                //Draw the lever
            viewer.drawSegment(posLeaf.x(),posLeaf.y(),_sH,(angleLeaf-M_PI/2.0)*180.0/M_PI,0.01,sf::Color(255,0,255,100));
            viewer.drawCircle(posLeaf.x()+_sH*cos(angleLeaf-M_PI/2.0),posLeaf.y()+_sH*sin(angleLeaf-M_PI/2.0),0.03,sf::Color(255,0,255,100));
            viewer.drawCircle(pos.x(),pos.y(),0.03,sf::Color(255,255,255,255));
                */
        }
};

}
}

#endif
