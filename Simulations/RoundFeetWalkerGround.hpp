
#if !defined(ROUNDFEETWALKERGROUND_HPP)
#define ROUNDFEETWALKERGROUND_HPP




#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"
#include "SimMecha/src/Constraint.hpp"
#include "SimMecha/src/UnaryConstraint.hpp"
#include "SimMecha/src/BinaryConstraint.hpp"
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/terms.h"
#include "SimMecha/src/Simulation.h"

#include "SimMecha/src/HeightUnaryConstraint.hpp"

#define _COLLISION_COOLDOWN 2

using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;

class RoundFeetGround: public Ground
{

    public:

    bool postCollision;
    int collision_delay;
    bool hasFallen;
    int nbStep;
    double impactAngle;
    double impactVel;

    double impactAngleQ2;
    double impactVelQ2;


    double meanImpactAngle;
    double devImpactAngle;
    double meanStep;
    double stepDist;
    double prevStep;

    bool isFootUpGround;
    bool prev_isFootUpGround;
    bool isContactEnabled;
    bool isInit;
    int phase;
    bool supportChanged;
    bool contactActivated;
    Vector2D contactPoint;

    double feetRadius; //FIXME

        RoundFeetGround(Body& body, System& system,
                 scalar restitutionCoef, bool isFriction,
                 std::function<float(float)> F, const Vector2D& posInBody):
            Ground(body, system, restitutionCoef, isFriction, F, posInBody)
        {
            postCollision=false;
            hasFallen=false;
            collision_delay=_COLLISION_COOLDOWN;
            nbStep=0;
            impactAngle=0.0;
            meanImpactAngle=0.0;
            devImpactAngle=0.0;
            meanStep=0.0;
            prevStep=0.0;
            stepDist=0.0;
            impactVel=0.0;
            impactAngleQ2=0.0;
            impactVelQ2=0.0;
            isFootUpGround=true;
            isContactEnabled=true;
            prev_isFootUpGround=true;
            isInit=true;
            phase=1;
            supportChanged=false;
            contactActivated=false;
        }

    inline void handle()
        {
            Vector2D point;
            Vector2D dir;
            Vector2D posInBody;



                    //draw
            Vector2D centerPos = UnaryConstraint::_system
                ->evalPosition(*UnaryConstraint::_body);
            scalar centerAngle = UnaryConstraint::_system
                ->evalAngle(*UnaryConstraint::_body);

            Vector2D pos = centerPos
                + Vector2D::rotate(_posInBody, centerAngle);

            _currentpos=pos;
            scalar gamma=atan2(_F(1.0)-_F(0.0),1.0); //FIXME ground angle


            bool contact=computeCheckConstraint(point,dir,posInBody);

            if((prev_isFootUpGround != isFootUpGround) ) //edge
            {
                isContactEnabled=true;

            }
            else
                isContactEnabled=false;

            prev_isFootUpGround=isFootUpGround;


            // if(pos.y()<=_F(pos.x()))
            //     isFootUpGround=false;
            // if(pos.y()>_F(pos.x()))
            //     isFootUpGround=true;

            if(contact)
                isFootUpGround=false;
            else
                isFootUpGround=true;


            if(isInit)
            {
                // if(pos.y()<=_F(pos.x()))
                if(contact)
                {
                    prev_isFootUpGround=false;
                    isFootUpGround=false;
                    isContactEnabled=false;
                }
                else
                {
                    prev_isFootUpGround=false;
                    isFootUpGround=true;
                    isContactEnabled=true;
                }

                prev_isFootUpGround=false;
                isFootUpGround=false;
                isContactEnabled=false;
                isInit=false;
            }



            if(fabs(_system->statePosition("q1")/feetRadius) > M_PI/2.0)
            {

                hasFallen=true;
                return;
            }

            if (!computeCheckConstraint(point, dir, posInBody))
                return;



            if(isContactEnabled)
            {

                if(_system->statePosition("q2")<0.0)
                    return;
            }
            else
            {
                contactActivated=false;
                return;
            }


            getConstraint(point, dir, posInBody);


            postCollision=true;


            nbStep++;
            stepDist+=(_currentpos.x()-fabs(prevStep));
            prevStep=fabs(_currentpos.x());
            meanStep=stepDist/nbStep;


            Vector2D newbase=point;
            Vector2D oldbase=_system->evalPosition(_system->getBase());
            // newbase.y()+=feetRadius; //FIXME!!!!!

                //manage the Heelstrike
            _system->getBase().setPos(newbase); //get the base to the new contact pos



            scalar theta=_system->statePosition("q1")/feetRadius-gamma;
            scalar theta_dot=_system->stateVelocity("q1")/feetRadius;

                //get the hip
            Vector2D hippos=_system->evalPosition(*(_system->getBodies()[1])); //should be the hip

                //TODO
            scalar psi=(Vector2D::direction(hippos-contactPoint)-M_PI/2.0-gamma)*2.0;

            // std::cout<<"DEBUG :"<<contactPoint<<" "<<hippos<<" "<<psi<<" "<<_system->statePosition("q2")<<" "<<(Vector2D::direction(hippos-newbase)-M_PI/2.0-gamma)*2.0<<std::endl;

            // scalar psi=_system->statePosition("q2");


            // std::cout<<"DEBUG test: "<<2.0*theta<<" "<<_system->statePosition("q2")<<std::endl;
            //in fact at heelstrike, 2*theta=phi

            // scalar psi_dot=cos(2.0*theta)*(1.0-cos(2.0*theta))*theta_dot;
            // theta_dot=cos(2.0*theta)*theta_dot;

            scalar psi_dot=cos(psi)*(1.0-cos(psi))*theta_dot;
            theta_dot=cos(psi)*theta_dot;



            _system->stateVelocity("q1")=theta_dot*feetRadius;
            _system->stateVelocity("q2")=psi_dot;



                //raz curvilign integral
            // Joint* j=_system->getBase().getJointRoot();
            Joint* j=_system->getJoints()[0];
            j->setCustomScalar(_system->evalAngle(*_body)*feetRadius);

            _system->statePosition("q1")=_system->evalAngle(*_body)*feetRadius;

            _system->statePosition("q2")=-_system->statePosition("q2");

            std::cout<<"COLLISION Q1: "<<_system->statePosition("q1")<<" dQ1: "<<_system->stateVelocity("q1")<<" Q2: "<<_system->statePosition("q2")<<" dQ2: "<<_system->stateVelocity("q2")<<" POS: "<<_currentpos.x()<<" STEP: "<<nbStep<<std::endl;


                //velocities

            impactAngle=_system->statePosition("q1")/feetRadius;
            impactVel=_system->stateVelocity("q1")/feetRadius;

            impactAngleQ2=_system->statePosition("q2");
            impactVelQ2=_system->stateVelocity("q2");


            meanImpactAngle=impactAngle/nbStep;
            if(nbStep>1)
                devImpactAngle=0.0;
            else
                devImpactAngle=1.0;
            devImpactAngle+=fabs(meanImpactAngle-_system->statePosition("q1")/feetRadius);


            _system->initSymbols();

            supportChanged=true;

        }

        void draw(Leph::SimViewer::SimViewer& viewer)
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


            int R=0;
            int G=0;
            int B=0;
            int A=0;

            /*
            if(contactActivated) //FIXME
            {
                R=255;
                G=0;
            }
            else
            {
                R=0;
                G=255;
            }

            if(phase==0)
            {
                B=0;
            }
            if(phase==1)
            {
                B=255;
            }
            */

            if(isContactEnabled)
            {
                R=255;
                G=0;
            }
            else
            {
                R=0;
                G=255;
            }

            // if(isContactEnabled)
            //     A=255;
            // else
            //     A=100;

            A=100;
                // viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,100));

            // viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(R,G,B,A));
            viewer.drawCircle(_currentpos.x(),_currentpos.y(),feetRadius,sf::Color(R,G,B,A));

                //the leg
            viewer.drawSegmentByEnd(_currentpos.x(),_currentpos.y(),_system->evalPosition(*_body).x() ,_system->evalPosition(*_body).y() ,0.05,sf::Color(0,255,0,50));

        }


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
            dir = Vector2D(0.0, 1.0); //TODO normal
            posInBody = _posInBody;
            _currentpos=pos;

            double p=pos.x();
            double q=pos.y();
            double c=_F(0);
            double m=_F(1)-c; //slope
            double r=feetRadius;

            //quadratic equation
            //(m^2+1)x^2+2(mc−mq−p)x+(q^2−r^2+p^2−2cq+c^2)=0
            //A.x^2+B.x+C=0
            //so let see the determinant B^2-4AC

            double A=(pow(m,2)+1.0);
            double B=2.0*(m*c-m*q-p);
            double C=(pow(q,2)-pow(r,2)+pow(p,2)-2*c*q+pow(c,2));
            double det=pow(B,2)-4*A*C;

            // std::cout<<"DEBUG: "<<det<<" "<<m<<" "<<c<<" "<<r<<std::endl;

            if (det >= 0.0) {
            // if ((pos.y()-_F(pos.x())) <=0.0 && fabs(pos.y()-_F(pos.x()))<0.01 ) {
            // if ((pos.y()-_F(pos.x())) <=0.0) {
                _contact=true;

                contactPoint=Vector2D(min((-B+sqrt(det))/(2*A),(-B-sqrt(det))/(2*A)),m*point.x()+c);
                //sign=+ or -
                // point.x()=(-B+sign*sqrt(det))/(2*A);
                // point.y()=m*point.x()+c;

                return true;
            } else {
                _contact=false;
                return false;
            }
        }


};

#endif
