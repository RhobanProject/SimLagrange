
#if !defined(SIMPLEWALKERGROUND_HPP)
#define SIMPLEWALKERGROUND_HPP




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

class SimpleWalkerGround: public Ground
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

        SimpleWalkerGround(Body& body, System& system,
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

            // std::cout<<"C: "<<postCollision<<" "<<collision_delay<<std::endl;


                    //draw
            Vector2D centerPos = UnaryConstraint::_system
                ->evalPosition(*UnaryConstraint::_body);
            scalar centerAngle = UnaryConstraint::_system
                ->evalAngle(*UnaryConstraint::_body);

            Vector2D pos = centerPos
                + Vector2D::rotate(_posInBody, centerAngle);

            _currentpos=pos;
            scalar gamma=atan2(_F(1.0)-_F(0.0),1.0); //FIXME ground angle



            if((prev_isFootUpGround != isFootUpGround) ) //edge
            {
                isContactEnabled=true;
                // if(!supportChanged)
                //     isContactEnabled=!isContactEnabled;
                // std::cout<<"CONTACT TRIGGER: "<<isContactEnabled<<std::endl;
                // supportChanged=false;
            }
            else
                isContactEnabled=false;
            // supportChanged=false;
            // if(prev_isFootUpGround && !isFootUpGround )
            // {
            //     isContactEnabled=!isContactEnabled;
            //     std::cout<<"CONTACT TRIGGER: "<<isContactEnabled<<std::endl;

            //     // std::cout<<"CONTACT DISABLED"<<std::endl;
            //     // isContactEnabled=false;
            // }

            prev_isFootUpGround=isFootUpGround;


            if(pos.y()<=_F(pos.x()))
                isFootUpGround=false;
            if(pos.y()>_F(pos.x()))
                isFootUpGround=true;



            if(isInit)
            {
                if(pos.y()<=_F(pos.x()))
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


            // if(pos.y()<_F(pos.x()) && !isContactEnabled)
            //     return;


            if(fabs(_system->statePosition("q1")) > M_PI/2.0)
            {
                hasFallen=true;
                return;
            }

                //ERROR
            // if(_system->statePosition("q2")<0.0 && _system->stateVelocity("q2") < 0.0)
            // {
            //     hasFallen=true;
            //     return;
            // }

            /*

            if(_system->statePosition("q2")<0.0 && _system->stateVelocity("q2") > 0.0)
                {
                // std::cout<<"Q2: "<<_system->statePosition("q2")<<std::endl;

                postCollision=true;
                collision_delay=_COLLISION_COOLDOWN;

                //     //draw
                // Vector2D centerPos = UnaryConstraint::_system
                //     ->evalPosition(*UnaryConstraint::_body);
                // scalar centerAngle = UnaryConstraint::_system
                //     ->evalAngle(*UnaryConstraint::_body);

                // Vector2D pos = centerPos
                //     + Vector2D::rotate(_posInBody, centerAngle);

                // _currentpos=pos;

                return;
            }
            */
            /*
            // if (!getConstraint(point, dir, posInBody) || postCollision)
            if (computeCheckConstraint(point, dir, posInBody) && postCollision)
            // if (postCollision)
            {


                if(collision_delay-- == 0)
                {
                    collision_delay=_COLLISION_COOLDOWN;
                    postCollision=false;
                }

                std::cout<<"COLLISION COOLDOWN"<<std::endl;
                                    //draw
                // Vector2D centerPos = UnaryConstraint::_system
                //     ->evalPosition(*UnaryConstraint::_body);
                // scalar centerAngle = UnaryConstraint::_system
                //     ->evalAngle(*UnaryConstraint::_body);

                // Vector2D pos = centerPos
                //     + Vector2D::rotate(_posInBody, centerAngle);

                // _currentpos=pos;

                return;
            }
            */

            /*
            if (computeCheckConstraint(point, dir, posInBody))
            {
                    //draw
                Vector2D centerPos = UnaryConstraint::_system
                ->evalPosition(*UnaryConstraint::_body);
                scalar centerAngle = UnaryConstraint::_system
                    ->evalAngle(*UnaryConstraint::_body);

                Vector2D pos = centerPos
                    + Vector2D::rotate(_posInBody, centerAngle);

                // _currentpos=pos;

                std::cout<<"pos: "<<pos.y()<<" ground: "<<_F(pos.x())<<" "<<(pos.y()-_F(pos.x()))<<std::endl;


                // if(fabs(pos.y()-_F(pos.x()))>0.01)
                // {
                //     std::cout<<"FALL: under ground"<<std::endl;
                //     hasFallen=true;
                //     return;
                // }



                // if(fabs(_system->statePosition("q1")-gamma)>1.0 )
                // {
                //     hasFallen=true;
                //     return;
                // }

                // //allow to go under ground
                // if(point.y()<_F(point.x()) && fabs(_system->statePosition("q2"))<0.3)
                // {
                //     if(fabs(_system->stateVelocity("q2"))>0.01)
                //     {
                //         std::cout<<"under ground"<<std::endl;
                //         return;
                //     }
                //     else
                //     {
                //         hasFallen=true;
                //         std::cout<<"FALL UNDER GROUND"<<std::endl;
                //         return;
                //     }

                // }


                // if(fabs(pos.y()-_F(pos.x()))<0.01 && _system->statePosition("q2")>=0.3)
                // {
                //     std::cout<<"FALL"<<std::endl;
                //     hasFallen=true;
                //     return;
                // }


                if(!isContactEnabled)
                {
                    std::cout<<"CONTACT DISABLED"<<std::endl;

                    return;
                }



            }
            */

            // std::cout<<"Q2: "<<_system->statePosition("q2")<<std::endl;

            /*
            if(fabs(_system->statePosition("q2")) < 0.01 || _system->statePosition("q2")<0.0)
            {
                hasFallen=true;
                return;
            }
            */

            if (!computeCheckConstraint(point, dir, posInBody))
                return;

            // std::cout<<"CONTACT: "<<isContactEnabled<<" phase: "<<phase<<std::endl;


            if(isContactEnabled)
            {
                /*
                if(phase<2)
                {
                    contactActivated=false;
                    phase++;
                    return;
                }
                else
                {
                    contactActivated=true;
                    phase=0;
                }
                */

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

            // std::cout<<"Collision"<<std::endl;


            nbStep++;
            stepDist+=(_currentpos.x()-fabs(prevStep));
            prevStep=fabs(_currentpos.x());
            meanStep=stepDist/nbStep;


                //manage the Heelstrike
            _system->getBase().setPos(point); //get the base to the new contact pos


            // if(point.y()- _F(point.x())<-0.01)
            // {
            //     std::cout<<"FALL UNDER GROUND "<<point.y()<<" "<<_F(point.x())<<std::endl;
            //     hasFallen=true;
            //     return;
            // }



            //     //velocities

            // impactAngle+=_system->statePosition("q1");
            // impactVel+=_system->statePosition("q1");

            // impactAngleQ2+=_system->statePosition("q2");
            // impactVelQ2+=_system->statePosition("q2");


            // meanImpactAngle=impactAngle/nbStep;
            // if(nbStep>1)
            //     devImpactAngle=0.0;
            // else
            //     devImpactAngle=1.0;
            // devImpactAngle+=fabs(meanImpactAngle-_system->statePosition("q1"));


            // scalar gamma=atan2(-0.21,1.0); //FIXME ground angle

            scalar theta=_system->statePosition("q1")-gamma;
            scalar theta_dot=_system->stateVelocity("q1");
            scalar psi_dot=cos(2.0*theta)*(1.0-cos(2.0*theta))*theta_dot;

            theta_dot=cos(2.0*theta)*theta_dot;

            _system->stateVelocity("q1")=theta_dot;
            _system->stateVelocity("q2")=psi_dot;

            std::cout<<"COLLISION Q1: "<<_system->statePosition("q1")<<" dQ1: "<<_system->stateVelocity("q1")<<" Q2: "<<_system->statePosition("q2")<<" dQ2: "<<_system->stateVelocity("q2")<<" POS: "<<_currentpos.x()<<std::endl;


            _system->statePosition("q1")=_system->evalAngle(*_body);
            _system->statePosition("q2")=-_system->statePosition("q2");



                //velocities

            impactAngle=_system->statePosition("q1");
            impactVel=_system->stateVelocity("q1");

            impactAngleQ2=_system->statePosition("q2");
            impactVelQ2=_system->stateVelocity("q2");


            meanImpactAngle=impactAngle/nbStep;
            if(nbStep>1)
                devImpactAngle=0.0;
            else
                devImpactAngle=1.0;
            devImpactAngle+=fabs(meanImpactAngle-_system->statePosition("q1"));


            _system->initSymbols();

            // isContactEnabled=false;
            // prev_isFootUpGround=false;
            // isFootUpGround=false;
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
            // if(isContactEnabled)
            //     A=255;
            // else
            //     A=100;

            A=255;
                // viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,100));

            viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(R,G,B,A));
                //the leg
            viewer.drawSegmentByEnd(_currentpos.x(),_currentpos.y(),_system->evalPosition(*_body).x() ,_system->evalPosition(*_body).y() ,0.05,sf::Color(0,255,0,50));

        }
};

#endif
