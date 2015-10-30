
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

#include "SimMecha/src/HeightUnaryConstraint.hpp"


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
    double meanImpactAngle;
    double devImpactAngle;
    double meanStep;
    double stepDist;
    double prevStep;

        SimpleWalkerGround(Body& body, System& system,
                 scalar restitutionCoef, bool isFriction,
                 std::function<float(float)> F, const Vector2D& posInBody):
            Ground(body, system, restitutionCoef, isFriction, F, posInBody)
        {
            postCollision=false;
            hasFallen=false;
            collision_delay=1;
            nbStep=0;
            impactAngle=0.0;
            meanImpactAngle=0.0;
            devImpactAngle=0.0;
            meanStep=0.0;
            prevStep=0.0;
            stepDist=0.0;
        }

    inline void handle()
        {
            Vector2D point;
            Vector2D dir;
            Vector2D posInBody;

            // std::cout<<"C: "<<postCollision<<" "<<collision_delay<<std::endl;

            // std::cout<<"Q1 "<<_system->statePosition("q1")<<std::endl;

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


            if(_system->statePosition("q2")<0.0 && _system->stateVelocity("q2") > 0.0)
            {
                // std::cout<<"SWING"<<std::endl;
                // std::cout<<"Q2: "<<_system->statePosition("q2")<<std::endl;

                postCollision=true;
                collision_delay=1;

                    //draw
                Vector2D centerPos = UnaryConstraint::_system
                    ->evalPosition(*UnaryConstraint::_body);
                scalar centerAngle = UnaryConstraint::_system
                    ->evalAngle(*UnaryConstraint::_body);

                Vector2D pos = centerPos
                    + Vector2D::rotate(_posInBody, centerAngle);

                _currentpos=pos;

                return;
            }

            if (!getConstraint(point, dir, posInBody) || postCollision) {


                if(collision_delay-- == 0)
                {
                    collision_delay=1;
                    postCollision=false;
                }

                return;
            }

            // std::cout<<"Q2: "<<_system->statePosition("q2")<<std::endl;

            if(fabs(_system->statePosition("q2")) < 0.01 || _system->statePosition("q2")<0.0)
            {
                hasFallen=true;
                return;
            }

            postCollision=true;
            std::cout<<"Collision"<<std::endl;
            nbStep++;

            stepDist+=(_currentpos.x()-fabs(prevStep));
            prevStep=fabs(_currentpos.x());
            meanStep=stepDist/nbStep;

                //manage the Heelstrike
            _system->getBase().setPos(point); //get the base to the new contact pos

                //velocities

            impactAngle+=_system->statePosition("q1");
            meanImpactAngle=impactAngle/nbStep;
            if(nbStep>1)
                devImpactAngle=0.0;
            else
                devImpactAngle=1.0;
            devImpactAngle+=fabs(meanImpactAngle-_system->statePosition("q1"));


            // scalar gamma=atan2(-0.21,1.0); //FIXME ground angle
            scalar gamma=atan2(_F(1.0)-_F(0.0),1.0); //FIXME ground angle

            scalar theta=_system->statePosition("q1")-gamma;
            scalar theta_dot=_system->stateVelocity("q1");
            scalar psi_dot=cos(2.0*theta)*(1.0-cos(2.0*theta))*theta_dot;

            theta_dot=cos(2.0*theta)*theta_dot;

            _system->stateVelocity("q1")=theta_dot;
            _system->stateVelocity("q2")=psi_dot;

            std::cout<<"Q2: "<<_system->statePosition("q2")<<" POS: "<<_currentpos.x()<<std::endl;
            _system->statePosition("q1")=_system->evalAngle(*_body);
            _system->statePosition("q2")=-_system->statePosition("q2");


            _system->initSymbols();


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

            if(_contact) //FIXME
            {
                viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,255));
            }
            else
                viewer.drawCircle(_currentpos.x(),_currentpos.y(),0.05,sf::Color(255,0,0,100));


                //the leg
            viewer.drawSegmentByEnd(_currentpos.x(),_currentpos.y(),_system->evalPosition(*_body).x() ,_system->evalPosition(*_body).y() ,0.05,sf::Color(0,255,0,50));

        }
};

#endif
