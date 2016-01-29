//*****************************************************************************
//
// File Name	: 'PassiveWalkerWithKnee.hpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@labri.fr
// Created	: mercredi, janvier 27 2016
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	notes
//
//*****************************************************************************

#if !defined(PASSIVEWALKERWITHKNEE_HPP)
#define PASSIVEWALKERWITHKNEE_HPP


#include <iostream>
#include <fstream>
#include <string>
#include <json/json.h>

#include "Model/PassiveWalker.hpp"

#include <cassert>

#include <cmath>
#include <Eigen/Dense>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"

#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/terms.h"
#include "SimMecha/src/Simulation.h"

#include "SimMecha/src/Constraint.hpp"
#include "SimMecha/src/UnaryConstraint.hpp"
#include "SimMecha/src/BinaryConstraint.hpp"
#include "SimMecha/src/HeightUnaryConstraint.hpp"

#include <stdio.h>
#include <functional>
// #include "Simulations/SimpleWalkerGround.hpp"

#define SKIP_FRAME 0 //20

// #define DRAW
// #define LOG

#define FREE_KNEE 0
#define LOCKED_KNEE 1
#define FALL 2

#define _COLLISION_COOLDOWN 2

using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;


class PassiveWalkerKneeStop//: public BinaryConstraint //useless
{
  public:

    Body* _thigh;
    Body *_shank;
    System* _system;

    PassiveWalkerKneeStop(Body* thigh, Body* shank, System *system): _thigh(thigh), _shank(shank), _system(system)
    {

    }

    bool computeCheckConstraint(
            Vector2D& pos1, Vector2D& norm1,
            Vector2D& pos2, Vector2D& norm2)
    {

        double angle_thigh=fmod(_system->evalAngle(*_thigh)+M_PI,2.0*M_PI); //seems to be the positive angle
        double angle_shank=fmod(_system->evalAngle(*_shank),2.0*M_PI);
        // std::cout<<"DEBUG knee: "<<angle_thigh<<" "<<angle_shank<<std::endl;
        if(angle_shank>=angle_thigh)
            return true;
        else
            return false;
    }
};


class PassiveWalkerGround: public Ground
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

    PassiveWalkerGround(Body& body, System& system,
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



        if((prev_isFootUpGround != isFootUpGround) ) //edge
        {
            isContactEnabled=true;

        }
        else
            isContactEnabled=false;

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


        if(fabs(_system->statePosition("q1")) > M_PI/2.0)
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


        //manage the Heelstrike
        _system->getBase().setPos(point); //get the base to the new contact pos


        scalar theta=_system->statePosition("q1")-gamma;
        scalar theta_dot=_system->stateVelocity("q1");
        scalar psi_dot=cos(2.0*theta)*(1.0-cos(2.0*theta))*theta_dot;

        // std::cout<<"DEBUG test: "<<2.0*theta<<" "<<_system->statePosition("q2")<<" "<<gamma<<std::endl;
        theta_dot=cos(2.0*theta)*theta_dot;

        _system->stateVelocity("q1")=theta_dot;
        _system->stateVelocity("q2")=psi_dot;

        std::cout<<"COLLISION Q1: "<<_system->statePosition("q1")<<" dQ1: "<<_system->stateVelocity("q1")<<" Q2: "<<_system->statePosition("q2")<<" dQ2: "<<_system->stateVelocity("q2")<<" POS: "<<_currentpos.x()<<" STEP: "<<nbStep<<std::endl;


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

    //SOOOooo stupid
    inline bool computeCheckConstraint(
            Vector2D& point, Vector2D& dir, Vector2D& posInBody)
    {
        return Ground::computeCheckConstraint(point,dir,posInBody);
    }
};


class PassiveWalkerWithKnee: public PassiveWalker
{
  public:

    //parameters
    double m_h;
    double m_s;
    double m_t;
    double L;
    double a1;
    double a2;
    double b1;
    double b2;
    double l_t,l_s;
    //init_state
    double init_q1, init_q1_dot;
    double init_q2, init_q2_dot;
    double init_q3, init_q3_dot;
    double init_swing;

    //world
    double slope;
    double offset;
    // double ga, gb;
    std::function<float(float)> F_ground;
    PassiveWalkerGround* ground;
    PassiveWalkerKneeStop* knee_stop;

    //model
    System* modelbase;
    Body* stance_leg;
    Body* swing_thigh;
    Body* swing_shank;

    Body* stance_thigh;
    Body* stance_shank;

    //simu
    double time;
    int skip;

    //State machine
    int state; //0=(stance leg + free knee swing leg); 1=(stance leg + locked knee swing leg)
    //2=fall

    Leph::SimViewer::SimViewer* viewer;

    PassiveWalkerWithKnee(const char* jsonconf): PassiveWalker(jsonconf)
    {
        Json::Value model=this->conf_root["model"];

        if(!model["has_knee"].asBool())
            throw("Trying to create a PassiveWalkerWithKnee: JSON conf said has_knee==false");

        if(model["has_feet"].asBool())
            throw("Trying to create a PassiveWalkerWithKnee: JSON conf said has_feet==true");

        m_h=model["parameters"]["m_h"].asDouble();
        m_s=model["parameters"]["m_s"].asDouble();
        m_t=model["parameters"]["m_t"].asDouble();
        L=model["parameters"]["L"].asDouble();
        a1=model["parameters"]["a1"].asDouble();
        a2=model["parameters"]["a2"].asDouble();
        b1=model["parameters"]["b1"].asDouble();
        b2=model["parameters"]["b2"].asDouble();
        l_t=a2+b2;
        l_s=a1+b1;
        init_q1=model["init_state"]["q1"].asDouble(); //should be useless
        init_q2=model["init_state"]["q2"].asDouble();
        init_q3=model["init_state"]["q3"].asDouble(); //should be useless
        init_q1_dot=model["init_state"]["q1_dot"].asDouble();
        init_q2_dot=model["init_state"]["q2_dot"].asDouble();
        init_q3_dot=model["init_state"]["q3_dot"].asDouble();
        init_swing=model["init_state"]["swing"].asDouble();

        Json::Value world=this->conf_root["world"];
        slope=world["ground"]["slope"].asDouble();


        // System system(Vector2D(0.0, 0.0));
        modelbase=new System(Vector2D(0.0, 0.0));;

        //Base is at (0,0) = stance foot
        // system.getBase().addMass(m_s, Vector2D(0.0, a1));
        // system.getBase().addMass(m_s, Vector2D(0.0, 0.0));

        BuildInitModel();

        viewer= new Leph::SimViewer::SimViewer(1280, 1024);

        viewer->setSpaceHandler(
            [this](Leph::Any::Any p) -> void {
                this->space_cb(p);
            } , param);
        viewer->setRHandler(
            [this](Leph::Any::Any p) -> void {
                this->R_cb(p);
            } , param);

    }

    ~PassiveWalkerWithKnee()
    {

    }

    void InitModelFreeKnee(double init_angle, double init_vel, double init_swing, double init_swingvel, double init_kneevel)
    {

        //TODO: handle the init position
        // Vector2D pos=modelbase->evalPosition(*modelbase);
        stance_leg = &(modelbase->addAngularJoint(
            modelbase->getBase(),
            Vector2D(0.0, 0.0), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            init_angle, init_vel));
        stance_leg->addMass(m_s, Vector2D(0.0, a1));
        stance_leg->addMass(m_t, Vector2D(0.0, L-b2));


        swing_thigh = &(modelbase->addAngularJoint(
            *stance_leg,
            Vector2D(0.0, L), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            init_swing, init_swingvel));
        swing_thigh->addMass(m_t, Vector2D(0.0, b2));


        swing_shank = &(modelbase->addAngularJoint(
            *swing_thigh,
            Vector2D(0.0, b2+a2), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            M_PI, init_kneevel)); //straight leg
        swing_shank->addMass(m_s, Vector2D(0.0, -b1));

        modelbase->initSymbols();

    }

    void InitModelLockedKnee(double init_angle, double init_vel, double init_swing, double init_swingvel)
    {

        //TODO initial position


        Vector2D pos=modelbase->evalPosition(*stance_leg);
        modelbase=new System(pos); //FIXME: memory?

        stance_leg = &(modelbase->addAngularJoint(
            modelbase->getBase(),
            Vector2D(0.0, 0.0), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            init_angle, init_vel));
        stance_leg->addMass(m_s, Vector2D(0.0, a1));
        stance_leg->addMass(m_t, Vector2D(0.0, L-b2));

        //Ignore the thigh

        swing_shank = &(modelbase->addAngularJoint(
            *stance_leg,
            Vector2D(0.0, L), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            init_swing, init_swingvel));
        swing_shank->addMass(m_t, Vector2D(0.0, b2));
        swing_shank->addMass(m_s, Vector2D(0.0, l_t+b1));


        modelbase->initSymbols();

    }


    void BuildInitModel()
    {

        //At init, both feet are on the ground and both legs are straight.
        //The stance (front) leg has a blocked knee

        //compute the angle in order to place both feet on the ground

        double slope_angle=slope;
                // atan2(slope,1.0); //FIXME
        double init_angle=(M_PI-init_swing)/2.0-slope_angle-M_PI/2.0;

        //build the model
        InitModelFreeKnee(-init_angle, init_q1_dot, M_PI-init_swing, init_q2_dot, init_q3_dot);

        state=FREE_KNEE;

        //build the ground
        offset=0.0;
        double& ga=slope;
        double& gb=offset;

        //function defining the curve of the ground
        F_ground = [&ga, &gb](double x) -> double
                        {
                            return ga*x+gb;
                        };

        //Collision stuffs
        //legacy
        ground=new PassiveWalkerGround(*swing_shank, *modelbase, 0.9, false, F_ground, Vector2D(0.0, -(b1+a1)));
        knee_stop=new PassiveWalkerKneeStop(swing_thigh, swing_shank, modelbase);

        skip=0;
        time=0.0;


    }

    void lock_the_knee()
    {

        double q2=fmod(modelbase->evalAngle(*swing_thigh)+M_PI,2.0*M_PI); //seems to be the positive angle
        double q3=fmod(modelbase->evalAngle(*swing_shank),2.0*M_PI);
        //TODO check
        double q1=modelbase->evalAngle(*stance_leg);

        //because of the referential...
        double old_q2=modelbase->evalAngle(*swing_thigh);
        double old_q3=modelbase->evalAngle(*swing_shank);

        double q1_dot=modelbase->stateVelocity("q1");
        double q2_dot=modelbase->stateVelocity("q2");
        double q3_dot=modelbase->stateVelocity("q3");


        double alpha=q1-q2;
        double beta=q1-q3;
        double gamma=q2-q3;

        //TODO optimize!
        double q11_m=-(m_s*l_t+m_t*b2)*L*cos(alpha)-(m_s*b1*L*cos(beta))+(m_t+m_s+m_h)*pow(L,2)+m_s*pow(a1,2)+m_t*pow((l_s+a2),2);
        double q12_m=-(m_s*l_t+m_t*b2)*L*cos(alpha)-(m_s*b1*l_t*cos(gamma))+m_t*pow(b2,2)+m_s*pow(l_t,2);
        double q13_m=-(m_s*b1*L*cos(beta))+m_s*b1*l_s*cos(gamma)+m_s*pow(b1,2);
        double q21_m=-(m_s*l_t+m_t*b2)*L*cos(alpha)-m_s*b1*L*cos(beta);
        double q22_m=m_s*b1*l_t*cos(gamma)+m_s*pow(l_t,2)+m_t*pow(b2,2);
        double q23_m=m_s*b1*l_t*cos(gamma)+m_s*pow(b1,2);


        double q21_p=-(m_s*(b1+l_t)+m_t*b2)*L*cos(alpha);
        double q11_p=q21_p+m_t*pow((l_s+a2),2)+(m_h+m_t+m_s)*pow(L,2)+m_s*pow(a1,2);
        double q12_p=q21_p+m_s*pow((l_t+a2),2)+m_t*pow(b2,2);
        double q22_p=m_s*pow((l_t+b1),2)+m_t*pow(b2,2);

        Eigen::Matrix<double, 2, 3> Q_m;
        Q_m<<q11_m,q12_m,q13_m,
                q21_m,q22_m,q23_m;

        Eigen::Matrix<double, 2, 2> Q_p;
        Q_p<<q11_p,q12_p,
                q21_p,q22_p;

        Eigen::Vector3d v_q_m(q1_dot,q2_dot,q3_dot);

        // Eigen::Vector2d v_q_p=Q_p.inverse()*v_q_m*Q_m;
        Eigen::Vector2d v_q_p=Q_m*v_q_m;
        v_q_p=v_q_p.transpose()*Q_p.inverse();

        std::cout<<"DEBUG knee res: "<<v_q_p<<std::endl;
        std::cout<<"DEBUG old: "<<q1_dot<<" "<<q2_dot<<std::endl;
        // std::cout<<"DEBUG knee vm:\n"<<v_q_m<<std::endl;
        // std::cout<<"DEBUG knee qm:\n"<<Q_m<<std::endl;
        // std::cout<<"DEBUG knee qp:\n"<<Q_p<<std::endl;

        InitModelLockedKnee(q1, v_q_p(0), -q1+old_q2, v_q_p(1));

    }

    void detect_collision()
    {
        //Two possibles correct collisions:
        // 1/ swing leg knee extension (knee lockup)
        // 2/ swing leg heel-ground collision (only if knee is locked)

        Vector2D point, dir, posInBody;
        Vector2D pos1, norm1, pos2, norm2;


        if(state==FREE_KNEE)
        {

            //check if collision happened
            bool groundcollision=ground->computeCheckConstraint(point, dir, posInBody);
            bool kneecollision=knee_stop->computeCheckConstraint(pos1, norm1, pos2, norm2);

            if(kneecollision)//ok lock the knee
            {
                //TODO
                std::cout<<"DEBUG COLLISION: knee"<<std::endl;
                lock_the_knee();
                state=LOCKED_KNEE;
            }
            if(groundcollision)//Forbidden
            {
                //TODO
                std::cout<<"DEBUG COLLISION: forbidden ground"<<std::endl;
                state=FALL;
            }
        }
        if(state==LOCKED_KNEE)
        {

            //check if collision happened

            /*
            bool groundcollision=ground->computeCheckConstraint(point, dir, posInBody);

            if(groundcollision) //ok swap stance leg
            {
                //TODO
                std::cout<<"DEBUG COLLISION: ground"<<std::endl;
                state=FREE_KNEE;
            }
            */
        }


    }




    //TODO: move that to the mother class
    void draw()
    {



        if (viewer->isOpen()) {
            if(skip==0)
            {
                viewer->beginDraw();
                viewer->drawFrame();
                modelbase->draw(*viewer);
                ground->draw(*viewer); //FIXME

                // scalar Ep=system.evalPotential();
                // scalar Ec=system.evalKinetic();
                // data.push_back(Vector2D(time,Ep+Ec));
                // system.plot(viewer,data);

                viewer->moveCam(-modelbase->evalPosition(*swing_thigh).x(),modelbase->evalPosition(*swing_thigh).y());

                viewer->endDraw(); //TODO
                skip=SKIP_FRAME;
            }
            else
                skip--;

        }
    }

    void SimuStep(double dt)
    {

        // std::cout<<"RESET: "<<simu_reset<<std::endl;
        // std::cout<<"PAUSE: "<<simu_pause<<std::endl;


        try{
            if(simu_reset){
                modelbase->stateReset();
                simu_reset=false;
            }
            if(!simu_pause){

                modelbase->runSimulationStep(dt);

                time+=dt;
                // std::cout<<"TIME: "<<t<<std::endl;
                // ground->handle();
                detect_collision();
            }
        }
        catch(const std::exception & e)
        {
            std::cerr << e.what()<<std::endl;
        }

    }


};

#endif
