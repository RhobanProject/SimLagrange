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

#include <stdio.h>
#include <functional>
// #include "Simulations/SimpleWalkerGround.hpp"

//As usual...
// Converts degrees to radians.
#define DEG_TO_RAD(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define RAD_TO_DEG(angleRadians) (angleRadians * 180.0 / M_PI)





#define SKIP_FRAME 20 //20

#define DRAW true
#define LOGGING false

#define FREE_KNEE 1
#define LOCKED_KNEE 2
#define FALL 4

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

    void swapmodel(Body* thigh, Body* shank, System *system)
    {
        _thigh=thigh;
        _shank=shank;
        _system=system;
    }

    bool computeCheckConstraint()
    {

        // double angle_thigh=fmod(_system->evalAngle(*_thigh)+M_PI,2.0*M_PI); //seems to be the positive angle
        // double angle_shank=fmod(_system->evalAngle(*_shank),2.0*M_PI);

        // std::cout<<"DEBUG knee: "<<angle_thigh<<" "<<angle_shank<<std::endl;

        double angle_thigh=_system->evalAngle(*_thigh);
        double angle_shank=_system->evalAngle(*_shank);

        // std::cout<<"DEBUG knee: "<<angle_thigh<<" "<<angle_shank<<std::endl;

        if((angle_shank-angle_thigh)>=0)
            return true;
        else
            return false;
    }
};


class PassiveWalkerGroundContact
{
  public:

    // Body* _thigh;
    Body *_shank;
    System* _system;
    Vector2D _posInBody;
    Vector2D lastContactPoint;
    std::function<float(float)> _F;
    bool _contact;

    PassiveWalkerGroundContact(Body* shank, System *system, Vector2D posInBody, std::function<float(float)> F): _shank(shank), _system(system), _posInBody(posInBody), _F(F)
    {
        lastContactPoint=Vector2D(0,0);
        _contact=false;
    }
    void swapmodel(Body* shank, System *system, Vector2D posInBody)
    {
        _shank=shank;
        _system=system;
        _posInBody=posInBody;
    }

    bool computeCheckConstraint()
    {
        Vector2D centerPos = _system->evalPosition(*_shank);
        scalar centerAngle = _system->evalAngle(*_shank);

        Vector2D pos = centerPos
                       + Vector2D::rotate(_posInBody, centerAngle);

        // std::cout<<"DEBUG ground: "<<pos.x()<<" "<<pos.y()<<" "<<centerPos.x()<<" "<<centerPos.y()<<" "<<centerAngle<<std::endl;

            // Vector2D kneepos=modelbase->evalPosition(*swing_shank);
            // scalar centerAngle = modelbase->evalAngle(*swing_shank);
            // Vector2D footpos = kneepos + Vector2D::rotate(Vector2D(0.0, -(b1+a1)), centerAngle);


        lastContactPoint = pos;
        // dir = Vector2D(0.0, 1.0); //TODO normal
        // posInBody = _posInBody;
        // _currentpos=pos;


        if (pos.y() <= _F(pos.x())) {
            // if ((pos.y()-_F(pos.x())) <=0.0 && fabs(pos.y()-_F(pos.x()))<0.01 ) {
            // if ((pos.y()-_F(pos.x())) <=0.0) {
            _contact=true;
            return true;
        } else {
            _contact=false;
            return false;
        }
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

    bool _forced_init;
    //world
    double slope;
    double offset;
    // double ga, gb;
    std::function<float(float)> F_ground;

    PassiveWalkerKneeStop* knee_stop;
    PassiveWalkerGroundContact* ground_contact;
    double collisiontimeoffset;
    int isCollided;

    //model
    System* modelbase;
    Body* stance_leg;
    Body* swing_thigh;
    Body* swing_shank;

    Body* stance_thigh;
    Body* stance_shank;

    double current_q1;
    double current_q2;
    double current_q3;
    double current_swing;

    double current_q1_dot;
    double current_q2_dot;
    double current_q3_dot;



    //simu
    double time;
    int _skip;
    int nbStep;
    bool _draw;

    //State machine
    unsigned char state; //0=(stance leg + free knee swing leg); 1=(stance leg + locked knee swing leg)
    //2=fall

    //log file
    ofstream logfile;
    bool _log;

    Leph::SimViewer::SimViewer* viewer;


    PassiveWalkerWithKnee(Json::Value conf, bool draw=DRAW, int skip=SKIP_FRAME, bool log=LOGGING): PassiveWalker(conf)
    {


        getConfig();


        // System system(Vector2D(0.0, 0.0));
        modelbase=new System(Vector2D(0.0, 0.0));
        //dummy, just to init the memory
        stance_leg = &(modelbase->addAngularJoint(
            modelbase->getBase(),
            Vector2D(0.0, 0.0), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            0, 0));
        modelbase->initSymbols();

        BuildInitModel();

        InitStuffs(draw, skip, log);

    }



    PassiveWalkerWithKnee(const char* jsonconf, bool draw=DRAW, int skip=SKIP_FRAME, bool log=LOGGING): PassiveWalker(jsonconf)
    {

        getConfig();

        // System system(Vector2D(0.0, 0.0));
        modelbase=new System(Vector2D(0.0, 0.0));
        //dummy, just to init the memory
        stance_leg = &(modelbase->addAngularJoint(
            modelbase->getBase(),
            Vector2D(0.0, 0.0), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            0, 0));
        modelbase->initSymbols();

        BuildInitModel();

        InitStuffs(draw, skip, log);

    }


    void InitStuffs(bool draw, int skip, bool log)
    {

                //Viewer stuffs

        _skip=skip;
        _draw=draw;
        _log=log;
        if(_draw)
        {
            viewer= new Leph::SimViewer::SimViewer(1280, 1024);

            //Viewer callbacks
            viewer->setSpaceHandler(
                [this](Leph::Any::Any p) -> void {
                    this->space_cb(p);
                } , param);
            viewer->setRHandler(
                [this](Leph::Any::Any p) -> void {
                    this->R_cb(p);
                } , param);
        }

        if(_log)
            logfile.open("log.dat");

        collisiontimeoffset=0.0;
        nbStep=0;
        isCollided=0;


        current_q1=0.0;
        current_q2=0.0;
        current_q3=0.0;
        current_swing=0.0;

        current_q1_dot=0.0;
        current_q2_dot=0.0;
        current_q3_dot=0.0;

    }


    void getConfig()
    {

        // std::cout<<"DEBUG my json: "<<std::endl;
        // std::cout<<this->conf_root<<std::endl;

        Json::Value model=this->conf_root["model"];


        if(!model["has_knee"].asBool())
            throw("Trying to create a PassiveWalkerWithKnee: JSON conf said has_knee==false");

        if(model["has_feet"].asBool())
            throw("Trying to create a PassiveWalkerWithKnee: JSON conf said has_feet==true");


        _forced_init=model["forced_init"].asBool();



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
        // std::cout<<"DEBUG Length: "<<l_t<<" "<<l_s<<" "<<a1<<" "<<a2<<std::endl;
        init_q1=model["init_state"]["q1"].asDouble(); //should be useless
        init_q2=model["init_state"]["q2"].asDouble();
        init_q3=model["init_state"]["q3"].asDouble(); //should be useless
        init_q1_dot=model["init_state"]["q1_dot"].asDouble();
        init_q2_dot=model["init_state"]["q2_dot"].asDouble();
        init_q3_dot=model["init_state"]["q3_dot"].asDouble();
        init_swing=model["init_state"]["swing"].asDouble();

        Json::Value world=this->conf_root["world"];
        slope=world["ground"]["slope"].asDouble();



    }


    PassiveWalkerWithKnee(double ground_slope, double init_vel, double init_swing_angle, double init_swing_vel, bool draw=DRAW, int skip=SKIP_FRAME, bool log=LOGGING): PassiveWalker()
    {
        //Without json


        m_h=1.0;
        m_s=0.001;
        m_t=0.001;
        // L=1.0;
        // a1=0.25;
        // a2=0.25;
        // b1=0.25;
        // b2=0.25;

        L=2.0;
        a1=0.5;
        a2=0.5;
        b1=0.5;
        b2=0.5;


        l_t=a2+b2;
        l_s=a1+b1;
        // init_q1=model["init_state"]["q1"].asDouble(); //should be useless
        // init_q2=model["init_state"]["q2"].asDouble();
        // init_q3=model["init_state"]["q3"].asDouble(); //should be useless
        init_q1_dot=init_vel;
        init_q2_dot=init_swing_vel;
        init_q3_dot=0.0;
        init_swing=init_swing_angle;


        slope=ground_slope;


        // System system(Vector2D(0.0, 0.0));
        modelbase=new System(Vector2D(0.0, 0.0));
        //dummy, just to init the memory
        stance_leg = &(modelbase->addAngularJoint(
            modelbase->getBase(),
            Vector2D(0.0, 0.0), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            0, 0));
        modelbase->initSymbols();

        BuildInitModel();


        InitStuffs(draw, skip, log);

    }




    ~PassiveWalkerWithKnee()
    {

        if(_log)
            logfile.close();

        // if(modelbase)
        delete modelbase;
        delete knee_stop;
        delete ground_contact;
        if(_draw)
            delete viewer;
    }


    void InitModelFreeKnee(Vector2D pos, double init_angle, double init_vel, double init_swing, double init_swingvel, double init_kneevel)
    {


        // Vector2D pos=modelbase->evalPosition(*stance_leg);
        // if(modelbase)
        delete modelbase;
        // if(stance_leg)
        //     delete stance_leg;
        // if(swing_thigh)
        //     delete swing_thigh;
        // if(swing_shank)
        //     delete swing_shank;

        modelbase=new System(pos);

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
        swing_thigh->addMass(m_t, Vector2D(0.0, -b2));


        swing_shank = &(modelbase->addAngularJoint(
            *swing_thigh,
            Vector2D(0.0, -(b2+a2)), 0.0,
            Vector2D(0.0, 0.0), 0.0,
            0, init_kneevel)); //straight leg
        swing_shank->addMass(m_s, Vector2D(0.0, -b1));

        modelbase->initSymbols();

        // knee_stop->swapmodel(swing_thigh,swing_shank,modelbase);
        // ground_contact->swapmodel(swing_shank,modelbase);

    }

    void InitModelLockedKnee(Vector2D pos, double init_angle, double init_vel, double init_swing, double init_swingvel)
    {


        // Vector2D pos=modelbase->evalPosition(*stance_leg);
        // delete modelbase;
        // delete stance_leg;
        // delete swing_thigh;
        // delete swing_shank;

        // if(modelbase)
        delete modelbase;
        // if(stance_leg)
        //     delete stance_leg;
        // if(swing_thigh)
        //     delete swing_thigh;
        // if(swing_shank)
        //     delete swing_shank;

        modelbase=new System(pos);

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
        swing_shank->addMass(m_t, Vector2D(0.0, -b2));
        swing_shank->addMass(m_s, Vector2D(0.0, -(l_t+b1)));


        modelbase->initSymbols();

        // knee_stop->swapmodel(swing_thigh,swing_shank,modelbase);
        // ground_contact->swapmodel(swing_shank,modelbase);

    }


    void BuildInitModel()
    {

        //At init, both feet are on the ground and both legs are straight.
        //The stance (front) leg has a blocked knee

        //compute the angle in order to place both feet on the ground

        double slope_angle=slope;
                // atan2(slope,1.0); //FIXME
        double init_angle=(M_PI+init_swing)/2.0-slope_angle-M_PI/2.0;

        //build the model
        // InitModelFreeKnee(Vector2D(0,0),-init_angle, init_q1_dot, M_PI-init_swing, init_q2_dot, init_q3_dot);
        if(!_forced_init)
            InitModelFreeKnee(Vector2D(0,0),-init_angle, init_q1_dot, init_swing, init_q2_dot, init_q3_dot);
        else
            InitModelFreeKnee(Vector2D(0,0),init_q1, init_q1_dot, init_q2-init_q1, init_q2_dot, init_q3_dot);

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

        knee_stop=new PassiveWalkerKneeStop(swing_thigh, swing_shank, modelbase);
        // ground_contact=new PassiveWalkerGroundContact(swing_shank, modelbase, Vector2D(0.0, L), F_ground);
        ground_contact=new PassiveWalkerGroundContact(swing_shank, modelbase, Vector2D(0,-(a1+b1)), F_ground); //carefull, we start in free_knee

        _skip=0;
        time=0.0;


    }


    //comes from UnaryConstraint.hpp
    bool getConstraintTime(std::function<bool()> computeCheckConstraint)
    {
        if (!computeCheckConstraint()) {
            return false;
        }

        //TODO Bijection
        scalar timeMin = 0.0;
        scalar timeMax = 0.01;
        scalar currentTime = 0.0;
        for (int i=0;i<100;i++) { //FIXME?
            scalar t = (timeMax-timeMin)/2.0;
            modelbase->runSimulationStep(currentTime-t);
            currentTime = t;
            if (computeCheckConstraint()) {
                timeMin = t;
            } else {
                timeMax = t;
            }
            // std::cout<<"DEBUG Col: "<<t<<" "<<timeMin<<" "<<timeMax<<std::endl;
        }
        // std::cout << "TTT> " << currentTime << " " << timeMax << std::endl;
        modelbase->runSimulationStep(currentTime-timeMax);
        computeCheckConstraint();
        std::cout<<"DEBUG collision time: "<<currentTime-timeMax<<std::endl;
        collisiontimeoffset=(currentTime-timeMax);
        return true;
    }



    void lock_the_knee()
    {

        // minus everywhere...

        // double q2=fmod(modelbase->evalAngle(*swing_thigh)+M_PI,2.0*M_PI); //seems to be the positive angle
        double q2=-fmod(modelbase->evalAngle(*swing_thigh),2.0*M_PI); //seems to be the positive angle

        double q3=-fmod(modelbase->evalAngle(*swing_shank),2.0*M_PI);
        //TODO check
        double q1=-modelbase->evalAngle(*stance_leg);

        //because of the referential...
        double old_q2=-modelbase->evalAngle(*swing_thigh);
        double old_q3=-modelbase->evalAngle(*swing_shank);

        double q1_dot=-modelbase->stateVelocity("q1");
        double q2_dot=-modelbase->stateVelocity("q2");
        double q3_dot=-modelbase->stateVelocity("q3");


        std::cout<<"DEBUG KNEE COL: "<<RAD_TO_DEG(q1)<<" "<<RAD_TO_DEG(q2)<<" "<<RAD_TO_DEG(q3)<<std::endl;
        std::cout<<q1_dot<<" "<<q2_dot<<" "<<q3_dot<<std::endl;



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

        // std::cout<<"DEBUG MATRIX0: "<<v_q_p.transpose()*Q_p.inverse()<<std::endl;
        // v_q_p=v_q_p.transpose()*Q_p.inverse();
        v_q_p=Q_p.inverse()*v_q_p;

        std::cout<<"DEBUG knee res: "<<v_q_p<<std::endl;
        std::cout<<"DEBUG old: "<<q1_dot<<" "<<q2_dot<<std::endl;
        // std::cout<<"DEBUG knee vm:\n"<<v_q_m<<std::endl;
        // std::cout<<"DEBUG knee qm:\n"<<Q_m<<std::endl;
        // std::cout<<"DEBUG knee qp:\n"<<Q_p<<std::endl;
        Vector2D pos=modelbase->evalPosition(*stance_leg);
        // InitModelLockedKnee(pos,q1, v_q_p(0), (-q1+old_q2), v_q_p(1)); //should be ok
        // InitModelLockedKnee(pos,q1, v_q_p(0), (-q1+old_q2), v_q_p(1));
        InitModelLockedKnee(pos,-q1, -v_q_p(0), -(-q1+old_q2), -v_q_p(1));


    }




    void foot_on_ground()
    {

        //minus everywhere
        double q2=-fmod(modelbase->evalAngle(*swing_shank),2.0*M_PI); //seems to be the positive angle

        double q1=-modelbase->evalAngle(*stance_leg);

        //because of the referential...
        //argh
        // double old_q2=modelbase->evalAngle(*swing_thigh);
        double old_q2=-modelbase->evalAngle(*swing_shank);
        // double old_q3=modelbase->evalAngle(*swing_shank);

        double q1_dot=-modelbase->stateVelocity("q1");
        double q2_dot=-modelbase->stateVelocity("q2");
        // double q3_dot=modelbase->stateVelocity("q3");


        double alpha=cos(q1-q2); //error in the paper?
        // double alpha=(q1-q2);

        //TODO optimize!
        double q12_m=-m_s*a1*(l_t+b1)+m_t*b2*(l_s+a2);
        double q11_m=q12_m+(m_h*L+2.0*m_t*(a2+l_s)+m_s*a1)*L*cos(alpha);
        double q21_m=q12_m;
        double q22_m=0.0;

        double q21_p=-(m_s*(b1+l_t)+m_t*b2)*L*cos(alpha);
        double q11_p=q21_p+(m_s+m_t+m_h)*pow(L,2)+m_s*pow(a1,2)+m_t*pow((a2+l_s),2);
        double q12_p=q21_p+m_s*pow((b1+l_t),2)+m_t*pow(b2,2);
        double q22_p=m_s*pow((l_t+b1),2)+m_t*pow(b2,2);


        Eigen::Matrix<double, 2, 2> Q_m;
        Q_m<<q11_m,q12_m,
                q21_m,q22_m;

        Eigen::Matrix<double, 2, 2> Q_p;
        Q_p<<q11_p,q12_p,
                q21_p,q22_p;

        Eigen::Vector2d v_q_m(q1_dot,q2_dot);

        // Eigen::Vector2d v_q_p=Q_p.inverse()*v_q_m*Q_m;
        Eigen::Vector2d v_q_p=Q_m*v_q_m;
        // v_q_p=v_q_p.transpose()*Q_p.inverse();

        v_q_p=Q_p.inverse()*v_q_p;


        std::cout<<"DEBUG foot res: "<<v_q_p<<std::endl;
        std::cout<<"DEBUG old: "<<q1_dot<<" "<<q2_dot<<std::endl;

        Vector2D oldcontact=modelbase->evalPosition(*stance_leg);
        std::cout<<"DEBUG step: ("<<oldcontact.x()<<", "<<oldcontact.y()<<") -> ("<<ground_contact->lastContactPoint.x()<<", "<<ground_contact->lastContactPoint.y()<<")"<<std::endl;

        // InitModelFreeKnee(ground_contact->lastContactPoint, q2, v_q_p(0), -(-q1+old_q2), v_q_p(1), v_q_p(1));
        // InitModelFreeKnee(ground_contact->lastContactPoint, q2, v_q_p(0), -(-q1+old_q2), v_q_p(1), 0); //knee_vel=swing_vel. Should be ok

        InitModelFreeKnee(ground_contact->lastContactPoint, -q2, -v_q_p(0), (-q1+old_q2), v_q_p(1), 0); //knee_vel=swing_vel. Minus q2_dot?
    }



    void detect_collision()
    {
        //Two possibles correct collisions:
        // 1/ swing leg knee extension (knee lockup)
        // 2/ swing leg heel-ground collision (only if knee is locked)

        //If collision is detected, find the exact time

        Vector2D hip;
        if(state&LOCKED_KNEE)
            hip=modelbase->evalPosition(*swing_shank);
        else
            hip=modelbase->evalPosition(*swing_thigh);

        if(hip.y()<=F_ground(hip.x()))
        {
            state|=FALL;
            std::cout<<"DEBUG hip under ground"<<std::endl;
        }
        collisiontimeoffset=0.0;

        if(state&FREE_KNEE && !(state&FALL))
        {


            bool groundcollision=ground_contact->computeCheckConstraint();
            bool kneecollision=knee_stop->computeCheckConstraint();

            // std::cout<<"DEBUG FREE: "<<kneecollision<<" "<<groundcollision<<std::endl;


            if(groundcollision)//Forbidden
            {

                std::cout<<"DEBUG COLLISION: forbidden ground"<<std::endl;
                state|=FALL;
            }


            if(kneecollision)//ok lock the knee
            {
                std::cout<<"DEBUG COLLISION: knee"<<std::endl;
                //find the collision time
                getConstraintTime( [this]() -> bool {
                        knee_stop->computeCheckConstraint();
                    });

                lock_the_knee();

                knee_stop->swapmodel(swing_thigh,swing_shank,modelbase);
                ground_contact->swapmodel(swing_shank,modelbase, Vector2D(0,-L));

                state=0;
                state|=LOCKED_KNEE;
                // state&=!(FREE_KNEE);
                isCollided=_COLLISION_COOLDOWN;
            }


        }
        if(state&LOCKED_KNEE && !(state&FALL))
        {
            bool groundcollision=ground_contact->computeCheckConstraint();
            //check if collision happened

            // double q2=modelbase->statePosition("q2")-M_PI;

            // double shank_angle=fmod(modelbase->evalAngle(*swing_shank)-M_PI,2.0*M_PI);
            double shank_angle=fmod(modelbase->evalAngle(*swing_shank),2.0*M_PI);

            double stance_angle=fmod(modelbase->evalAngle(*stance_leg),2.0*M_PI);


            double q2=shank_angle-stance_angle;

            // std::cout<<"DEBUG Q2: "<<q2<<" stance: "<<stance_angle<<" swing: "<<shank_angle<<std::endl;
            //TODO Check if foot if behind.
            if(groundcollision && q2>0.0) //ok swap stance leg
            {
                std::cout<<"DEBUG Q2: "<<q2<<" stance: "<<stance_angle<<" swing: "<<shank_angle<<std::endl;

                std::cout<<"DEBUG COLLISION: ground"<<std::endl;

                //find the collision time
                getConstraintTime( [this]() -> bool {
                        ground_contact->computeCheckConstraint();
                    });

                //Change the model
                foot_on_ground();

                knee_stop->swapmodel(swing_thigh,swing_shank,modelbase);
                ground_contact->swapmodel(swing_shank,modelbase, Vector2D(0,-(a1+b1)));

                state=0;
                state|=FREE_KNEE;
                // state&=!(LOCKED_KNEE);
                nbStep++;
                isCollided=_COLLISION_COOLDOWN;
            }
            if(groundcollision && q2<=0.0)
            {
                std::cout<<"DEBUG COLLISION: forbidden ground (back leg)"<<std::endl;
                state|=FALL;
            }

        }


    }

    void draw_swing_leg()
    {


        if(state&FREE_KNEE)
        {
            Vector2D hippos=modelbase->evalPosition(*swing_thigh);

            Vector2D kneepos=modelbase->evalPosition(*swing_shank);
            scalar centerAngle = modelbase->evalAngle(*swing_shank);
            Vector2D footpos = kneepos + Vector2D::rotate(Vector2D(0.0, -(b1+a1)), centerAngle);

            if(state&FALL)
                viewer->drawCircle(footpos.x(),footpos.y(),0.05,sf::Color(255,0,0,255));
            else
                viewer->drawCircle(footpos.x(),footpos.y(),0.05,sf::Color(0,200,200,255));

            //the leg
            viewer->drawSegmentByEnd(footpos.x(),footpos.y(),kneepos.x() , kneepos.y() ,0.05,sf::Color(0,255,0,50));
            viewer->drawSegmentByEnd(kneepos.x(),kneepos.y(),hippos.x() , hippos.y() ,0.05,sf::Color(0,255,0,50));
        }
        if(state&LOCKED_KNEE)
        {

            Vector2D hippos=modelbase->evalPosition(*swing_shank);

            // scalar centerAngle = modelbase->evalAngle(*swing_thigh);
            scalar centerAngle = modelbase->evalAngle(*swing_shank);

            Vector2D footpos = hippos + Vector2D::rotate(Vector2D(0.0, -L), centerAngle);

            if(state&FALL)
                viewer->drawCircle(footpos.x(),footpos.y(),0.05,sf::Color(255,0,0,255));
            else
                viewer->drawCircle(footpos.x(),footpos.y(),0.05,sf::Color(0,200,200,255));

            //the leg
            viewer->drawSegmentByEnd(footpos.x(),footpos.y(),hippos.x() , hippos.y() ,0.05,sf::Color(0,255,0,50));
        }
    }

    void draw_ground()
    {
        Vector2D currentpos=modelbase->evalPosition(*stance_leg);
        Vector2D pos_r=currentpos;
        Vector2D pos_l=currentpos;

        for(double xi=0.0; xi<5.0;xi+=0.01)
        {
            viewer->drawSegmentByEnd(pos_r.x(),F_ground(pos_r.x()), xi+pos_r.x(),F_ground(xi+pos_r.x()) ,0.02,sf::Color(255,255,255,255));
            pos_r=Vector2D(pos_r.x()+xi, F_ground(xi+pos_r.x()));
            viewer->drawSegmentByEnd(pos_l.x(),F_ground(pos_l.x()), pos_l.x()-xi,F_ground(pos_l.x()-xi) ,0.02,sf::Color(255,255,255,255));
            pos_l=Vector2D(pos_l.x()-xi, F_ground(pos_l.x()-xi));
        }

    }


    //TODO: move that to the mother class
    void draw()
    {


        if (viewer->isOpen()) {

            if(_skip==0)
            {
                viewer->beginDraw();
                viewer->drawFrame();
                modelbase->draw(*viewer);
                draw_ground();
                draw_swing_leg();


                // scalar Ep=system.evalPotential();
                // scalar Ec=system.evalKinetic();
                // data.push_back(Vector2D(time,Ep+Ec));
                // system.plot(viewer,data);

                // viewer->moveCam(-modelbase->evalPosition(*swing_shank).x(),modelbase->evalPosition(*swing_shank).y());


                Vector2D foot=modelbase->evalPosition(*stance_leg);
                scalar centerAngle = modelbase->evalAngle(*stance_leg);

                Vector2D hippos = foot + Vector2D::rotate(Vector2D(0.0, L), centerAngle);

                //Follow the model
                viewer->moveCam(-hippos.x(),hippos.y());


                viewer->endDraw(); //TODO
                _skip=SKIP_FRAME;
            }
            else
                _skip--;
        }

    }

    void SimuStep(double dt)
    {

        try{
            if(simu_reset){


                BuildInitModel();
                simu_reset=false;
            }
            if(!simu_pause){

                if(!(state&FALL))
                {
                    modelbase->runSimulationStep(dt);
                    // try
                    // {
                    //     logfile<<time<<" "<<modelbase->statePosition("q1")<<" "<<modelbase->stateVelocity("q1")<<" "<<modelbase->statePosition("q2")<<" "<<modelbase->stateVelocity("q2")<<" "<<modelbase->statePosition("q3")<<" "<<modelbase->stateVelocity("q3")<<std::endl;
                    // }
                    // catch(const std::exception & e){}

                    /*
                    current_q1=modelbase->statePosition("q1");
                    current_q2=modelbase->statePosition("q2");
                    // current_q3=modelbase->statePosition("q3");
                    current_swing=current_q2;

                    current_q1_dot=modelbase->stateVelocity("q1");
                    current_q2_dot=modelbase->stateVelocity("q2");
                    // current_q3_dot=modelbase->stateVelocity("q3");
                    */



                    time+=dt;

                    if(isCollided==0) //cooldown. Wait a little just after a collision
                        detect_collision();
                    else
                        isCollided--;

                    time+=collisiontimeoffset;


                    current_q1=modelbase->statePosition("q1");
                    current_q2=modelbase->statePosition("q2");
                    // current_q3=modelbase->statePosition("q3");
                    current_swing=current_q2;

                    current_q1_dot=modelbase->stateVelocity("q1");
                    current_q2_dot=modelbase->stateVelocity("q2");
                    // current_q3_dot=modelbase->stateVelocity("q3");
                    if(_log)
                    {
                        scalar Ep=modelbase->evalPotential();
                        scalar Ec=modelbase->evalKinetic();
                        logfile<<time<<" "<<current_q1<<" "<<current_q1_dot<<" "<<current_q2<<" "<<current_q2_dot<<" "<<Ep<<" "<<Ec<<std::endl;
                    }

                }
            }
        }

        catch(const std::exception & e)
        {
            std::cerr << e.what()<<std::endl;
            state=FALL; //heuu
        }

        if(_draw)
            draw();

    }


};

#endif
