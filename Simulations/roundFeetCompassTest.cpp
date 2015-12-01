#include <iostream>
#include <fstream>
#include <cassert>

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

#include <stdio.h>
#include <functional>
#include "Simulations/RoundFeetWalkerGround.hpp"

#define SKIP_FRAME 20

#define DRAW
// #define LOG


using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;


bool simu_pause=false;
bool simu_reset=false;

void* space_cb(Leph::Any::Any param)
{
    if(simu_pause){
        std::cout<<"PAUSE: off"<<std::endl;
        simu_pause=false;
    }
    else
    {
        std::cout<<"PAUSE: on"<<std::endl;
        simu_pause=true;
    }
}

void* R_cb(Leph::Any::Any param)
{
    simu_reset=true;
    std::cout<<"RESET"<<std::endl;
}




int main()
{
    // Leph::SimViewer::SimViewer viewer(1280, 1024);

    Leph::Any::Any param;


    // double init_vel=-0.25*0.2;
    // double init_swingangle=-0.3;
    // double init_swingvel=-0.3;
    // double slope=-0.05;

    // double init_vel=0.848809699067517 *0.2;
    // double init_swingangle=-0.0125287420878489;
    // double init_swingvel=0.108215801468413;
    // double slope= -0.391331903396806;

    // double init_vel=-4.46388647130933e-06 *0.2;
    // double init_swingangle=2.01031524333551e-06;
    // double init_swingvel=-7.77340078896863e-09;
    // double slope= -0.000172640382141967;


    // double init_vel=-0.180775952444816 *0.2;
    // double init_swingangle=-0.178513250407527;
    // double init_swingvel=-1.95205970158943e-05;
    // double slope= -5.38636255033428e-09;



    // double init_vel=-0.127804318497643 *0.2;
    // double init_swingangle=-0.126200881282818;
    // double init_swingvel=-5.18309767791898e-06;
    // double slope=-3.0970297455562e-10;



    // double foot_radius=1.11956469704295;//0.2;
    // // double slope=-0.1;
    // double init_vel=-0.237516716283862*foot_radius;
    // double init_swingangle=-0.299266047913129;
    // double init_swingvel=-0.00427183120241122;
    // double slope=-0.00887709411969127;


    //pas mal
        double foot_radius=0.499999999998606;//0.2;
    // double slope=-0.1;
    double init_vel=-0.240615871582297*foot_radius;
    double init_swingangle=-0.247198466740646;
    double init_swingvel=-0.00418033423404718;
    double slope=-3.24581907366929e-12;


    //pas mal
    //     double foot_radius=0.967462093241187;//0.2;
    // // double slope=-0.1;
    // double init_vel=-0.336046859935751*foot_radius;
    // double init_swingangle=-0.384679658116544;
    // double init_swingvel=-0.0100908435576543;
    // double slope=-1.97186334255077e-07;


    // double foot_radius=0.0811667762698315;//0.2;
    // // double slope=-0.1;
    // double init_vel=-0.180016332095015*foot_radius;
    // double init_swingangle=-0.168777467213;
    // double init_swingvel=-6.431923527597e-07;
    // double slope=-3.07501111216845e-08;





    double init_angle=((M_PI-init_swingangle)/2.0+atan2(slope,1.0)-M_PI/2.0)*foot_radius;



    // double slope=-0.1;
    // double init_angle=atan2(slope,1)-0.2;
    // double init_vel=-0.2;
    // double init_swingangle=0.8;
    // double init_swingvel=0.0;


    // double ga=-0.21;
    double ga=slope;
    // double ga=0.0;
    double gb=-foot_radius;//-1.3;



    auto F_ground = [&ga, &gb](double x) -> double
        {
            // gb+=0.000001;
            return ga*x+gb;
        };





    System system(Vector2D(0.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(0.1, Vector2D(0.0, 0.0));



    Body& b1 = system.addRoundFoot(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        foot_radius, atan2(slope,1.0) , init_angle, init_vel);
    b1.addMass(0.001, Vector2D(0.0, 2.0));

    Body& b2 = system.addAngularJoint(
        b1,
        Vector2D(0.0, 2.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_swingangle, init_swingvel);
    b2.addMass(0.001, Vector2D(0.0, -2.0));

    b2.addMass(1, Vector2D(0.0, 0.0));

    system.initSymbols();

    RoundFeetGround g(b2, system, 0.9, false, F_ground, Vector2D(0.0, -2.0));

    g.feetRadius=foot_radius;




    int skip=0;

    double time=0.0;

    std::vector<Vector2D> data;
#ifdef LOG
    ofstream logfile;
    logfile.open("log.dat");
#endif

#ifdef DRAW
    Leph::SimViewer::SimViewer viewer(1280, 1024);
    viewer.setSpaceHandler((SimViewer::HandlerFunction)space_cb, param);
    viewer.setRHandler((SimViewer::HandlerFunction)R_cb, param);
    while (viewer.isOpen()) {
        if(skip==0)
        {
            viewer.beginDraw();
            viewer.drawFrame();
            system.draw(viewer);
            g.draw(viewer);

            // scalar Ep=system.evalPotential();
            // scalar Ec=system.evalKinetic();
            // data.push_back(Vector2D(time,Ep+Ec));
            // system.plot(viewer,data);

            viewer.moveCam(-system.evalPosition(b2).x(),system.evalPosition(b2).y());



            viewer.endDraw(); //TODO
            skip=SKIP_FRAME;
        }
        else
            skip--;

#else
    while (true) {
#endif


        try{
            if(simu_reset){
                system.stateReset();
                simu_reset=false;
                g.hasFallen=false;
            }
            if(!simu_pause && !g.hasFallen)
            {
#ifdef LOG
                logfile<<time<<" "<<system.statePosition("q1")<<" "<<system.stateVelocity("q1")<<" "<<system.statePosition("q2")<<" "<<system.stateVelocity("q2")<<"\n";
#endif
                system.runSimulationStep(0.001);

                scalar Ep=system.evalPotential();
                scalar Ec=system.evalKinetic();
                std::cout.precision(15);
                // std::cout<<"ENERGY: "<<Ep+Ec<<" Ep: "<<Ep<<" Ec: "<<Ec<<std::endl;
                std::cout<<Ep+Ec<<" "<<Ep<<" "<<Ec<<std::endl;
            }
        }
        catch(const std::exception & e)
        {
            std::cerr << e.what()<<std::endl;
        }

        g.handle();
        if(g.hasFallen)
            std::cout<<"FALL, nbStep: "<<g.nbStep<<std::endl;
        time+=0.001;
    }


    return 0;
}
