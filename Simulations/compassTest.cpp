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
#include "Simulations/SimpleWalkerGround.hpp"

#define SKIP_FRAME 20

// #define DRAW
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

        //6 steps
        // double slope=-0.133003;
    // double init_angle=-0.376547;
    // double init_vel=0.274254;
    // double init_swingangle=0.783498;
    // double init_swingvel=0.0920029;

 //   pas mal
    // double slope=-0.133003;
    // double init_angle=-0.376547;
    // double init_vel=0.274255;
    // double init_swingangle=0.783498;
    // double init_swingvel=0.0920026;

    // double slope=-0.35723;
    // double init_angle=0.0296499;
    // double init_vel=0.0302701;
    // double init_swingangle=1.45022e-05;
    // double init_swingvel=1.45022e-05;


    //pas mal
    // double init_vel=-0.518444;
    // double init_swingangle=-0.613237;
    // double init_swingvel=0.0205823;
    // double slope=-0.1;

    //fixed point
    // double init_vel=-0.394568;
    // double init_swingangle=-0.353006;
    // double init_swingvel=-0.0249299;
    // double slope=-0.00515664;


    //fixed point
    // double init_vel=-0.318324391427563;
    // double init_swingangle=-0.280009715614562;
    // double init_swingvel=-0.0127821587541628;
    // double slope=-0.00204763028992204;

    //best fixed point
    double init_vel=-0.297119731718663;
    double init_swingangle=-0.261985514082957;
    double init_swingvel=-0.0101880039522822;
    double slope=-0.00232116135922738;


    // double init_vel=-0.565276211768901;
    // double init_swingangle=-0.533851692013085;
    // double init_swingvel=-0.0812830282307813;
    // double slope=-0.02;

    // double init_vel=-0.459707316675029;
    // double init_swingangle=-0.420024613478308;
    // double init_swingvel=-0.0402200727105516;
    // double slope=-0.01;



    // //fixed point
    // double init_vel=-0.556258268721377;
    // double init_swingangle=-0.525716604307055;
    // double init_swingvel=-0.0757577623630868;
    // double slope=-0.02;

        //fixed point 10e-10
    // double init_vel=-0.556258304656607;
    // double init_swingangle=-0.525716634797992;
    // double init_swingvel=-0.0757577151745346;
    // double slope=-0.02;




    // double init_vel=-0.661016;
    // double init_swingangle=-0.702991;
    // double init_swingvel=0.45054;


    // double slope=-0.1;



    //test with 0.1 and 1 mass
    // double init_vel=-0.46557571316247;
    // double init_swingangle=-0.402591267965758;
    // double init_swingvel=-0.04077794269305;
    // double slope=-0.02;




    double init_angle=(M_PI-init_swingangle)/2.0+atan2(slope,1.0)-M_PI/2.0;






    // double slope=-0.1;
    // double init_angle=atan2(slope,1)-0.2;
    // double init_vel=-0.2;
    // double init_swingangle=0.8;
    // double init_swingvel=0.0;


    // double ga=-0.21;
    double ga=slope;
    // double ga=0.0;
    double gb=0.0;//-1.3;



    auto F_ground = [&ga, &gb](double x) -> double
        {
            // gb+=0.000001;
            return ga*x+gb;
        };



    System system(Vector2D(0.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(0.1, Vector2D(0.0, 0.0));



    Body& b1 = system.addAngularJoint(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_angle, init_vel);
    b1.addMass(0.001, Vector2D(0.0, 2.0));


    Body& b2 = system.addAngularJoint(
        b1,
        Vector2D(0.0, 2.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_swingangle, init_swingvel);
    b2.addMass(0.001, Vector2D(0.0, -2.0));

    b2.addMass(1, Vector2D(0.0, 0.0));

    system.initSymbols();


    SimpleWalkerGround g(b2, system, 0.9, false, F_ground, Vector2D(0.0, -2.0));

    int skip=0;

    double time=0.0;

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
