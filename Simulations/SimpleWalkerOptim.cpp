#include <iostream>
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

#include <cmaes/cmaes.h>
#include "Simulations/SimpleWalkerGround.hpp"

// #define DRAW


using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;
using namespace libcmaes;

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




FitFunc walk=[](const double *x, const int N)
{

    #ifdef DRAW
    Leph::SimViewer::SimViewer viewer(1280, 1024);
    Leph::Any::Any param;
    viewer.setSpaceHandler((SimViewer::HandlerFunction)space_cb, param);
    viewer.setRHandler((SimViewer::HandlerFunction)R_cb, param);
    #endif

    double score=0.0;

    double slope=x[0];
    double init_angle=x[1];
    double init_vel=x[2];
    double init_swingangle=x[3];
    double init_swingvel=x[4];
    // double init_swingvel=0.0;

        //check range
    if(slope>0.0)
        score+=1000.0;
    if(slope<-0.5)
        score+=1000.0;

    if(fabs(init_angle)>0.5)
        score+=1000.0;

    if(fabs(init_vel)>0.5)
        score+=1000.0;

    if(fabs(init_swingangle)>1.0)
        score+=1000.0;

    if(fabs(init_swingvel)>0.5)
        score+=1000.0;



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

    double t=0.0;
    // while (viewer.isOpen() && t<10.0 && !g.hasFallen) {
    while (t<10.0 && !g.hasFallen) { //5.5s is a bit long

        #ifdef DRAW
        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);
        g.draw(viewer);
        viewer.moveCam(-system.evalPosition(b2).x(),system.evalPosition(b2).y());
        viewer.endDraw(); //TODO
        #endif

        try{
            if(simu_reset){
                system.stateReset();
                simu_reset=false;
            }
            if(!simu_pause)
                system.runSimulationStep(0.01);
        }
        catch(const std::exception & e)
        {
            std::cerr << e.what()<<std::endl;
        }
        t+=0.01;
        // std::cout<<"TIME: "<<t<<std::endl;
        g.handle();
    }

    std::cout<<"PARAMS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<std::endl;
    // std::cout<<"SCORE: "<<score+1.0/(1.0+g._currentpos.x()+g.nbStep)+g.devImpactAngle<<" DISTANCE: "<<g._currentpos.x()<<" STEPS: "<<g.nbStep<<" DEV_IMPACT_ANGLE: "<<g.devImpactAngle<<" MEAN_STEP: "<<g.meanStep<<" PENALTY: "<<score<<std::endl;
    std::cout<<"SCORE: "<<score+1.0/(1.0+g.nbStep)+0.1*g.devImpactAngle<<" DISTANCE: "<<g._currentpos.x()<<" STEPS: "<<g.nbStep<<" DEV_IMPACT_ANGLE: "<<g.devImpactAngle<<" MEAN_STEP: "<<g.meanStep<<" PENALTY: "<<score<<std::endl;
    return score+1.0/(1.0+g.nbStep)+0.1*g.devImpactAngle;
};



int main(int argc, char *argv[])
{
    int dim = 5; // problem dimensions.
        // std::vector<double> x0(dim,0.0);
    std::vector<double> x0({-0.1,-0.3,0,0.8,0});
    double sigma = 0.1;
        //int lambda = 100; // offsprings at each generation.
    CMAParameters<> cmaparams(x0,sigma);
        //cmaparams._algo = BIPOP_CMAES;
        // cmaparams.set_mt_feval(true); //multithread

    CMASolutions cmasols = cmaes<>(walk,cmaparams);
    std::cout << "best solution: " << cmasols << std::endl;
    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    return cmasols.run_status();
}
