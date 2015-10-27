#include <iostream>
#include <cassert>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
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
    Leph::SimViewer::SimViewer viewer(1280, 1024);

    Leph::Any::Any param;

    viewer.setSpaceHandler((SimViewer::HandlerFunction)space_cb, param);
    viewer.setRHandler((SimViewer::HandlerFunction)R_cb, param);


    TermPtr a=Constant::create(0.001);
    TermPtr b=Constant::create(0.1);
    TermPtr c=Constant::create(6.0);

    //parabolic
    // auto F = [&a, &b](TermPtr x) -> TermPtr
    //     {
    //         return Leph::Symbolic::Add<scalar>::create(Leph::Symbolic::Mult<scalar, scalar, scalar>::create(a,x),Leph::Symbolic::Mult<scalar, scalar, scalar>::create(b,Leph::Symbolic::Pow<scalar>::create(x,2)));
    //     };

    //hyperbolic
    auto F = [&a, &b, &c](TermPtr x) -> TermPtr
        {

            return Leph::Symbolic::Add<scalar>::create( Leph::Symbolic::Frac<scalar>::create( a, Leph::Symbolic::Add<scalar>::create(b,x) ), Leph::Symbolic::Mult<scalar, scalar, scalar>::create( c, Leph::Symbolic::Pow<scalar>::create(x,2) ) );

        };




    System system(Vector2D(-1.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(1, Vector2D(0.0, 0.0));



    Body& b1 = system.addLinearSpring(
        system.getBase(),
        Vector2D(0.0, 0.4), M_PI/2.0,
        Vector2D(0.0, 0.0), 0.0,
            1.0, 0.1, 0.0, 0.0);
    b1.addMass(0.1, Vector2D(0.4, 0.0));


    // Body& b2 = system.addAngularJoint(
    //     b1,
    //     Vector2D(0.0, 1.6), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.0, 0.0);
    // b2.addMass(0.1, Vector2D(0.0, 0.8));


    // Body& b2 = system.addLinearJoint(
    //     b1,
    //     Vector2D(0.0, 0.4), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.0, 0.0);
    // b2.addMass(0.1, Vector2D(0.0, 0.4));





    system.initSymbols();

    while (viewer.isOpen()) {
        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);

        viewer.endDraw(10);

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

    }

    return 0;
}
