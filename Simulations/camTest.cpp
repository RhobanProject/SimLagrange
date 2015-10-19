#include <iostream>
#include <cassert>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"
#include "SimMecha/src/Constraint.hpp"
#include "SimMecha/src/UnaryConstraint.hpp"
#include "SimMecha/src/BinaryConstraint.hpp"

#include "SimMecha/src/HeightUnaryContraint.hpp"

#include <stdio.h>

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
    Leph::SimViewer::SimViewer viewer(1024, 800);

    Leph::Any::Any param;

    viewer.setSpaceHandler((SimViewer::HandlerFunction)space_cb, param);
    viewer.setRHandler((SimViewer::HandlerFunction)R_cb, param);


    System system(Vector2D(-1.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(1, Vector2D(0.0, 0.0));


    Body& b1 = system.addCamJoint(
        system.getBase(),
        Vector2D(0.0, 0.4), 0.,
        Vector2D(0.0, 0.0), 0.0,
        0.0, 8, 0.29, 0.0, 0.0, 0.0);
    b1.addMass(0.1, Vector2D(0.0, 0.4));


    // Body& b2 = system.addCamJointInverted(
    //     b1,
    //     Vector2D(0.0, 0.4), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.0, 8, 0.29, 0.0, 0.1, 0.0);
    // b2.addMass(0.5, Vector2D(0.0, 0.2));


    // Body& b2 = system.addCamJointInverted(
    //     b1,
    //     Vector2D(0.0, 0.0), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.0, 8, 0.29, 0.0, 0.0, 0.0);
    // b2.addMass(0.5, Vector2D(0.0, 0.2));



    ////


    Body& b2 = system.addAngularJoint(
        b1,

        Vector2D(0.0, 0.4), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        M_PI+0.2, 0.0);
    b2.addMass(0.1, Vector2D(0.0, 0.4));



    Body& b3 = system.addCamJointInverted(
        b2,
        Vector2D(0.0, 0.8), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.0, 8, 0.29, 0.0, 0, 0.0);
    b3.addMass(0.1, Vector2D(0.0, 0.4));


    // Body& b3 = system.addAngularJoint(
    //     b2,
    //     Vector2D(1.0, 0.0), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.2, 0.0);
    // b3.addMass(1.0, Vector2D(1.0, 0.0));

    system.initSymbols();

    /*
    HeightUnaryContraint c1(
        b3, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryContraint c2(
        b2, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryContraint c3(
        b1, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryContraint c4(
        system.getBase(), system, 0.90, false, 0.0, Vector2D(0.0, 0.0));
    */

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
        /*
        c1.handle();
        c2.handle();
        c3.handle();
        c4.handle();
        */
    }

    return 0;
}
