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

int main()
{
    Leph::SimViewer::SimViewer viewer(1024, 800);

    System system(Vector2D(-1.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(1, Vector2D(0.0, 0.0));

    Body& b1 = system.addCamJoint(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.0, 6, 0.29, 0.0, 0.1, 0.0);
    b1.addMass(0.1, Vector2D(0.0, 0.3));

    Body& b2 = system.addCamJoint(
        b1,
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.0, 6, 0.29, 0.0, 0.0, 0.0);
    b2.addMass(0.1, Vector2D(0.0, 0.3));


    // Body& b2 = system.addAngularJoint(
    //     b1,
    //     Vector2D(1.0, 0.0), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     0.2, 0.0);
    // b2.addMass(1.0, Vector2D(1.0, 0.0));

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

        system.runSimulationStep(0.01);

        /*
        c1.handle();
        c2.handle();
        c3.handle();
        c4.handle();
        */
    }

    return 0;
}
