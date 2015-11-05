#include <iostream>
#include <cassert>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"
#include "SimMecha/src/Constraint.hpp"
#include "SimMecha/src/UnaryConstraint.hpp"
#include "SimMecha/src/BinaryConstraint.hpp"

#include "SimMecha/src/HeightUnaryConstraint.hpp"

using namespace std;
using namespace Leph::SimMecha;

int main()
{
    Leph::SimViewer::SimViewer viewer(800, 600);

    System system(Vector2D(-1.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(1.0, Vector2D(0.0, 0.0));

    Body& b1 = system.addAngularJoint(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.0, 0.0);
    b1.addMass(1.0, Vector2D(1.0, 0.0));

    Body& b2 = system.addAngularJoint(
        b1,
        Vector2D(1.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.2, 0.0);
    b2.addMass(1.0, Vector2D(1.0, 0.0));

    Body& b3 = system.addAngularJoint(
        b2,
        Vector2D(1.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        0.2, 0.0);
    b3.addMass(1.0, Vector2D(1.0, 0.0));

    system.initSymbols();


    HeightUnaryConstraint c1(
        b3, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryConstraint c2(
        b2, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryConstraint c3(
        b1, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
    HeightUnaryConstraint c4(
        system.getBase(), system, 0.90, false, 0.0, Vector2D(0.0, 0.0));


    while (viewer.isOpen()) {

        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);
        viewer.moveCam(-system.evalPosition(system.getBase()).x(),system.evalPosition(system.getBase()).y());
        viewer.endDraw(10);

        system.runSimulationStep(0.01);


        // c1.handle();
        // c2.handle();
        // c3.handle();
        // c4.handle();

    }

    return 0;
}
