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

void testFloating()
{
    Leph::SimViewer::SimViewer viewer(800, 600);
    System system(Vector2D(-1.0, 1.0), Vector2D(0.0, 0.0));
    system.getBase().addMass(1.0, Vector2D(0.0, 0.0));



    Body& b1 = system.addAngularJoint(
      (system.getBase()),
      Vector2D(0.0, 0.0), 0.0,
      Vector2D(0.0, 0.0), 0.0,
      0.2, 0.0);
    b1.addMass(1.0, Vector2D(0.0, 1.0));

    
    
    HeightUnaryConstraint c1(
      system.getBase(), system, 0.00, true, 0.0, Vector2D(0.0, 0.0));


    HeightUnaryConstraint c2(
      b1, system, 0.00, true, 0.0, Vector2D(0.0, 1.0));


    
    system.initSymbols();
    
    const double dt = 0.01;
    while (viewer.isOpen()) {
      viewer.beginDraw();
      viewer.drawFrame();
      system.draw(viewer);
      viewer.endDraw();
        
      system.runSimulationStep(dt);
      scalar Ep=system.evalPotential();
      scalar Ec=system.evalKinetic();
      std::cout.precision(15);
      std::cout<<"ENERGY: "<<Ep+Ec<<" Ep: "<<Ep<<" Ec: "<<Ec<<std::endl;

      c1.handle(dt);
      c2.handle(dt);

    }
}

void testFixed()
{
    Leph::SimViewer::SimViewer viewer(800, 600);

    System system(Vector2D(-1.0, 1.5));
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
      b1, system, 1.0, false, 0.0, Vector2D(0.0, 0.0));
    HeightUnaryConstraint c2(
      b2, system, 1.0, false, 0.0, Vector2D(0.0, 0.0));
    HeightUnaryConstraint c3(
      b3, system, 1.0, false, 0.0, Vector2D(0.0, 0.0));
    HeightUnaryConstraint c4(
      b3, system, 1.0, false, 0.0, Vector2D(1.0, 0.0));

    // HeightUnaryConstraint c1(
    //   b1, system, 1.0, true, 0.0, Vector2D(0.0, 0.0));
    // HeightUnaryConstraint c2(
    //   b2, system, 1.0, true, 0.0, Vector2D(0.0, 0.0));
    // HeightUnaryConstraint c3(
    //   b3, system, 1.0, true, 0.0, Vector2D(0.0, 0.0));
    // HeightUnaryConstraint c4(
    //   b3, system, 1.0, true, 0.0, Vector2D(1.0, 0.0));

    
    
    const double dt = 0.01;
    while (viewer.isOpen()) {

        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);
        viewer.endDraw();

        system.runSimulationStep(dt);
        scalar Ep=system.evalPotential();
        scalar Ec=system.evalKinetic();
        std::cout.precision(15);
        std::cout<<"ENERGY: "<<Ep+Ec<<" Ep: "<<Ep<<" Ec: "<<Ec<<std::endl;

        c1.handle(dt);
        c2.handle(dt);
        c3.handle(dt);
        c4.handle(dt);
    }
}

int main()
{
    testFloating();
    testFixed();

    return 0;
}
