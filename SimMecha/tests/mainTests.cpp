#include <iostream>
#include <cassert>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"

using namespace std;
using namespace Leph::SimMecha;

int main()
{
    Leph::SimViewer::SimViewer viewer(800, 600);

    System system(Vector2D(1.0, 1.0));
    Body& b1 = system.addLinearJoint(
        system.getBase(), 
        Vector2D(1.0, 0.0), 0.5, 
        Vector2D(0.0, 0.0), 0.0,
        0.0, 0.0);
    b1.addMass(10.0, Vector2D(1.0, 0.0));
    system.getBase().addMass(1.0, Vector2D(0.0, 0.0));
    system.initSymbols();

    while (viewer.isOpen()) {
        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);
        viewer.endDraw(10);

        system.runSimulationStep(0.001);
    }

    return 0;
}

