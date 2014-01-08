#include <iostream>
#include <cassert>
#include "SimViewer/src/SimViewer.hpp"

using namespace std;
using namespace Leph::SimViewer;

int main()
{
    SimViewer viewer(800, 600);
    double t = 0.0;
    while (viewer.isOpen()) {
        t += 0.3;
        //Handle events
        viewer.beginDraw();
        //Draw first chain
        viewer.beginChain();
        viewer.drawMass();
        viewer.drawJoint(0.3*t);
        viewer.drawSegment(1.0);
        viewer.drawMass();
        viewer.drawJoint(t);
        viewer.drawSegment(2.0);
        viewer.drawMass();
        //Draw second chain
        viewer.beginChain();
        viewer.drawJoint(t);
        viewer.drawSegment(1.5);
        viewer.drawMass();
        //Display 
        viewer.endDraw();
    }

    return 0;
}

