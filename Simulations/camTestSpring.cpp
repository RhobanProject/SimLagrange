#include <iostream>

#include "SimMecha/src/Body.hpp"
#include "SimMecha/src/Joint.hpp"
#include "SimMecha/src/System.hpp"
#include "SimViewer/src/SimViewer.hpp"
#include "SimMecha/src/Constraint.hpp"
#include "SimMecha/src/UnaryConstraint.hpp"
//#include "SimMecha/src/BinaryConstraint.hpp"
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/terms.h"
#include "SimMecha/src/Simulation.h"

#include "SimMecha/src/HeightUnaryConstraint.hpp"

#include <stdio.h>
#include <functional>
#include <cstdlib>

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



    //parabolic
    // TermPtr a=Constant::create(0.001);
    // TermPtr b=Constant::create(6.0);


    // auto F = [&a, &b](TermPtr x) -> TermPtr
    //     {
    //         return Leph::Symbolic::Add<scalar>::create(Leph::Symbolic::Mult<scalar, scalar, scalar>::create(a,x),Leph::Symbolic::Mult<scalar, scalar, scalar>::create(b,Leph::Symbolic::Pow<scalar>::create(x,2)));
    //     };

    // hyperbolic
    TermPtr a=Constant::create(0.001);
    TermPtr b=Constant::create(0.025);
    TermPtr c=Constant::create(6.0);

    auto F = [&a, &b, &c](TermPtr x) -> TermPtr
             {
                 //a/(b+x)+c.x²
                 return Leph::Symbolic::Add<scalar>::create(
                     Leph::Symbolic::Frac<scalar>::create( a,
                                                           Leph::Symbolic::Add<scalar>::create(b,x) ),
                     Leph::Symbolic::Mult<scalar, scalar, scalar>::create( c, Leph::Symbolic::Pow<scalar>::create(x,2) )
                                                            );

             };


    //numerical version
    const double na=0.001;
    const double nb=0.025;
    const double nc=6.0;

    auto nF = [&na, &nb, &nc](double x) -> double
              {
                  //a/(b+x)+c.x²
                  return na/(nb+x)+nc*pow(x,2);
              };

    std::cout << "DEBUG, initial length: "<<nF(0.0) << "\n";

    // auto mF = [&a, &b, &c](TermPtr x) -> TermPtr
    //     {

    //         return Leph::Symbolic::Minus<scalar>::create(Leph::Symbolic::Add<scalar>::create( Leph::Symbolic::Frac<scalar>::create( a, Leph::Symbolic::Add<scalar>::create(b,x) ), Leph::Symbolic::Mult<scalar, scalar, scalar>::create( c, Leph::Symbolic::Pow<scalar>::create(x,2) ) ));

    //     };



    System system(Vector2D(-1.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(1, Vector2D(0.0, 0.0));







    // Body& b1 = system.addCamSpringJoint(
    //     system.getBase(),
    //     Vector2D(0.0, 0.4), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     F,0.29, 0.0, 5, nF(0.0),0.5,0.0);
    // b1.addMass(0.1, Vector2D(0.0, 0.4));

    // Body& b1 = system.addCamSpringJointInverted(
    //     system.getBase(),
    //     Vector2D(0.0, 0.4), 0.0,
    //     Vector2D(0.0, 0.0), 0.0,
    //     F,0.29, 0.0, 5, nF(0.0),-0.5,0.0);
    // b1.addMass(0.1, Vector2D(0.0, 0.4));



    // test inverted
    Body& b1 = system.addAngularJoint(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        M_PI+0.5, 1.0);
    b1.addMass(1, Vector2D(0.0, 0.4));



    Body& b2 = system.addCamSpringJointInverted(
        b1,
        Vector2D(0.0, 0.8), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        F,0.29, 0.0, 50, nF(0.0),-0.5,0.0);
    b2.addMass(1, Vector2D(0.0, 0.4));



    system.initSymbols();

    // Ground g(b2, system, 0.9, false, F_ground, Vector2D(0.0, 1.0));

    /*
      HeightUnaryConstraint c1(
      b3, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
      HeightUnaryConstraint c2(
      b2, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
      HeightUnaryConstraint c3(
      b1, system, 0.90, false, 0.0, Vector2D(1.0, 0.0));
      HeightUnaryConstraint c4(
      system.getBase(), system, 0.90, false, 0.0, Vector2D(0.0, 0.0));
    */


    //animation stuff
    // sf::View view(viewer._window.getView());
    // view.zoom(0.3);
    // viewer._window.setView(view);
    // viewer.moveCam(10,0.5);
    // std::stringstream filename;

    // int i=0;

    while (viewer.isOpen()) {
        viewer.beginDraw();
        viewer.drawFrame();
        system.draw(viewer);
        // g.draw(viewer);

        viewer.endDraw(10);

        // sf::Image Screen = viewer._window.capture();
        // filename.str(std::string());
        // filename.clear();
        // filename<<std::setfill('0') << std::setw(3)<<i++<<".bmp";
        // Screen.saveToFile(filename.str());

        try{
            if(simu_reset){
                system.stateReset();
                simu_reset=false;
            }
            if(!simu_pause)
                system.runSimulationStep(0.001);
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

        // g.handle();
    }

    return 0;
}
