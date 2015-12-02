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

#include <iomanip>
#include <ctime>

#include <stdio.h>
#include <functional>

#include <cmaes/cmaes.h>
#include "Simulations/RoundFeetWalkerGround.hpp"

// #define DIST(x,y) (sqrt(pow((x-y),2)))
#define DIST(x,y) (sqrt(pow((10.0*x-10*y),2)))
#define SKIP_FRAME 20

// #define DRAW


#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define DEFAULT "\033[39m"

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

    // double slope=x[0];
    // // double slope=-0.1;
    // double init_angle=x[1];
    // double init_vel=x[2];
    // double init_swingangle=x[3];
    // double init_swingvel=x[4];
    // // double init_swingvel=0.0;


    // double foot_radius=1.11956469704295;//0.2;
    // // double slope=-0.1;
    // double init_vel=-0.237516716283862*foot_radius;
    // double init_swingangle=-0.299266047913129;
    // double init_swingvel=-0.00427183120241122;
    // double slope=-0.00887709411969127;




    double foot_radius=x[4];//0.2;
    // double foot_radius=2.0;
    // double slope=-0.1;
    double init_vel=x[0]*foot_radius;
    double init_swingangle=x[1];
    double init_swingvel=x[2];
    double slope=x[3];


        //test

    // double foot_radius=1.0;
    // double slope=-0.1;
    // double init_vel=-0.1*foot_radius;
    // double init_swingangle=-0.1;
    // double init_swingvel=0.5;
    // double slope=x[3];


    // double init_vel=0.848809699067517 *0.2;
    // double init_swingangle=-0.0125287420878489;
    // double init_swingvel=0.108215801468413;
    // double slope= -0.391331903396806;



    // double slope=-0.02;
    double init_angle=((M_PI-init_swingangle)/2.0+atan2(slope,1.0)-M_PI/2.0)*foot_radius;

    if(foot_radius<0.001)
        score+=1000.0;

    // if(foot_radius>1.0)
    //     score+=1000.0;

    // score+=foot_radius*0.1;

        //check range
    if(slope>0.0)
        score+=1000.0;
    if(slope<-0.5)
        score+=1000.0;

    // if(fabs(init_angle)>1.0*foot_radius)
    //     score+=1000.0;

    if(fabs(init_vel)>2.0)
        score+=1000.0;

    if(init_vel>0.0)
        score+=1000.0;


    if(fabs(init_swingangle)>1.0)
        score+=1000.0;

    if(fabs(init_swingvel)>2.0)
        score+=1000.0;



    // double ga=-0.21;
    double ga=slope;
    // double ga=0.0;
    // double gb=-foot_radius;//-1.3;
    double gb=0;//-1.3;

    auto F_ground = [&ga, &gb](double x) -> double
        {
            // gb+=0.000001;
            return ga*x+gb;
        };



    System system(Vector2D(0.0, 0.0));
    //System system(Vector2D(-1.0, 1.0), Vector2D());
    system.getBase().addMass(0.1, Vector2D(0.0, 0.0));



    Body& b1 = system.addRoundFoot(
        system.getBase(),
        Vector2D(0.0, 0), 0.0,
        Vector2D(0.0, foot_radius), 0.0,
        foot_radius, atan2(slope,1.0) , init_angle, init_vel);
    b1.addMass(0.001, Vector2D(0.0, 2.0));

    Body& b2 = system.addAngularJoint(
        b1,
        Vector2D(0.0, 2.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_swingangle, init_swingvel);
    b2.addMass(0.001, Vector2D(0.0, -2.0));

    b2.addMass(1, Vector2D(0.0, 0.0));

    system.initSymbols();

    RoundFeetGround g(b2, system, 0.9, false, F_ground, Vector2D(0.0, -2.0+foot_radius));

    g.feetRadius=foot_radius;

//small
    /*
    Body& b1 = system.addAngularJoint(
        system.getBase(),
        Vector2D(0.0, 0.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_angle, init_vel);
    b1.addMass(0.001, Vector2D(0.0, 1.0));


    Body& b2 = system.addAngularJoint(
        b1,
        Vector2D(0.0, 1.0), 0.0,
        Vector2D(0.0, 0.0), 0.0,
        init_swingangle, init_swingvel);
    b2.addMass(0.001, Vector2D(0.0, -1.0));

    b2.addMass(1, Vector2D(0.0, 0.0));

    system.initSymbols();

    SimpleWalkerGround g(b2, system, 0.9, false, F_ground, Vector2D(0.0, -1.0));
*/

    double t=0.0;
    int skip=0;
    // while (viewer.isOpen() && t<10.0 && !g.hasFallen) {
    while (t<10.0 && !g.hasFallen) { //5.5s is a bit long
    // while(true){
        #ifdef DRAW
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
        #endif

        try{
            if(simu_reset){
                system.stateReset();
                simu_reset=false;
            }
            if(!simu_pause){
                system.runSimulationStep(0.001);
                t+=0.001;
                    // std::cout<<"TIME: "<<t<<std::endl;
                g.handle();

            }
        }
        catch(const std::exception & e)
        {
            std::cerr << e.what()<<std::endl;
        }



        if(g.nbStep>=1) //we made one step
        {

                //Look is fixed point
            // std::cout<<"PARAMS: "<<x[0]*0.2<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<std::endl;
            // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swingangle<<" "<<init_swingvel<<" "<<slope<<std::endl;
            std::cout<<"PARAMS: "<<init_vel<<" "<<init_swingangle<<" "<<init_swingvel<<" "<<slope<<" "<<foot_radius<<std::endl;


            // std::cout<<"PARAMS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<std::endl;
            // std::cout<<"STATE: "<<g.impactAngle<<" "<<g.impactVel<<" "<<g.impactAngleQ2<<" "<<g.impactVelQ2<<std::endl;
            std::cout<<"STATE: "<<g.impactVel<<" "<<g.impactAngleQ2<<" "<<g.impactVelQ2<<std::endl;
            score+=DIST(init_vel,g.impactVel)+DIST(init_swingangle,g.impactAngleQ2)+DIST(init_swingvel,g.impactVelQ2);
        // score=score/(1/+fabs(g._currentpos.x())*10.0+t*10.0);
            // score+=1.0/(fabs(g._currentpos.x()));
                // score+=10.0/(1.0+10*t);

            score+= max( 1.0/(fabs(init_swingangle/(M_PI/12.0)))-1.0 , 0.0 );
            // score+= max( 1.0/(fabs(init_swingangle/(M_PI/24.0)))-1.0 , 0.0 );



            std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<" nbStep: "<<BLUE<<g.nbStep<<DEFAULT<<std::endl;
            std::cout<<std::endl;
            return score;
        }

        if(g.hasFallen)
        {

            std::cout<<"FALLEN"<<std::endl;
                //Look is fixed point
            // std::cout<<"PARAMS: "<<x[0]*0.2<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<std::endl;
            // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swingangle<<" "<<init_swingvel<<" "<<slope<<std::endl;
            std::cout<<"PARAMS: "<<init_vel<<" "<<init_swingangle<<" "<<init_swingvel<<" "<<slope<<" "<<foot_radius<<std::endl;
            // std::cout<<"PARAMS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<std::endl;
            // std::cout<<"STATE: "<<g.impactAngle<<" "<<g.impactVel<<" "<<g.impactAngleQ2<<" "<<g.impactVelQ2<<std::endl;
            std::cout<<"STATE: "<<g.impactVel<<" "<<g.impactAngleQ2<<" "<<g.impactVelQ2<<std::endl;

            // score+=DIST(init_vel,g.impactVel)+DIST(init_swingangle,g.impactAngleQ2)+DIST(init_swingvel,g.impactVelQ2);

                // score=score/(1/+fabs(g._currentpos.x())*10.0+t*10.0)+1.0;
            // score+=1.0/(fabs(g._currentpos.x()));
                // score+=10.0/(1.0+10*t);
            // score+=1.0/(fabs(init_swingangle));
            score+= max( 1.0/(fabs(init_swingangle/(M_PI/12.0)))-1.0 , 0.0 );
            // score+= max( 1.0/(fabs(init_swingangle/(M_PI/24.0)))-1.0 , 0.0 );
            // if(g.nbStep==2)
            //     score+=100;
            // if(g.nbStep==1)
            //     score+=100;
            // if(g.nbStep==0)
            //     score+=100;

            score+=10;

            // std::cout<<"SCORE: "<<score<<" nbStep: "<<g.nbStep<<std::endl;
            std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<" nbStep: "<<BLUE<<g.nbStep<<DEFAULT<<std::endl;
            std::cout<<std::endl;
            return score;

        }


    }

        std::cout<<"WTF "<<t<<std::endl;

        //just in case

        return 10000.0;
//Limit cycle?

    // std::cout<<"PARAMS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<std::endl;
    // // std::cout<<"PARAMS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<std::endl;
    // // std::cout<<"SCORE: "<<score+1.0/(1.0+g._currentpos.x()+g.nbStep)+g.devImpactAngle<<" DISTANCE: "<<g._currentpos.x()<<" STEPS: "<<g.nbStep<<" DEV_IMPACT_ANGLE: "<<g.devImpactAngle<<" MEAN_STEP: "<<g.meanStep<<" PENALTY: "<<score<<std::endl;
    // std::cout<<"SCORE: "<<score+1.0/(1.0+g.nbStep)+0.1*g.devImpactAngle<<" DISTANCE: "<<g._currentpos.x()<<" STEPS: "<<g.nbStep<<" DEV_IMPACT_ANGLE: "<<g.devImpactAngle<<" MEAN_STEP: "<<g.meanStep<<" PENALTY: "<<score<<std::endl;
    // return score+1.0/(1.0+g.nbStep)+0.1*g.devImpactAngle+1.0/fabs(g._currentpos.x());
};

int main(int argc, char *argv[])
{



    int dim = 5; // problem dimensions.
        // std::vector<double> x0(dim,0.0);
    // std::vector<double> x0({-0.4,-0.35,-0.025,-0.005});
    std::vector<double> x0({-0.4,-0.35,-0.025,-0.005,1});
    // std::vector<double> x0({-0.4,-0.35,-0.025});
    // std::vector<double> x0({-0.6,-0.3,0.3});
    double sigma = 0.1;
        //int lambda = 100; // offsprings at each generation.
    CMAParameters<> cmaparams(x0,sigma);
        //cmaparams._algo = BIPOP_CMAES;
    cmaparams.set_mt_feval(true); //multithread
    // cmaparams.set_algo(aCMAES); //standard

    cmaparams.set_algo(aBIPOP_CMAES);
    cmaparams.set_restarts(1);


    cmaparams.set_ftarget(1e-10);



    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y_%I%M.dat",timeinfo);
    std::string str(buffer);

    cmaparams.set_fplot(str);


    std::cout.precision(15);
    CMASolutions cmasols = cmaes<>(walk,cmaparams);
    std::cout << "best solution: " << cmasols << std::endl;
    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";

    return cmasols.run_status();
}
