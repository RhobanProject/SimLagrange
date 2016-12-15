//*****************************************************************************
//
// File Name	: 'KneeWalkerOptim.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@inria.fr
// Created	: mercredi, janvier 27 2016
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	notes
//
//*****************************************************************************


#include "Model/PassiveWalkerWithKnee.hpp"

#include <iostream>
#include <fstream>
#include <cmaes/cmaes.h>

#define DIST(x,y) (sqrt(pow((x-y),2)))
// #define DIST(x,y) (pow(sqrt(pow((x-y),2))),2)

#define DIST_ANGLE(x,y) (fabs(atan2(sin(x-y), cos(x-y))))
// #define DIST_ANGLE(x,y) (pow(fabs(atan2(sin(x-y), cos(x-y)))),2)
// #define DIST(x,y) (sqrt(pow((10.0*x-10*y),2)))


#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define DEFAULT "\033[39m"

using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;
using namespace libcmaes;


FitFunc walk=[](const double *x, const int N)
{

    double slope=x[0];
    double init_vel=x[1];
    double init_swing=x[2];
    double init_swing_vel=x[3];

    double score=0.0;


    //Punishement on bounds
    if(slope>0.0)
        score+=1000.0;
    if(slope<-0.25)
        score+=1000.0;

    if(fabs(init_vel)>10.0)
        score+=1000.0;

    // if(init_vel>0.0)
    //     score+=1000.0;

    if((init_swing)>0.0)
        score+=1000.0;


    if(fabs(init_swing)>1.5)
        score+=1000.0;


    // if(fabs(init_swing)>1.0)
    //     score+=1000.0;

    // if(fabs(init_swing)>2.0)
    //     score+=1000.0;

    PassiveWalkerWithKnee w(slope, init_vel, init_swing, init_swing_vel, false);
    while(!(w.state&FALL) && (w.nbStep<1))
    {
        w.SimuStep(0.001);
    }

    //--------------


    std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<std::endl;
    std::cout<<"STATE: "<<w.current_q1_dot<<" "<<-w.current_swing<<" "<<w.current_q2_dot<<std::endl;
    std::cout<<"STEP: "<<w.ground_contact->lastContactPoint.x()<<std::endl;

    // score+=DIST(init_vel,w.current_q1_dot)+DIST(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    score+=DIST(init_vel,w.current_q1_dot)+DIST_ANGLE(init_swing,-w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    score+= max( 1.0/(fabs(init_swing/(M_PI/12.0)))-1.0 , 0.0 );

    // score+=  pow(1.0+w.ground_contact->lastContactPoint.x(),2);



    if(w.state&FALL)
    {
        std::cout<<"FALLEN"<<std::endl;
        score+=10;

    }
    std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<" nbStep: "<<BLUE<<w.nbStep<<DEFAULT<<std::endl;

    std::cout<<std::endl;

    return score;
};



int main(int argc, char* argv[])
{


    int dim = 4; // problem dimensions.

    std::vector<double> x0({-0.1,-1.4,-0.5,1.5});
    double sigma = 0.1;
    //int lambda = 100; // offsprings at each generation.
    CMAParameters<> cmaparams(x0,sigma);

    // cmaparams.set_mt_feval(true); //multithread

    // cmaparams.set_algo(aCMAES); //standard
    cmaparams.set_algo(aBIPOP_CMAES);
    // cmaparams.set_elitism(true);
    // cmaparams.set_restarts(3);
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
