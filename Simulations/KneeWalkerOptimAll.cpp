//*****************************************************************************
//
// File Name	: 'KneeWalkerOptimAll.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@labri.fr
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
#include <json/json.h>
#include <iomanip>

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
    double a1=x[4];
    double a2=x[5];

    double mh=x[6];
    double ms=x[7];
    double mt=x[8];


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


    if(fabs(init_swing)>2.5)
        score+=1000.0;

    if(a1<=0.0 || a1>=1.0)
        return 10000.0;
    if(a2<=0.0 || a2 >=1.0)
        return 10000.0;

    if(mh>=2.0 || mh<=0.0)
        return 10000.0;
    if(ms>=2.0 || ms<=0.0)
        return 10000.0;
    if(mt>=2.0 || mt<=0.0)
        return 10000.0;



    // if(fabs(init_swing)>1.0)
    //     score+=1000.0;

    // if(fabs(init_swing)>2.0)
    //     score+=1000.0;


    //Create a json

    Json::Value conf;

    Json::Value model=conf["model"];

    conf["model"]["has_knee"]=true;
    conf["model"]["has_feet"]=false;
    conf["model"]["forced_init"]=false;

    // conf["model"]["parameters"]["m_h"]=1.0;
    // conf["model"]["parameters"]["m_s"]=0.001;
    // conf["model"]["parameters"]["m_t"]=0.001;

    conf["model"]["parameters"]["m_h"]=mh;
    conf["model"]["parameters"]["m_s"]=ms;
    conf["model"]["parameters"]["m_t"]=mt;


    conf["model"]["parameters"]["L"]=2.0;

    conf["model"]["parameters"]["a1"]=a1;
    conf["model"]["parameters"]["a2"]=a2;

    conf["model"]["parameters"]["b1"]=1.0-a1;
    conf["model"]["parameters"]["b2"]=1.0-a2;

    conf["model"]["init_state"]["q1"]=0.0;  //automatic
    conf["model"]["init_state"]["q2"]=0.0;  //automatic cf swing
    conf["model"]["init_state"]["q3"]=0.0;  //automatic
    conf["model"]["init_state"]["q1_dot"]=init_vel;
    conf["model"]["init_state"]["q2_dot"]=init_swing_vel;
    conf["model"]["init_state"]["q3_dot"]=0.0; //automatic
    conf["model"]["init_state"]["swing"]=init_swing;
    conf["world"]["ground"]["slope"]=slope;


    // std::cout<<"DEBUG JSON:"<<std::endl;
    // std::cout<<conf<<std::endl;
    // exit(0);



    // PassiveWalkerWithKnee w(slope, init_vel, init_swing, init_swing_vel, false);

    PassiveWalkerWithKnee w(conf, false);


    while(!(w.state&FALL) && (w.nbStep<1))
    {
        try{
            w.SimuStep(0.001);
        }
        catch(const std::logic_error& e)
        {
            std::cout<<"DEBUG SINGULAR MATRIX?"<<std::endl;
            score=10000.0;
            break;
        }
    }

    //--------------


    // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<std::endl;
    std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<" "<<mh<<" "<<ms<<" "<<mt<<std::endl;


    std::cout<<"STATE: "<<w.current_q1_dot<<" "<<w.current_swing<<" "<<w.current_q2_dot<<std::endl;
    std::cout<<"STEP: "<<w.ground_contact->lastContactPoint.x()<<std::endl;

    // score+=DIST(init_vel,w.current_q1_dot)+DIST(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    score+=DIST(init_vel,w.current_q1_dot)+DIST_ANGLE(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    score+= max( 1.0/(fabs(init_swing/(M_PI/24.0)))-1.0 , 0.0 );

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


void save_solution(std::vector<double> x, const char* filename)
{
    Json::Value conf;
    std::ofstream file;
    file.open(filename);

    double slope=x[0];
    double init_vel=x[1];
    double init_swing=x[2];
    double init_swing_vel=x[3];
    double a1=x[4];
    double a2=x[5];

    double mh=x[6];
    double ms=x[7];
    double mt=x[8];

    conf["model"]["has_knee"]=true;
    conf["model"]["has_feet"]=false;
    conf["model"]["forced_init"]=false;

    // conf["model"]["parameters"]["m_h"]=1.0;
    // conf["model"]["parameters"]["m_s"]=0.001;
    // conf["model"]["parameters"]["m_t"]=0.001;

    conf["model"]["parameters"]["m_h"]=mh;
    conf["model"]["parameters"]["m_s"]=ms;
    conf["model"]["parameters"]["m_t"]=mt;


    conf["model"]["parameters"]["L"]=2.0;

    conf["model"]["parameters"]["a1"]=a1;
    conf["model"]["parameters"]["a2"]=a2;

    conf["model"]["parameters"]["b1"]=1.0-a1;
    conf["model"]["parameters"]["b2"]=1.0-a2;

    conf["model"]["init_state"]["q1"]=0.0;  //automatic
    conf["model"]["init_state"]["q2"]=0.0;  //automatic cf swing
    conf["model"]["init_state"]["q3"]=0.0;  //automatic
    conf["model"]["init_state"]["q1_dot"]=init_vel;
    conf["model"]["init_state"]["q2_dot"]=init_swing_vel;
    conf["model"]["init_state"]["q3_dot"]=0.0; //automatic
    conf["model"]["init_state"]["swing"]=init_swing;
    conf["world"]["ground"]["slope"]=slope;

    Json::StyledWriter Writer;
    file << std::setprecision(25);
    file << Writer.write(conf);
    file.close();
}

int main(int argc, char* argv[])
{


    int dim = 9; // problem dimensions.

    // std::vector<double> x0({-0.1,-1.4,-0.5,1.5,0.5,0.5});

    std::vector<double> x0({-0.1,-1.4,-0.5,1.5,0.5,0.5,1.0,0.001,0.001});
    // double sigma = 0.1;

    //Start from a good solution
    // std::vector<double> x0({-0.00138611487182065,-1.53859247193321,-0.532506967778129,1.9864034821433,0.52119274561097,0.269452637112695,0.13281810737985,0.523979626929375,0.510355579117084});

    double sigma = 0.001;

    //int lambda = 100; // offsprings at each generation.
    CMAParameters<> cmaparams(x0,sigma);

    cmaparams.set_mt_feval(true); //multithread

    // cmaparams.set_algo(aCMAES); //standard
    cmaparams.set_algo(aBIPOP_CMAES);
    // cmaparams.set_elitism(true);
    // cmaparams.set_restarts(3);
    cmaparams.set_restarts(1);

    cmaparams.set_ftarget(1e-15);


    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y_%I%M.dat",timeinfo);
    std::string str(buffer);

    cmaparams.set_fplot(str);


    std::cout.precision(25);
    CMASolutions cmasols = cmaes<>(walk,cmaparams);
    std::cout << "best solution: " << cmasols << std::endl;
    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";

    Candidate bcand = cmasols.best_candidate();
    std::vector<double>x = bcand.get_x();

    strftime(buffer,80,"best_solution_%d-%m-%Y_%I%M.json",timeinfo);
    std::string file(buffer);

    save_solution(x,file.c_str());

    return cmasols.run_status();



}
