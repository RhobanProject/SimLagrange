//*****************************************************************************
//
// File Name	: 'KneeWalkerOptimEigen.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@labri.fr
// Created	: vendredi, février 26 2016
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
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

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

void save_solution(std::vector<double> x, const char* filename);
bool makeStep(Json::Value conf, std::vector<double> &state);
Eigen::MatrixXd computeStability(Json::Value conf, double delta);


bool makeStep(Json::Value conf, std::vector<double> &state)
{


    //TODO generalize the dimensionality

    // conf["model"]["init_state"]["q1"]=0.0;  //automatic
    // conf["model"]["init_state"]["q2"]=0.0;  //automatic cf swing
    // conf["model"]["init_state"]["q3"]=0.0;  //automatic
    conf["model"]["init_state"]["q1_dot"]=state[0];
    conf["model"]["init_state"]["q2_dot"]=state[2];
    // conf["model"]["init_state"]["q3_dot"]=0.0; //automatic
    conf["model"]["init_state"]["swing"]=state[1];


    PassiveWalkerWithKnee w(conf, false);


    while(!(w.state&FALL) && (w.nbStep<1))
    {
        try{
            w.SimuStep(0.001);
        }
        catch(const std::logic_error& e)
        {
            std::cout<<"DEBUG SINGULAR MATRIX?"<<std::endl;
            return false;
        }
    }

    //--------------


    // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<std::endl;
    // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<" "<<mh<<" "<<ms<<" "<<mt<<std::endl;


    // std::cout<<"STATE: "<<w.current_q1_dot<<" "<<w.current_swing<<" "<<w.current_q2_dot<<std::endl;
    // std::cout<<"STEP: "<<w.ground_contact->lastContactPoint.x()<<std::endl;

    // score+=DIST(init_vel,w.current_q1_dot)+DIST(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    // score+=DIST(init_vel,w.current_q1_dot)+DIST_ANGLE(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    // score+= max( 1.0/(fabs(init_swing/(M_PI/24.0)))-1.0 , 0.0 );

    // score+=  pow(1.0+w.ground_contact->lastContactPoint.x(),2);
    state[0]=w.current_q1_dot;
    state[1]=w.current_swing;
    state[2]=w.current_q2_dot;



    if(w.state&FALL)
    {
        std::cout<<"FALLEN"<<std::endl;
        return false;

    }
    // std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<" nbStep: "<<BLUE<<w.nbStep<<DEFAULT<<std::endl;

    // std::cout<<std::endl;

    return true;
};



Eigen::MatrixXd computeStability(Json::Value conf, double delta=10e-8)
{
    //3 dimensions
    std::vector<double> init_state;
    init_state.push_back(conf["model"]["init_state"]["q1_dot"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["swing"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["q2_dot"].asDouble());

    Eigen::Matrix<double, 3, 3> Psi;

    //make som small delta variations on each dimension
    for(int i=0;i<3;i++)
    {
        std::vector<double> state(init_state);
        state[i]+=delta;
        if(!makeStep(conf,state))
            std::cerr<<"Problem"<<std::endl;

        std::vector<double> diff;
        for(int j=0;j<3;j++)
            diff.push_back(state[j]-init_state[j]);

        Eigen::Vector3d v(diff.data());
        Psi.col(i)=v;

    }


    Eigen::Vector3d deltas(delta,delta,delta);
    Eigen::Matrix<double, 3, 3> Tau=deltas.asDiagonal();
    Eigen::Matrix<double, 3, 3> DF=Psi*Tau.inverse();

    // Eigen::VectorXcd EV=DF.eigenvalues();

    // std::cout<<"Tau:"<<std::endl;
    // std::cout<<Tau<<std::endl;
    // std::cout<<"Psi:"<<std::endl;
    // std::cout<<Psi<<std::endl;
    // std::cout<<"Df:"<<std::endl;
    // std::cout<<DF<<std::endl;
    // std::cout<<"Eigenvalues:"<<std::endl;
    // std::cout<<EV<<std::endl;
    // std::cout<<"Eigenvalues norm:"<<std::endl;
    // std::cout<<std::abs(EV(0))<<std::endl;
    // std::cout<<std::abs(EV(1))<<std::endl;
    // std::cout<<std::abs(EV(2))<<std::endl;


    // std::cout<<EV[1].norm()<<std::endl;
    // std::cout<<EV[2].norm()<<std::endl;

    // std::cout<<EV.norm()<<std::endl;
    return DF;
}



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
        score+=1000000.0;
    if(slope<-0.25)
        score+=1000000.0;

    if(fabs(init_vel)>10.0)
        score+=1000000.0;

    // if(init_vel>0.0)
    //     score+=1000000.0;

    if((init_swing)>0.0)
        score+=1000000.0;


    if(fabs(init_swing)>2.5)
        score+=1000000.0;

    if(a1<=0.0 || a1>=1.0)
        return 1000000.0;
    if(a2<=0.0 || a2 >=1.0)
        return 1000000.0;

    if(mh>=2.0 || mh<=0.0)
        return 1000000.0;
    if(ms>=2.0 || ms<=0.0)
        return 1000000.0;
    if(mt>=2.0 || mt<=0.0)
        return 1000000.0;



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



    std::vector<double> init_state;
    init_state.push_back(init_vel);
    init_state.push_back(init_swing);
    init_state.push_back(init_swing_vel);

    std::vector<double> current_state(init_state);

    /*
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
    */

    bool res=makeStep(conf,current_state);
    if(!res)
        score+=1000.0; //FALLEN

    //--------------


    // std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<std::endl;
    std::cout<<"PARAMS: "<<init_vel<<" "<<init_swing<<" "<<init_swing_vel<<" "<<slope<<" "<<a1<<" "<<a2<<" "<<mh<<" "<<ms<<" "<<mt<<std::endl;


    // std::cout<<"STATE: "<<w.current_q1_dot<<" "<<w.current_swing<<" "<<w.current_q2_dot<<std::endl;
    std::cout<<"STATE: "<<current_state[0]<<" "<<current_state[1]<<" "<<current_state[2]<<std::endl;
    // std::cout<<"STEP: "<<w.ground_contact->lastContactPoint.x()<<std::endl;

    // score+=DIST(init_vel,w.current_q1_dot)+DIST(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    score+=DIST(init_vel,current_state[0])+DIST_ANGLE(init_swing,current_state[1])+DIST(init_swing_vel,current_state[2]);

    score+= max( 1.0/(fabs(init_swing/(M_PI/24.0)))-1.0 , 0.0 );

    // score+=  pow(1.0+w.ground_contact->lastContactPoint.x(),2);

    if(res)
    {
        Eigen::MatrixXd DF=computeStability(conf);
        Eigen::VectorXcd EV=DF.eigenvalues();

        std::cout<<"EIGENVALUES: "<<std::endl;
        std::cout<<EV<<std::endl;

        double maxeigen=0.0;
        for(int it=0;it<3;it++)
            if(std::abs(EV(it))>maxeigen)
                maxeigen=std::abs(EV(it));

        // score+=std::abs(EV(0))/100000.0;
        // score+=std::abs(EV(1))/100000.0;
        // score+=std::abs(EV(2))/100000.0;
        std::cout<<"MAXEIGEN: "<<maxeigen<<std::endl;
        score+=maxeigen/100000.0;

        if(maxeigen<=1.0)
        {
            //Interesting
            time_t rawtime;
            struct tm * timeinfo;
            char buffer[80];

            time (&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(buffer,80,"interesting_solution_%d-%m-%Y_%I%M.json",timeinfo);
            std::string file(buffer);
            std::vector<double> xx;
            xx.assign(x, x + N);
            save_solution(xx,file.c_str());
        }

    }

    std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<std::endl;//" nbStep: "<<BLUE<<w.nbStep<<DEFAULT<<std::endl;

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

    double sigma = 0.1;

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
