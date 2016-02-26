//*****************************************************************************
//
// File Name	: 'testStability.cpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@labri.fr
// Created	: vendredi, fevrier 19 2016
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
#include <random>

#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define DEFAULT "\033[39m"

#define DIST(x,y) (sqrt(pow((x-y),2)))
// #define DIST(x,y) (pow(sqrt(pow((x-y),2))),2)

#define DIST_ANGLE(x,y) (fabs(atan2(sin(x-y), cos(x-y))))
// #define DIST_ANGLE(x,y) (pow(fabs(atan2(sin(x-y), cos(x-y)))),2)
// #define DIST(x,y) (sqrt(pow((10.0*x-10*y),2)))




using namespace std;
using namespace Leph::SimMecha;
using namespace Leph::SimViewer;
using namespace libcmaes;


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


    std::cout<<"STATE: "<<w.current_q1_dot<<" "<<w.current_swing<<" "<<w.current_q2_dot<<std::endl;
    std::cout<<"STEP: "<<w.ground_contact->lastContactPoint.x()<<std::endl;

    // score+=DIST(init_vel,w.current_q1_dot)+DIST(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    // score+=DIST(init_vel,w.current_q1_dot)+DIST_ANGLE(init_swing,w.current_swing)+DIST(init_swing_vel,w.current_q2_dot);

    // score+= max( 1.0/(fabs(init_swing/(M_PI/24.0)))-1.0 , 0.0 );

    // score+=  pow(1.0+w.ground_contact->lastContactPoint.x(),2);



    if(w.state&FALL)
    {
        std::cout<<"FALLEN"<<std::endl;
        return false;

    }
    // std::cout<<"SCORE: "<<RED<<score<<DEFAULT<<" nbStep: "<<BLUE<<w.nbStep<<DEFAULT<<std::endl;

    // std::cout<<std::endl;
    state[0]=w.current_q1_dot;
    state[1]=w.current_swing;
    state[2]=w.current_q2_dot;

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
    Eigen::VectorXcd EV=DF.eigenvalues();

    std::cout<<"Tau:"<<std::endl;
    std::cout<<Tau<<std::endl;
    std::cout<<"Psi:"<<std::endl;
    std::cout<<Psi<<std::endl;
    std::cout<<"Df:"<<std::endl;
    std::cout<<DF<<std::endl;
    std::cout<<"Eigenvalues:"<<std::endl;
    std::cout<<EV<<std::endl;
    std::cout<<"Eigenvalues norm:"<<std::endl;
    std::cout<<std::abs(EV(0))<<std::endl;
    std::cout<<std::abs(EV(1))<<std::endl;
    std::cout<<std::abs(EV(2))<<std::endl;
    // std::cout<<EV[1].norm()<<std::endl;
    // std::cout<<EV[2].norm()<<std::endl;

    // std::cout<<EV.norm()<<std::endl;
    return DF;
}

Eigen::MatrixXd computeStabilityPseudoInv(Json::Value conf, int nbIter=10,double delta=10e-8)
{
    //3 dimensions
    std::vector<double> init_state;
    init_state.push_back(conf["model"]["init_state"]["q1_dot"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["swing"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["q2_dot"].asDouble());

    /*
      Eigen::Matrix<double, 3, _NB_ITER*3> Psi;
      Eigen::Matrix<double, 3, _NB_ITER*3> Tau;
      Tau=Eigen::Matrix<double, 3, _NB_ITER*3>::Zero();
    */

    Eigen::MatrixXd Psi(3, nbIter*3);
    Eigen::MatrixXd Tau(3, nbIter*3);
    Tau=Eigen::MatrixXd::Zero(3, nbIter*3);

    std::random_device rd;
    std::default_random_engine re(rd());
    std::uniform_real_distribution<double> unif(-delta, delta);
    std::vector<double> deltas={0.0,0.0,0.0};

    for(int k=0;k<nbIter;k++)
    {
        std::cout<<"\tk: "<<k<<std::endl;

        //make som small delta variations on each dimension
        for(int i=0;i<3;i++)
        {
            std::cout<<"\t\ti: "<<i<<std::endl;
            std::vector<double> state(init_state);

            double r=unif(re);
            state[i]+=r;
            deltas[i]=r;

            if(!makeStep(conf,state))
                std::cerr<<"Problem"<<std::endl;

            std::vector<double> diff;
            for(int j=0;j<3;j++)
                diff.push_back(state[j]-init_state[j]);


            Eigen::Vector3d v(diff.data());

            Psi.col(k*3+i)=v;
            Eigen::Vector3d d(deltas.data());

            Tau.col(k*3+i)=d;
            deltas[i]=0.0;
        }
    }

    Eigen::MatrixXd DF=Tau.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Psi.transpose());

    // Eigen::VectorXcd EV=DF.eigenvalues();

    Eigen:: EigenSolver<Eigen::MatrixXd> es(DF.transpose());


    std::cout<<"Tau:"<<std::endl;
    std::cout<<Tau<<std::endl;
    std::cout<<"Psi:"<<std::endl;
    std::cout<<Psi<<std::endl;
    std::cout<<"Df:"<<std::endl;
    std::cout<<DF.transpose()<<std::endl;

    std::cout<<"Eigenvectors:"<<std::endl;
    std::cout<<es.eigenvectors()<<std::endl;
    std::cout<<"Eigenvalues:"<<std::endl;
    std::cout<<es.eigenvalues()<<std::endl;
    std::cout<<"Eigenvalues norm:"<<std::endl;
    std::cout<<std::abs(es.eigenvalues()(0))<<std::endl;
    std::cout<<std::abs(es.eigenvalues()(1))<<std::endl;
    std::cout<<std::abs(es.eigenvalues()(2))<<std::endl;

    return DF.transpose();
}

void testNewtonRaphson(Json::Value conf)
{
    //3 dimensions
    std::vector<double> init_state;
    init_state.push_back(conf["model"]["init_state"]["q1_dot"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["swing"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["q2_dot"].asDouble());
    Eigen::Vector3d xn(init_state.data()); // x_n=x0
    Eigen::Vector3d xn_p1; //x_n+1
    std::cout<<std::setprecision(25);
    double norm=999.9;
    // for(int i=0;i<10;i++)
    while(norm>10e-14)
    {
        std::vector<double> state(init_state.size());
        Eigen::Map<Eigen::Vector3d>(state.data(),init_state.size())=xn; //how convenient is this...

        Json::Value tmp=conf;
        tmp["model"]["init_state"]["q1_dot"]=state[0];
        tmp["model"]["init_state"]["swing"]=state[1];
        tmp["model"]["init_state"]["q2_dot"]=state[2];

        if(!makeStep(tmp,state))
            std::cerr<<"Problem"<<std::endl;

        std::vector<double> diff;
        for(int j=0;j<3;j++)
            diff.push_back(state[j]-init_state[j]);

        //F_xn= F(xn)-x0
        Eigen::Vector3d F_xn(diff.data());

        Eigen::MatrixXd DF=computeStability(tmp);

        xn_p1=xn-DF.inverse()*F_xn;
        xn=xn_p1;

        std::cout<<"F(Xn): "<<std::endl;
        std::cout<<F_xn<<std::endl;
        std::cout<<"Xn: "<<std::endl;
        std::cout<<xn_p1<<std::endl;
        norm=F_xn.norm();
        std::cout<<"NORM: "<<RED<<norm<<DEFAULT<<std::endl;
    }

}


void logEigenvectors(Json::Value conf)
{

    //3 dimensions
    std::vector<double> init_state;
    init_state.push_back(conf["model"]["init_state"]["q1_dot"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["swing"].asDouble());
    init_state.push_back(conf["model"]["init_state"]["q2_dot"].asDouble());

    std::cout<<std::setprecision(25);

    ofstream logfile;
    logfile.open("log_eigen.dat");
    logfile<<std::setprecision(25);
    std::vector<double> state(init_state);

    for(int i=0;i<200;i++)
        // while(norm>10e-14)
    {


        Json::Value tmp=conf;
        tmp["model"]["init_state"]["q1_dot"]=state[0];
        tmp["model"]["init_state"]["swing"]=state[1];
        tmp["model"]["init_state"]["q2_dot"]=state[2];


        if(!makeStep(tmp,state))
            std::cerr<<"Problem"<<std::endl;


        Eigen::MatrixXd DF=computeStability(tmp);
        Eigen::EigenSolver<Eigen::MatrixXd> es(DF);

        // logfile<<i<<" "<<state[0]<<" "<<state[1]<<" "<<state[2]<<" "<<std::abs(es.eigenvalues()(0))<<" "<<std::abs(es.eigenvalues()(1))<<" "<<std::abs(es.eigenvalues()(2))<<" "<<es.eigenvectors().col(0)(0).real()<<std::endl;;

        logfile<<i<<" ";
        for(int i=0;i<3;i++)
            logfile<<state[i]<<" ";
        for(int i=0;i<3;i++)
            logfile<<es.eigenvalues()(i).real()<<" "<<es.eigenvalues()(i).imag()<<" ";
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                logfile<<es.eigenvectors().col(i)(j).real()<<" "<<es.eigenvectors().col(i)(j).imag()<<" ";
        logfile<<std::endl;



    }

    logfile.close();
}

int main(int argc, char* argv[])
{

    if(argc==2)
    {
        Json::Value conf_root;
        std::ifstream jsonfile (argv[1]);
        jsonfile >> conf_root;
        // computeStability(conf_root);
        // computeStabilityPseudoInv(conf_root);
        // testNewtonRaphson(conf_root);
        logEigenvectors(conf_root);
    }
    else
    {
        std::cout<<"require a json conf"<<std::endl;
        return -1;
    }


}
