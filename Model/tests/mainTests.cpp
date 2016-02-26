//*****************************************************************************
//
// File Name	: 'mainTests.cpp'
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
#include <iomanip>
#include<chrono>



int main(int argc, char* argv[])
{
    double score=0.0;
    try{
        PassiveWalkerWithKnee w(argv[1], true, 0, true);


        //animation stuff
        // sf::View view(w.viewer->_window.getView());
        // // view.zoom(0.3);
        // wviewer._window.setView(view);
        // viewer.moveCam(00,0.5);
        // std::stringstream filename;
        // int i=0;
        // int im=0;

        int prev=0;
        // for(int i=0;i<100;i++)
        // w.SimuStep(0.001);
        while(1)//(!(w.state&FALL)) //!w.state&FALL
        {
            // w.draw();
            // auto t1=std::chrono::steady_clock::now();
            w.SimuStep(0.001);
            // auto t2=std::chrono::steady_clock::now();
            // auto d=std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            // std::cout<<"TIME: "<<d.count()<<std::endl;




            // if(im++%10==0)
            // {
            //     sf::Image Screen = w.viewer->_window.capture();
            //     filename.str(std::string());
            //     filename.clear();
            //     filename<<std::setfill('0') << std::setw(4)<<i++<<".bmp";
            //     Screen.saveToFile(filename.str());
            // }

            if(w.nbStep==1 && prev==0)
            {
                std::cout<<"STATE: "<<w.current_q1<<" "<<w.current_q1_dot<<" "<<w.current_swing<<" "<<w.current_q2_dot<<std::endl;
                prev=1;
            }
        }
        std::cout<<"FALLEN"<<std::endl;
    }
    catch(const char* e){
        std::cout<<e<<std::endl;
    }

    return 0;

}
