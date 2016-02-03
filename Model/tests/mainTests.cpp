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


int main(int argc, char* argv[])
{
    try{
        PassiveWalkerWithKnee w(argv[1], true);

        // for(int i=0;i<100;i++)
        // w.SimuStep(0.001);
        while(!(w.state&FALL)) //!w.state&FALL
        {
            // w.draw();
            w.SimuStep(0.001);
        }
        std::cout<<"FALLEN"<<std::endl;
    }
    catch(const char* e){
        std::cout<<e<<std::endl;
    }

    return 0;

}
