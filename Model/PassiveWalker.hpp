//*****************************************************************************
//
// File Name	: 'PassiveWalker.hpp'
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

#if !defined(PASSIVEWALKER_HPP)
#define PASSIVEWALKER_HPP



#include <iostream>
#include <fstream>
#include <string>
#include <json/json.h>
#include "SimViewer/src/SimViewer.hpp"

using namespace Leph::SimMecha;
using namespace Leph::SimViewer;

class PassiveWalker
{
  public:

    Json::Value conf_root;

    bool simu_pause;
    bool simu_reset;
    Leph::Any::Any param;

    PassiveWalker(){};
    PassiveWalker(const char* jsonconf)
    {
        std::ifstream jsonfile (jsonconf);
        jsonfile >> conf_root;

        // std::cout<<"JSON:"<<std::endl;
        // std::cout<<conf_root;

        simu_pause=false;
        simu_reset=false;

    }

    void space_cb(Leph::Any::Any param)
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

    void R_cb(Leph::Any::Any param)
    {
        simu_reset=true;
        std::cout<<"RESET"<<std::endl;
    }

    virtual void BuildModel(){};


};

#endif
