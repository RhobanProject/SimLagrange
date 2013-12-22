#ifndef FLYMODEL_HPP
#define FLYMODEL_HPP

#include "Model.hpp"
#include "Vector/src/Vector2D.hpp"
#include "Symbolic/src/Constant.hpp"
#include "Symbolic/src/terms.h"

namespace Leph {

/**
 * FlyModel
 */
class FlyModel : public Model
{
    public:

        /**
         * @Inherit
         */
        FlyModel(const Model::ParameterContainer& parameters) :
            Model(parameters)
        {
        }

        /**
         * @Inherit
         */
        virtual inline void initialization()
        {
            //Initialization of degrees of freedom
            Model::addDof("theta");

            //Definition of the mechanical model
            Model::TermVectorPtr origin = 
                Symbolic::Constant<Model::Vector2D>::
                create(Model::Vector2D(0.0, 0.0));
            Model::TermPtr minusTheta = 
                Symbolic::Minus<Model::scalar>::
                create(Model::_dofs["theta"]);
            Model::TermPtr l = 
                Symbolic::Constant<Model::scalar>::
                create(Model::_parameters.at("l"));
            Model::TermVectorPtr tmp1 = 
                Symbolic::PolarInv<Model::Vector2D,Model::scalar>::
                create(minusTheta);
            Model::TermVectorPtr tmp2 = 
                Symbolic::Mult<Model::Vector2D,Model::scalar,Model::Vector2D>::
                create(l, tmp1);
            Model::TermVectorPtr positionP =
                Symbolic::Sub<Model::Vector2D>::
                create(origin, tmp2);
            Model::TermVectorPtr velocityP =
                positionP->derivate(Model::_time);

            Model::TermPtr velocityPSquared =
                Symbolic::Mult<Model::scalar,Model::Vector2D,Model::Vector2D>::
                create(velocityP, velocityP);

            Model::TermPtr mass = 
                Symbolic::Constant<Model::scalar>::
                create(Model::_parameters.at("mass"));
            Model::TermPtr half = 
                Symbolic::Constant<Model::scalar>::
                create(0.5);
            Model::TermPtr tmp3 = 
                Symbolic::Mult<Model::scalar,Model::scalar,Model::scalar>::
                create(half, mass);
            Model::TermPtr kineticEnergy = 
                Symbolic::Mult<Model::scalar,Model::scalar,Model::scalar>::
                create(tmp3, velocityPSquared);

            Model::TermPtr tmp4 = 
                Symbolic::Mult<Model::scalar,Model::scalar,Model::scalar>::
                create(mass, Model::_gravity);
            Model::TermPtr heightP =
                Symbolic::Im<Model::scalar,Model::Vector2D>::
                create(positionP);
            Model::TermPtr potentialEnergy = 
                Symbolic::Mult<Model::scalar,Model::scalar,Model::scalar>::
                create(tmp4, heightP);

            Model::_lagrangian = 
                Symbolic::Sub<Model::scalar>::
                create(kineticEnergy, potentialEnergy);;
        }
};

}

#endif

