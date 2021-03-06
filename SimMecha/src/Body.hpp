#ifndef LEPH_SIMMECHA_BODY_HPP
#define LEPH_SIMMECHA_BODY_HPP

#include <vector>
#include "SimMecha/src/Simulation.h"
#include "SimViewer/src/SimViewer.hpp"

namespace Leph {
namespace SimMecha {

class Joint;

/**
 * Body
 *
 * Base class repressenting
 * rigid Body mechanical object
 */
class Body
{
    public:

        /**
         * Keep track of potential and kinetic energy
         */

    TermPtr _Ep;
    TermPtr _Ec;

        /**
         * Initialization with time
         * Symbol
         */
        Body(SymbolPtr time) :
            _time(time),
            _symPos(),
            _symAngle(),
            _symPosVel(),
            _symAngleVel(),
            _lagrangian(),
            _jointRoot(NULL),
            _joints(),
            _massValues(),
            _massPositions()
        {
            //Init lagrangian to null
            _lagrangian = Symbol::create(
                Symbolic::BaseSymbol::zero());
        }

        /**
         * Virtual destructor
         */
        virtual ~Body()
        {
        }

        /**
         * Return true if the Body is
         * the System base
         */
        inline bool isBase() const
        {
            return (_jointRoot == NULL);
        }

        /**
         * Return the Joint linked to the Base root
         */
        inline Joint* getJointRoot()
        {
            return _jointRoot;
        }

        /**
         * Add a mass with given value and position
         * to the Body coordinates system
         */
        inline void addMass(scalar value, const Vector2D& pos)
        {
            _massValues.push_back(value);
            _massPositions.push_back(pos);
        }

        /**
         * Return the Symbolic position, angle
         * and their velocity of center in
         * Body coordinate system
         */
        inline TermVectorPtr getSymPosition()
        {
            return _symPos;
        }
        inline TermPtr getSymAngle()
        {
            return _symAngle;
        }
        inline TermVectorPtr getSymPositionVel()
        {
            return _symPosVel;
        }
        inline TermPtr getSymAngleVel()
        {
            return _symAngleVel;
        }

        /**
         * Build and return the Symbolic position and
         * velocity expression in global frame of given point
         * in Body coordinate
         */
        inline TermVectorPtr buildSymPosition(const Vector2D& point)
        {
            return Symbolic::Add<Vector2D>::create(
                _symPos,
                Symbolic::Rotation<Vector2D, scalar>::create(
                    ConstantVector::create(point),
                    _symAngle));
        }
        inline TermVectorPtr buildSymPositionVel(const Vector2D& point)
        {
            return buildSymPosition(point)->derivate(_time);
        }

        /**
         * Return the Symbolic lagrangian
         */
        inline TermPtr getLagrangian()
        {
            return _lagrangian;
        }

        inline TermPtr getPotential()
        {
            return _Ep;
        }
        inline TermPtr getKinetic()
        {
            return _Ec;
        }

        /**
         * Set the Joint root
         * (Do not use)
         */
        inline void setJointRoot(Joint* joint)
        {
            _jointRoot = joint;
        }

        /**
         * Add a new Leaf Joint to internal
         * joint container
         * (Do not use)
         */
        inline void addLeafJoint(Joint* joint)
        {
            _joints.push_back(joint);
        }

        /**
         * Set and initialize Symbolic expression
         * Position, angle and their velocity
         * of body center
         * (Do not use)
         */
        inline void initSymbols(
            TermVectorPtr symPos, TermPtr symAngle,
            TermVectorPtr symPosVel, TermPtr symAngleVel)
        {
            //Assign Symbolic position and velocity
            _symPos = symPos;
            _symAngle = symAngle;
            _symPosVel = symPosVel;
            _symAngleVel = symAngleVel;

            _lagrangian = Symbol::create(
                Symbolic::BaseSymbol::zero()); //reset Lagrangian

            //Compute lagrangian Symbolic expression
            computeLagrangian();
        }

        /**
         * Recompute lagrangian expression
         */
        inline void initSymbols()
        {
            computeLagrangian();
        }

        inline void setPos(const Vector2D& pos)
        {
            _symPos =ConstantVector::create(pos);
            computeLagrangian();
        }

        /**
         * Draw the Body on given SimViewer with given
         * center coordinates and angle
         */
        inline virtual void draw(SimViewer::SimViewer& viewer,
            const Vector2D& pos, scalar angle)
        {
            //Draw a scale on center
            viewer.drawFrame(0.3,
                pos.x(), pos.y(), angle*180.0/M_PI);
            //Draw masses
            for (size_t i=0;i<_massPositions.size();i++) {
                Vector2D posMass = pos
                    + Vector2D::rotate(_massPositions[i], angle);
                viewer.drawMass(posMass.x(), posMass.y(),
                    _massValues[i]);
            }
        }

    private:

        /**
         * Time Symbolic variable
         */
        SymbolPtr _time;

        /**
         * Symbolic position, angle, position velocity
         * and angle velocity of center of Body coordinate
         * system in global frame
         */
        TermVectorPtr _symPos;
        TermPtr _symAngle;
        TermVectorPtr _symPosVel;
        TermPtr _symAngleVel;
        /**
         * Body lagrangian Symbolic expression
         */
        TermPtr _lagrangian;



        /**
         * The joint that links the Body
         * to the Base root
         */
        Joint* _jointRoot;

        /**
         * Others joints container
         */
        std::vector<Joint*> _joints;

        /**
         * Masses values and positions containers
         */
        std::vector<scalar> _massValues;
        std::vector<Vector2D> _massPositions;

        /**
         * Compute lagrangian Symbolic expression
         */
        inline void computeLagrangian()
        {

            _Ep=Symbol::create(
                Symbolic::BaseSymbol::zero());
            _Ec=Symbol::create(
                Symbolic::BaseSymbol::zero());


            //Compute lagrangian for all masses
            for (size_t i=0;i<_massPositions.size();i++) {
                //Mass position
                TermVectorPtr posMass =
                    buildSymPosition(_massPositions[i]);
                //Mass velocity
                TermVectorPtr velMass = posMass->derivate(_time);
                //Kinetic energy
                TermPtr halfSym = Constant::create(0.5);
                TermPtr massSym = Constant::create(_massValues[i]);
                TermPtr squaredVel =
                    Symbolic::Dot<scalar, Vector2D, Vector2D>::
                    create(velMass, velMass);
                TermPtr tmp1 =
                    Symbolic::Mult<scalar, scalar, scalar>::
                    create(halfSym, massSym);
                TermPtr kinetic =
                    Symbolic::Mult<scalar, scalar, scalar>::
                    create(tmp1, squaredVel);
                _Ec=Symbolic::Add<scalar>::create(
                    kinetic, _Ec);

                //Potential energy
                TermPtr gravitySym = Constant::create(9.81); //TODO
                TermPtr height =
                    Symbolic::Y<scalar, Vector2D>::
                    create(posMass);
                TermPtr tmp2 =
                    Symbolic::Mult<scalar, scalar, scalar>::
                    create(massSym, gravitySym);
                TermPtr potential =
                    Symbolic::Mult<scalar, scalar, scalar>::
                    create(tmp2, height);
                _Ep=Symbolic::Add<scalar>::create(
                    potential, _Ep);

                //Mass lagrangian
                TermPtr tmp3 = Symbolic::Sub<scalar>::create(
                    kinetic, potential);
                //Increment lagrangian
                _lagrangian = Symbolic::Add<scalar>::create(
                    _lagrangian, tmp3);
            }
        }
};

}
}

#endif
