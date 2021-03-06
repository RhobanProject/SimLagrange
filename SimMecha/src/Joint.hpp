#ifndef LEPH_SIMMECHA_JOINT_HPP
#define LEPH_SIMMECHA_JOINT_HPP

#include <string>
#include "SimMecha/src/Simulation.h"
#include "SimMecha/src/Body.hpp"

namespace Leph {
namespace SimMecha {

/**
 * Joint
 *
 * Base class repressenting
 * a link between two rigid bodies
 */
class Joint
{
    public:

    /**
     * very dirty hack thanks to c++
     *
     */

/*
    TermPtr _initposX;
    TermPtr _initposY;
*/


        /**
         * Keep track of potential and kinetic energy
         */

    TermPtr _Ep;
    TermPtr _Ec;

        /**
         * Default constructor
         * (Do not use)
         */
        Joint() :
            _bodyRoot(NULL),
            _bodyLeaf(NULL),
            _posRoot(),
            _posLeaf(),
            _angleRoot(),
            _angleLeaf()
        {
        }

        /**
         * Initialization with Joint structure and
         * Symbolic degree of freedom
         */
        Joint(
            Body& bodyRoot, const Vector2D& posRoot, scalar angleRoot,
            Body& bodyLeaf, const Vector2D& posLeaf, scalar angleLeaf,
            SymbolPtr dof) :
            _bodyRoot(&bodyRoot),
            _bodyLeaf(&bodyLeaf),
            _posRoot(posRoot),
            _posLeaf(posLeaf),
            _angleRoot(angleRoot),
            _angleLeaf(angleLeaf),
            _dof(dof)
        {
        }

        /**
         * Virtual Destructor
         */
        virtual ~Joint()
        {
        }

        /**
         * Getters
         */
        inline const Body& getBodyRoot() const
        {
            return *_bodyRoot;
        }
        inline Body& getBodyRoot()
        {
            return *_bodyRoot;
        }
        inline const Body& getBodyLeaf() const
        {
            return *_bodyLeaf;
        }
        inline Body& getBodyLeaf()
        {
            return *_bodyLeaf;
        }
        inline const Vector2D& getPosRoot() const
        {
            return _posRoot;
        }
        inline const Vector2D& getPosLeaf() const
        {
            return _posLeaf;
        }

        inline Vector2D& getPosRoot()
        {
            return _posRoot;
        }
        inline Vector2D& getPosLeaf()
        {
            return _posLeaf;
        }

        inline scalar getAngleRoot() const
        {
            return _angleRoot;
        }
        inline scalar getAngleLeaf() const
        {
            return _angleLeaf;
        }

        /**
         * Return the internal degree of freeodm
         */
        inline SymbolPtr getDof()
        {
            return _dof;
        }

        /**
         * Return Joint name
         */
        inline const std::string& getName()
        {
            return _dof->toString();
        }

        /**
         * Build Symbolic transformation (position and angle)
         * of Body center from root Body to leaf Body
         * and then update Symbolic leaf Body
         * internal expressions
         */
        virtual void computeSymTransformation(SymbolPtr time) = 0;




        /**
         * Recompute lagrangian expression
         */
        inline void initSymbols()
        {
            computeLagrangian();
        }

        /**
         * Default virtual Lagrangian.
         * To be redefined
         */
        virtual void computeLagrangian()
        {
            _lagrangian = Constant::create(0.0);
            _Ep= Constant::create(0.0);
            _Ec= Constant::create(0.0);

        }

        /**
         * Return the Symbolic lagrangian
         */
        inline TermPtr getLagrangian()
        {
            return _lagrangian;
        }

        inline void setLagrangian(TermPtr L)
        {
            _lagrangian=L;
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
* stupid hack
*/

   inline virtual void setCustomScalar(scalar val)
        {
            std::cout<<"Bah?"<<std::endl;
        }

        /**
         * Draw the Joint on given SimViewer with given
         * center and angle coordinates for both linked
         * Bodies and dof value
         */
        virtual void draw(SimViewer::SimViewer& viewer,
            const Vector2D& posRoot, scalar angleRoot,
            const Vector2D& posLeaf, scalar angleLeaf,
            scalar value)
        {
            (void)angleRoot;
            (void)angleLeaf;
            (void)value;

            viewer.drawSegmentByEnd(posRoot.x(), posRoot.y(),
                posLeaf.x(), posLeaf.y());
        }

    private:

        /**
         * The linked Body
         * connected to the Base root
         */
        Body* _bodyRoot;

        /**
         * The linked Body opposed
         * to the Base root
         */
        Body* _bodyLeaf;

        /**
         * The position in Bodies coordinate system
         * of Joint position
         */
        Vector2D _posRoot;
        Vector2D _posLeaf;

        /**
         * The orientation in Bodies coordinate system
         * of Point axis angle
         */
        scalar _angleRoot;
        scalar _angleLeaf;

        /**
         * Joint Symbolic degree of freedom
         */
        SymbolPtr _dof;

        /**
         * Joint lagrangian Symbolic expression
         */
        TermPtr _lagrangian;

};

}
}

#endif
