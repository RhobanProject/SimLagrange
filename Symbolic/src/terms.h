#ifndef LEPH_SYMBOLIC_TERMS_H
#define LEPH_SYMBOLIC_TERMS_H

#include "Symbolic/src/terms/Add.hpp"
#include "Symbolic/src/terms/Dot.hpp"
#include "Symbolic/src/terms/Exp.hpp"
#include "Symbolic/src/terms/Frac.hpp"
#include "Symbolic/src/terms/Y.hpp"
#include "Symbolic/src/terms/Minus.hpp"
#include "Symbolic/src/terms/Mult.hpp"
#include "Symbolic/src/terms/Polar.hpp"
#include "Symbolic/src/terms/PolarInv.hpp"
#include "Symbolic/src/terms/Pow.hpp"
#include "Symbolic/src/terms/X.hpp"
#include "Symbolic/src/terms/Sub.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Rotation
 */
template <class T, class U>
class Rotation 
{
    public:

        inline static typename Term<T>::TermPtr create(
            typename Term<T>::TermPtr termLeft, 
            typename Term<U>::TermPtr termRight)
        {
            if (termLeft->toString() == BaseSymbol::zero()) {
                return termLeft;
            } else if (termRight->toString() == BaseSymbol::zero()) {
                return termLeft;
            } else {
                return Add<T>::create(
                    Mult<T,U,T>::create(
                        X<U,T>::create(termLeft), 
                        Polar<T,U>::create(termRight)),
                    Mult<T,U,T>::create(
                        Y<U,T>::create(termLeft), 
                        PolarInv<T,U>::create(Minus<U>::create(termRight)))
                );
            }
        }
};

}
}

#endif

