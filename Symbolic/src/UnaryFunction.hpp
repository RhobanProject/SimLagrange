#ifndef LEPH_SYMBOLIC_UNARYFUNCTION_HPP
#define LEPH_SYMBOLIC_UNARYFUNCTION_HPP

#include "Symbolic/src/UnaryBase.hpp"
#include "Symbolic/src/terms/Mult.hpp"

namespace Leph {
namespace Symbolic {

/**
 * UnaryFunction
 *
 * Represent an Unary Function
 * from type U to type T
 */
template <class T, class U>
class UnaryFunction : public UnaryBase<T,U>
{
    protected:
        
        /**
         * Initialization with the function 
         * term argument of type U
         */
        UnaryFunction(const typename Term<U>::TermPtr& term) :
            UnaryBase<T,U>(term)
        {
        }

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return Mult<T,U,T>::create(
                UnaryBase<T,U>::_arg->derivate(sym), 
                this->functionDerivative(UnaryBase<T,U>::_arg));
        }
};

}
}

#endif

