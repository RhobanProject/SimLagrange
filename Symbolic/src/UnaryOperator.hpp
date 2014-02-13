#ifndef LEPH_SYMBOLIC_UNARYOPERATOR_HPP
#define LEPH_SYMBOLIC_UNARYOPERATOR_HPP

#include "Symbolic/src/UnaryBase.hpp"

namespace Leph {
namespace Symbolic {

/**
 * UnaryOperator
 *
 * Represent an Unary Operator
 * from type U to type T
 */
template <class T, class U>
class UnaryOperator : public UnaryBase<T,U>
{
    protected:
        
        /**
         * Initialization with the function 
         * term argument of type U
         */
        UnaryOperator(const typename Term<U>::TermPtr& term) :
            UnaryBase<T,U>(term)
        {
        }

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return this->functionDerivative(
                UnaryBase<T,U>::_arg->derivate(sym));
        }
};

}
}

#endif

