#ifndef LEPH_SYMBOLIC_UNARYFUNCTION_HPP
#define LEPH_SYMBOLIC_UNARYFUNCTION_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"

namespace Leph {
namespace Symbolic {

/**
 * UnaryFunction
 *
 * Represent an Unary Function from
 * type U to type T
 */
template <class T, class U>
class UnaryFunction : public Term<T>
{
    protected:

        /**
         * The Term as function argument of type U
         */
        typename Term<U>::TermPtr _arg;

        /**
         * Initialization with the function 
         * term argument of type U
         */
        UnaryFunction(const typename Term<U>::TermPtr& term) :
            Term<T>(),
            _arg(term)
        {
        }

        /**
         * @Inherit
         */
        virtual inline std::string computeString()
        {
            throw std::logic_error("UnaryFunction not implemented");
        }
        
        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            throw std::logic_error("UnaryFunction not implemented");
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            throw std::logic_error("UnaryFunction not implemented");
        }

        /**
         * @Inherit
         */
        virtual void reset() 
        {
            _arg->reset();
        }
};

}
}

#endif

