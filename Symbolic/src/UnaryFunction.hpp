#ifndef LEPH_SYMBOLIC_UNARYFUNCTION_HPP
#define LEPH_SYMBOLIC_UNARYFUNCTION_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/terms/Mult.hpp"

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
            return functionString() + "(" + _arg->toString() + ")";
        }
        
        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return Mult<T,T,T>::create(
                _arg->derivate(sym), 
                functionderivative(_arg));
        }
       
        /**
         * @Inherit
         */
        virtual inline T computeEvaluation(const Bounder& bounder)
        {
            return functionEvaluation(_arg->evaluate(bounder));
        }

        /**
         * @Inherit
         */
        virtual inline void doReset()
        {
            _arg->reset();
        }

        /**
         * Return the function string name, its derivative with 
         * respect to its argument arg and the function evaluation
         * given the argument value argVal
         */
        virtual std::string functionString() const = 0;
        virtual typename Term<T>::TermPtr functionderivative
            (const typename Term<U>::TermPtr arg) const = 0;
        virtual T functionEvaluation(const U& argVal) const = 0;
};

}
}

#endif

