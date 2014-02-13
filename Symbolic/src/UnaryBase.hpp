#ifndef LEPH_SYMBOLIC_UNARYBASE_HPP
#define LEPH_SYMBOLIC_UNARYBASE_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"

namespace Leph {
namespace Symbolic {

/**
 * UnaryBase
 *
 * Represent an Unary Function or Operator
 * from type U to type T
 */
template <class T, class U>
class UnaryBase : public Term<T>
{
    public:

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeSubstitution
            (const BaseSymbol::BaseSymbolPtr& sym, const Any::Any& term)
        {
            return functionCreate(_arg->computeSubstitution(sym, term));
        }
       
    protected:

        /**
         * The Term as function argument of type U
         */
        typename Term<U>::TermPtr _arg;

        /**
         * Initialization with the function 
         * term argument of type U
         */
        UnaryBase(const typename Term<U>::TermPtr& term) :
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
        virtual typename Term<T>::TermPtr functionDerivative
            (const typename Term<U>::TermPtr& arg) const = 0;
        virtual T functionEvaluation(const U& argVal) const = 0;

        /**
         * Return a newly allocated instance of derived class 
         * with given argument
         */
        virtual typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const = 0;
};

}
}

#endif

