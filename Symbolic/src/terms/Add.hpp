#ifndef LEPH_SYMBOLIC_ADD_HPP
#define LEPH_SYMBOLIC_ADD_HPP

#include "Symbolic/src/BinaryFunction.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Add
 */
template <class T, class U, class V>
class Add : public BinaryFunction<T,U,V>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr termLeft, 
            typename Term<V>::TermPtr termRight)
        {
            if (termLeft->toString() == BaseSymbol::zero()) {
                return termRight;
            } else if (termLeft->toString() == BaseSymbol::zero()) {
                return termLeft;
            } else {
                return typename Term<T>::TermPtr(
                    new Add<T,U,V>(termLeft, termRight));
            }
        }

    protected:

        Add(const typename Term<U>::TermPtr& termLeft, 
            const typename Term<V>::TermPtr& termRight) :
            BinaryFunction<T,U,V>(termLeft, termRight)
        {
        }

        virtual inline std::string computeString()
        {
            return "(" + BinaryFunction<T,U,V>::_argLeft->toString()
                + ")+(" + BinaryFunction<T,U,V>::_argRight->toString() + ")";
        }
        
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            typename Term<U>::TermPtr argLeft = 
                BinaryFunction<T,U,V>::_argLeft;
            typename Term<V>::TermPtr argRight = 
                BinaryFunction<T,U,V>::_argRight;
            typename Term<U>::TermPtr termLeft = 
                argLeft->derivate(sym);
            typename Term<V>::TermPtr termRight = 
                argRight->derivate(sym);

            return Add<T,U,V>::create(termLeft, termRight);
        }
        
        virtual T computeEvaluation(const Bounder& bounder)
        {
            U left = BinaryFunction<T,U,V>::_argLeft->evaluate(bounder);
            V right = BinaryFunction<T,U,V>::_argRight->evaluate(bounder);
            return left + right;
        }
};

}
}

#endif

