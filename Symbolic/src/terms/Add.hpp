#ifndef LEPH_SYMBOLIC_ADD_HPP
#define LEPH_SYMBOLIC_ADD_HPP

#include "Symbolic/src/BinaryFunction.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Add
 */
template <class T>
class Add : public BinaryFunction<T,T,T>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<T>::TermPtr termLeft, 
            typename Term<T>::TermPtr termRight)
        {
            if (termLeft->toString() == BaseSymbol::zero()) {
                return termRight;
            } else if (termRight->toString() == BaseSymbol::zero()) {
                return termLeft;
            } else {
                return typename Term<T>::TermPtr(
                    new Add<T>(termLeft, termRight));
            }
        }

    protected:

        Add(const typename Term<T>::TermPtr& termLeft, 
            const typename Term<T>::TermPtr& termRight) :
            BinaryFunction<T,T,T>(termLeft, termRight)
        {
        }

        virtual inline std::string computeString()
        {
            return "(" + BinaryFunction<T,T,T>::_argLeft->toString()
                + ")+(" + BinaryFunction<T,T,T>::_argRight->toString() + ")";
        }
        
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            typename Term<T>::TermPtr argLeft = 
                BinaryFunction<T,T,T>::_argLeft;
            typename Term<T>::TermPtr argRight = 
                BinaryFunction<T,T,T>::_argRight;
            typename Term<T>::TermPtr termLeft = 
                argLeft->derivate(sym);
            typename Term<T>::TermPtr termRight = 
                argRight->derivate(sym);

            return Add<T>::create(termLeft, termRight);
        }
        
        virtual T computeEvaluation(const Bounder& bounder)
        {
            T left = BinaryFunction<T,T,T>::_argLeft->evaluate(bounder);
            T right = BinaryFunction<T,T,T>::_argRight->evaluate(bounder);
            return left + right;
        }
};

}
}

#endif

