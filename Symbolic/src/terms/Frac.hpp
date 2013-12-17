#ifndef LEPH_SYMBOLIC_FRAC_HPP
#define LEPH_SYMBOLIC_FRAC_HPP

#include "Symbolic/src/BinaryFunction.hpp"
#include "Symbolic/src/terms/Sub.hpp"
#include "Symbolic/src/terms/Mult.hpp"
#include "Symbolic/src/terms/Pow.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Frac
 */
template <class T>
class Frac : public BinaryFunction<T,T,T>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<T>::TermPtr termLeft, 
            typename Term<T>::TermPtr termRight)
        {
            if (termLeft->toString() == BaseSymbol::zero()) {
                return termLeft;
            } else if (termRight->toString() == BaseSymbol::zero()) {
                throw std::logic_error("Frac division by zero");
            } else if (termRight->toString() == BaseSymbol::one()) {
                return termLeft;
            } else {
                return typename Term<T>::TermPtr(
                    new Frac<T>(termLeft, termRight));
            }
        }

        virtual inline typename Term<T>::TermPtr computeSubstitution
            (const BaseSymbol::BaseSymbolPtr& sym, const Any::Any& term)
        {
            typename Term<T>::TermPtr argLeft = 
                BinaryFunction<T,T,T>::_argLeft;
            typename Term<T>::TermPtr argRight = 
                BinaryFunction<T,T,T>::_argRight;
            typename Term<T>::TermPtr termLeft = 
                argLeft->computeSubstitution(sym, term);
            typename Term<T>::TermPtr termRight = 
                argRight->computeSubstitution(sym, term);

            return Frac<T>::create(termLeft, termRight);
        }

    protected:
        
        Frac(const typename Term<T>::TermPtr& termLeft, 
            const typename Term<T>::TermPtr& termRight) :
            BinaryFunction<T,T,T>(termLeft, termRight)
        {
        }

        virtual inline std::string computeString()
        {
            return "(" + BinaryFunction<T,T,T>::_argLeft->toString()
                + ")/(" + BinaryFunction<T,T,T>::_argRight->toString() + ")";
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

            return Frac<T>::create(
                Sub<T>::create(
                    Mult<T,T,T>::create(termLeft, argRight), 
                    Mult<T,T,T>::create(argLeft, termRight)),
                Pow<T>::create(argRight, 2));
        }
        
        virtual T computeEvaluation(const Bounder& bounder)
        {
            T left = BinaryFunction<T,T,T>::_argLeft->evaluate(bounder);
            T right = BinaryFunction<T,T,T>::_argRight->evaluate(bounder);
            return left / right;
        }
};

}
}

#endif

