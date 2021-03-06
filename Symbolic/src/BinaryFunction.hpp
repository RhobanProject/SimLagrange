#ifndef LEPH_SYMBOLIC_BINARYFUNCTION_HPP
#define LEPH_SYMBOLIC_BINARYFUNCTION_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"
#include "Constant.hpp"

namespace Leph {
namespace Symbolic {

/**
 * BinaryFunction
 *
 * Represent a Binary Function from
 * type U,V to type T
 */
template <class T, class U, class V>
class BinaryFunction : public Term<T>
{
    protected:

        /**
         * The Terms as function arguments left 
         * and right of type U and type V
         */
        typename Term<U>::TermPtr _argLeft;
        typename Term<V>::TermPtr _argRight;

        /**
         * Initialization with the function 
         * term argument of type U and type V
         */
        BinaryFunction(const typename Term<U>::TermPtr& termLeft, 
            const typename Term<V>::TermPtr& termRight) :
            Term<T>(),
            _argLeft(termLeft),
            _argRight(termRight)
        {
        }

        /**
         * If the arguments of given binary are constant, 
         * a constant value is evaluated and return
         * Else the original is return
         */
        static inline typename Term<T>::TermPtr checkCst
            (BinaryFunction<T,U,V>* binary)
        {
            if (
                binary->_argLeft->isConstant() && 
                binary->_argRight->isConstant()
            ) {
                return Constant<T>::create(binary->evaluate(Bounder()));
            } else {
                return binary;
            }
        }

        /**
         * @Inherit
         */
        virtual inline std::string computeString()
        {
            throw std::logic_error("BinaryFunction not implemented");
        }
        
        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            (void)sym;
            throw std::logic_error("BinaryFunction not implemented");
        }
       
        /**
         * @Inherit
         */
        virtual inline T computeEvaluation(const Bounder& bounder)
        {
            (void)bounder;
            throw std::logic_error("BinaryFunction not implemented");
        }

        /**
         * @Inherit
         */
        virtual inline void doReset() 
        {
            _argLeft->reset();
            _argRight->reset();
        }
};

}
}

#endif

