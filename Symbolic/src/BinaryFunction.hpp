#ifndef LEPH_SYMBOLIC_BINARYFUNCTION_HPP
#define LEPH_SYMBOLIC_BINARYFUNCTION_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"

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
            throw std::logic_error("BinaryFunction not implemented");
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            throw std::logic_error("BinaryFunction not implemented");
        }

        /**
         * @Inherit
         */
        virtual void doReset() 
        {
            _argLeft->reset();
            _argRight->reset();
        }
};

}
}

#endif

