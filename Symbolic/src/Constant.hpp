#ifndef LEPH_SYMBOLIC_CONSTANT_HPP
#define LEPH_SYMBOLIC_CONSTANT_HPP

#include <sstream>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/BaseSymbol.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Constant
 *
 * Represent a numeric constant
 * of type T
 */
template <class T>
class Constant : public Term<T>
{
    public:

        /**
         * Create a constant Object
         */
        static inline typename Term<T>::TermPtr create(const T& value)
        {
            return typename Term<T>::TermPtr(new Constant<T>(value));
        }

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeSubstitution
            (const BaseSymbol::BaseSymbolPtr& sym, const Any::Any& term)
        {
            return Constant<T>::create(_value);
        }

    protected:

        /**
         * @Inherit
         */
        virtual inline std::string computeString()
        {
            std::ostringstream oss;
            oss << _value;
            return oss.str();
        }
        
        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return Symbol<T>::create(BaseSymbol::zero());
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            return _value;
        }

        /**
         * @Inherit
         */
        virtual void doReset()
        {
            //Nothing to do
        }

    private:

        /**
         * Initialization
         */
        Constant(const T& value) :
            _value(value)
        {
        }

        /**
         * The constant value
         */
        T _value;
};

}
}

#endif

