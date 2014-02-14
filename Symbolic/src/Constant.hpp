#ifndef LEPH_SYMBOLIC_CONSTANT_HPP
#define LEPH_SYMBOLIC_CONSTANT_HPP

#include <sstream>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/BaseSymbol.hpp"
#include "Symbolic/src/Symbol.hpp"

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
            if (value == T()) {
                return typename Term<T>::TermPtr(
                    Symbol<T>::create(BaseSymbol::zero()));
            } else {
                return typename Term<T>::TermPtr(
                    new Constant<T>(value));
            }
        }

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeSubstitution
            (const BaseSymbol::BaseSymbolPtr& sym, const Any::Any& term)
        {
            (void)sym;
            (void)term;
            return Constant<T>::create(_value);
        }

        /**
         * @Inherit
         */
        virtual inline bool isConstant() const
        {
            return true;
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
            (void)sym;
            return Symbol<T>::create(BaseSymbol::zero());
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            (void)bounder;
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

