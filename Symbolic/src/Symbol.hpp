#ifndef LEPH_SYMBOLIC_SYMBOL_HPP
#define LEPH_SYMBOLIC_SYMBOL_HPP

#include <stdexcept>
#include "Symbolic/src/Term.hpp"
#include "Symbolic/src/BaseSymbol.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Symbol
 *
 * Represent a Symbolic Term of type T
 * (A leaf in syntaxic expression tree)
 */
template <class T>
class Symbol : public BaseSymbol, public Term<T>
{
    public:

        /**
         * SmartPtr alias
         */
        typedef SmartPointer::SmartPtr<Symbol<T> > SymbolPtr;

        /**
         * Create Symbol Object
         */
        static inline SymbolPtr create(const std::string& name)
        {
            return SymbolPtr(new Symbol<T>(name));
        }
        
    protected:

        /**
         * @Inherit
         */
        virtual inline std::string computeString()
        {
            return BaseSymbol::getName();
        }
        
        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return SymbolPtr(new Symbol<T>(
                BaseSymbol::derivateString(sym),
                *this));
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            if (BaseSymbol::getName() == BaseSymbol::zero()) {
                return T();
            } else if (BaseSymbol::getName() == BaseSymbol::one()) {
                return T(1);
            } else {
                if (bounder.isSymbol(*this)) {
                    return bounder.getValue<T>(*this);
                } else {
                    throw std::logic_error(
                        "Symbol unbound: " + BaseSymbol::getName());
                }
            }
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
        Symbol(const std::string& name) :
            BaseSymbol(name),
            Term<T>()
        {
        }

        /**
         * Copy with new name
         */
        Symbol(const std::string& name, const Symbol<T>& copy) :
            BaseSymbol(name, copy),
            Term<T>()
        {
        }
};

}
}

#endif

