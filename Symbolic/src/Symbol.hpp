#ifndef LEPH_SYMBOLIC_SYMBOL_HPP
#define LEPH_SYMBOLIC_SYMBOL_HPP

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
            return Symbol<T>::create(BaseSymbol::derivateString(sym));
        }
       
        /**
         * @Inherit
         */
        virtual T computeEvaluation(const Bounder& bounder)
        {
            return bounder.getValue<T>(*this);
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

};

}
}

#endif

