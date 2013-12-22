#ifndef LEPH_SYMBOLIC_TERM_HPP
#define LEPH_SYMBOLIC_TERM_HPP

#include <map>
#include <ostream>
#include "Any/src/Any.hpp"
#include "SmartPointer/src/SmartPtr.hpp"
#include "Symbolic/src/BaseSymbol.hpp"
#include "Symbolic/src/Bounder.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Term
 *
 * Base class for Symbolic expression
 * of type T
 */
template <class T>
class Term
{
    public:

        /**
         * SmartPtr alias
         */
        typedef SmartPointer::SmartPtr<Term<T> > TermPtr;

        /**
         * Destructor
         * (need to be virtual)
         */
        virtual ~Term()
        {
        }

        /**
         * Return the string representation of the Term
         */
        inline const std::string& toString()
        {
            if (_string.empty()) {
                _string = computeString();
            }

            return _string;
        }

        /**
         * Return the derivated Term with respect to the given
         * Symbol
         */
        inline TermPtr derivate(const BaseSymbol::BaseSymbolPtr& sym)
        {
            if (_derivatives.count(sym->getName()) == 0) {
                _derivatives[sym->getName()] = computeDerivative(sym);
            }

            return _derivatives.at(sym->getName());
        }

        /**
         * Return the Term with all Symbol sym substituted by
         * the expression Term of return type U
         * (The Symbol sym must be of type U)
         */
        template <class U>
        inline TermPtr substitute(const BaseSymbol::BaseSymbolPtr& sym, 
            const typename Term<U>::TermPtr& term)
        {
            //Call substitution implementation holding term 
            //in a Any container (template virtual methods not allowed)
            return computeSubstitution(sym, Any::Any(term));
        }
        
        /**
         * Compute and return the Term where all Symbol sym
         * are substituated by the Term term
         * Any hold a TermPtr with same type of sym 
         * (template virtual methods not allowed)
         * (This method should not be used by user)
         */
        virtual TermPtr computeSubstitution
            (const BaseSymbol::BaseSymbolPtr& sym, const Any::Any& term) = 0;

        /**
         * Return the typed value of Term evaluation with respect
         * to Bounder Symbol values
         */
        inline const T& evaluate(const Bounder& bounder)
        {
            if (!_isValueComputed) {
                _value = computeEvaluation(bounder);
                _isValueComputed = true;
            }

            return _value;
        }

        /**
         * Clear all computed string, derivative and values
         */
        inline void reset()
        {
            _string = std::string();
            _derivatives.clear();
            _isValueComputed = false;
            doReset();
        }

    protected:
        
        /**
         * Initialization
         */
        Term() :
            _string(),
            _derivatives(),
            _value(),
            _isValueComputed(false)
        {
        }

        /**
         * Compute and return the string representation
         */
        virtual std::string computeString() = 0;

        /**
         * Compute and return the derivative 
         * with respect to sym
         */
        virtual TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym) = 0;

        /**
         * Compute and return the typed value 
         * of the Term
         */
        virtual T computeEvaluation(const Bounder& bounder) = 0;

        /**
         * Reset recursively all computed values
         */
        virtual void doReset() = 0;

    private:

        /**
         * The string representation of the Term
         */
        std::string _string;

        /**
         * The container of computed derivatives
         * indexed by derivation Symbol string
         */
        std::map<std::string,TermPtr> _derivatives;

        /**
         * The value of the term and a flag indicating
         * if it has been computed
         */
        T _value;
        bool _isValueComputed;
};

/**
 * Print operator
 */
template <class T>
std::ostream& operator<<(std::ostream& os, Term<T>& term)
{
    os << term.toString();
    return os;
}

}
}

#endif

