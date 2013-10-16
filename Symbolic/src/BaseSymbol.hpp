#ifndef LEPH_SYMBOLIC_BASESYMBOL_HPP
#define LEPH_SYMBOLIC_BASESYMBOL_HPP

#include <set>
#include <stdexcept>
#include "SmartPointer/src/SmartPtr.hpp"

namespace Leph {
namespace Symbolic {

/**
 * BaseSymbol
 *
 * Base class for derivable 
 * Symbol term
 */
class BaseSymbol
{
    public:

        /**
         * SmartPtr alias
         */
        typedef SmartPointer::SmartPtr<BaseSymbol> BaseSymbolPtr;

        /**
         * Initialization with the Symbol 
         * string name
         */
        BaseSymbol(const std::string& name) :
            _name(name),
            _depends()
        {
            if (_name.empty()) {
                throw std::logic_error("BaseSymbol empty name");
            }
        }

        /**
         * Copy with new name
         * (Copy the depends)
         */
        BaseSymbol(const std::string& name, const BaseSymbol& copy) :
            _name(name),
            _depends(copy._depends)
        {
            if (_name.empty()) {
                throw std::logic_error("BaseSymbol empty name");
            }
        }

        /**
         * Return the Symbol name
         */
        inline const std::string& getName() const
        {
            return _name;
        }

        /**
         * Register the given Symbol as a dependence
         */
        inline void depend(const BaseSymbolPtr& sym)
        {
            _depends.insert(sym->getName());
        }

        /**
         * Return the string representing the derivate
         * of the Symbol with respect to the given Symbol
         */
        inline std::string derivateString(const BaseSymbolPtr& sym) const
        {
            if (sym->getName() == zero() || sym->getName() == one()) {
                throw std::logic_error("BaseSymbol invalid derivation");
            }

            if (_name == zero() || _name == one()) {
                return zero();
            }

            if (sym->getName() == _name) {
                return one();
            } else if (_depends.count(sym->getName()) > 0) {
                return "d("+_name+")/d"+sym->getName();
            } else {
                return zero();
            }
        }

        /**
         * Special Symbolic constant
         */
        static inline std::string zero()
        {
            return "ZERO";
        }
        static inline std::string one()
        {
            return "ONE";
        }

    private:

        /**
         * The symbol name
         */
        std::string _name;

        /**
         * The container of other related Symbols
         * indexed by their name
         */
        std::set<std::string> _depends;
};

}
}

#endif

