#ifndef LEPH_SYMBOLIC_BOUNDER_HPP
#define LEPH_SYMBOLIC_BOUNDER_HPP

#include <map>
#include "Any/src/Any.hpp"
#include "Symbolic/src/BaseSymbol.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Bounder
 *
 * This class is use to assign a value
 * to typed Symbol for evaluation
 */
class Bounder
{
    public:

        /**
         * Initialization empty Bounder
         */
        Bounder() :
            _symbolValues()
        {
        }

        /**
         * Test if a given Symbol is registered
         * into the Bounder
         */
        inline bool isSymbol(const BaseSymbol::BaseSymbolPtr& sym) const
        {
            return _symbolValues.count(sym->getName());
        }
        inline bool isSymbol(const BaseSymbol& sym) const
        {
            return _symbolValues.count(sym.getName());
        }

        /**
         * Register the given Symbol
         */
        template <class T>
        inline void setSymbol(const BaseSymbol::BaseSymbolPtr& sym)
        {
            //Insert value zero
            T zero = T();
            _symbolValues[sym->getName()] = Any::Any(zero);
        }

        /**
         * Get a value for the given Symbol
         */
        template <class T>
        inline const T& getValue(const BaseSymbol::BaseSymbolPtr& sym) const
        {
            return _symbolValues.at(sym->getName()).get<T>();
        }
        template <class T>
        inline const T& getValue(const BaseSymbol& sym) const
        {
            return _symbolValues.at(sym.getName()).get<T>();
        }

        /**
         * Register a value for the given Symbol
         */
        template <class T>
        inline void setValue(const BaseSymbol::BaseSymbolPtr& sym, 
            const T& val)
        {
            _symbolValues[sym->getName()] = Any::Any(val);
        }

        /**
         * Merge into one two Bounder object
         */
        static inline Bounder merge(const Bounder& b1, const Bounder& b2)
        {
            Bounder merged;
            std::map<std::string,Any::Any>::const_iterator it;

            //Add all registered Symbol from b1
            for (it=b1._symbolValues.begin();it!=b1._symbolValues.end();it++) {
                merged._symbolValues[it->first] = it->second;
            }
            //And from b2
            for (it=b2._symbolValues.begin();it!=b2._symbolValues.end();it++) {
                merged._symbolValues[it->first] = it->second;
            }
            
            return merged;
        }

    private:

        /**
         * Symbol value container indexed by 
         * Symbol string name
         */
        std::map<std::string,Any::Any> _symbolValues;
};

}
}

#endif

