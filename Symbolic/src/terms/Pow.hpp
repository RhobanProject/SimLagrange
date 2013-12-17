#ifndef LEPH_SYMBOLIC_POW_HPP
#define LEPH_SYMBOLIC_POW_HPP

#include <cmath>
#include <sstream>
#include "Symbolic/src/UnaryFunction.hpp"
#include "Symbolic/src/Symbol.hpp"
#include "Symbolic/src/terms/Mult.hpp"
#include "Symbolic/src/Constant.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Pow
 */
template <class T>
class Pow : public UnaryFunction<T,T>
{
    public:
        
        static inline typename Term<T>::TermPtr create(
            const typename Term<T>::TermPtr& term, long power)
        {
            if (power == 0) {
               return Symbol<T>::create(BaseSymbol::one());
            } else if (power == 1) {
                return term;
            } else {
                return typename Term<T>::TermPtr(
                    new Pow<T>(term, power));
            }
        }

    protected:

        /**
         * Power order
         */
        long _power;

        Pow(const typename Term<T>::TermPtr& term,
            long power) :
            UnaryFunction<T,T>(term),
            _power(power)
        {
        }
        
        /**
         * @Inherit
         */
        virtual inline std::string computeString()
        {
            std::ostringstream oss;
            oss << _power;
            return "(" + UnaryFunction<T,T>::_arg->toString() 
                + ")^" + oss.str();
        }
        
        virtual inline std::string functionString() const
        {
            return "pow";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<T>::TermPtr& arg) const
        {
            return Mult<T,long,T>::create(
                Constant<long>::create(_power),
                Pow<T>::create(arg, _power-1));
        }

        virtual inline T functionEvaluation(const T& argVal) const
        {
            throw std::logic_error("Pow not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<T>::TermPtr& arg) const
        {
            return Pow<T>::create(arg, _power);
        }
};

template <>
inline double Pow<double>::functionEvaluation
    (const double& argVal) const
{
    return pow(argVal, _power);
}

}
}

#endif

