#ifndef LEPH_SYMBOLIC_MINUS_HPP
#define LEPH_SYMBOLIC_MINUS_HPP

#include "Symbolic/src/UnaryFunction.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Minus
 */
template <class T>
class Minus : public UnaryFunction<T,T>
{
    public:

        static inline typename Term<T>::TermPtr create(
            const typename Term<T>::TermPtr& term)
        {
            return typename Term<T>::TermPtr(
                new Minus<T>(term));
        }
    protected:
        
        Minus(const typename Term<T>::TermPtr& term) :
            UnaryFunction<T,T>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "-";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<T>::TermPtr& arg) const
        {
            return Minus<T>::create(arg);
        }

        virtual inline T functionEvaluation(const T& argVal) const
        {
            return -argVal;
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<T>::TermPtr& arg) const
        {
            return Minus::create(arg);
        }
};

}
}

#endif

