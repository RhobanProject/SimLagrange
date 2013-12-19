#ifndef LEPH_SYMBOLIC_POLAR_HPP
#define LEPH_SYMBOLIC_POLAR_HPP

#include <cmath>
#include "Symbolic/src/UnaryFunction.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

template <class t,class U>
class PolarInv;

/**
 * Polar
 */
template <class T, class U>
class Polar : public UnaryFunction<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            return typename Term<T>::TermPtr(
                new Polar<T,U>(term));
        }

    protected:
        
        Polar(const typename Term<U>::TermPtr& term) :
            UnaryFunction<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "Polar";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return PolarInv<T,U>::create(Minus<U>::create(arg));
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("Polar not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return Polar<T,U>::create(arg);
        }
};

template <>
inline Vector::Vector2D<double> Polar<Vector::Vector2D<double>, double>::
    functionEvaluation(const double& argVal) const
{
    return Vector::Vector2D<double>(cos(argVal), sin(argVal));
}

}
}

#endif

