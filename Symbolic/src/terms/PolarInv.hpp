#ifndef LEPH_SYMBOLIC_POLARINV_HPP
#define LEPH_SYMBOLIC_POLARINV_HPP

#include <cmath>
#include "Symbolic/src/UnaryFunction.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

template <class T,class U>
class Polar;

/**
 * PolarInv
 */
template <class T, class U>
class PolarInv : public UnaryFunction<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            return UnaryBase<T,U>::checkCst(
                new PolarInv<T,U>(term));
        }

    protected:
        
        PolarInv(const typename Term<U>::TermPtr& term) :
            UnaryFunction<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "PolarInv";
        }

        virtual inline typename Term<T>::TermPtr functionDerivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return Polar<T,U>::create(Minus<U>::create(arg));
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("PolarInv not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return PolarInv<T,U>::create(arg);
        }
};

template <>
inline Vector::Vector2D<double> PolarInv<Vector::Vector2D<double>, double>::
    functionEvaluation(const double& argVal) const
{
    return Vector::Vector2D<double>(sin(argVal), cos(argVal));
}

template <>
inline double PolarInv<double, double>::
    functionEvaluation(const double& argVal) const
{
    return sin(argVal);
}

}
}

#endif

