#ifndef LEPH_SYMBOLIC_EXP_HPP
#define LEPH_SYMBOLIC_EXP_HPP

#include <cmath>
#include "Symbolic/src/UnaryFunction.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Exp
 */
template <class T, class U>
class Exp : public UnaryFunction<T,U>
{
    public:
        
        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            return UnaryBase<T,U>::checkCst(
                new Exp<T,U>(term));
        }

    protected:

        Exp(const typename Term<U>::TermPtr& term) :
            UnaryFunction<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "exp";
        }

        virtual inline typename Term<T>::TermPtr functionDerivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return Exp<T,U>::create(arg);
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("Exp not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return Exp<T,U>::create(arg);
        }
};

template <>
inline double Exp<double,double>::functionEvaluation
    (const double& argVal) const
{
    return exp(argVal);
}

}
}

#endif

