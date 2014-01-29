#ifndef LEPH_SYMBOLIC_X_HPP
#define LEPH_SYMBOLIC_X_HPP

#include "Symbolic/src/UnaryOperator.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

/**
 * X
 */
template <class T, class U>
class X : public UnaryOperator<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            if (term->toString() == BaseSymbol::zero()) {
                return Symbol<T>::create(BaseSymbol::zero());
            } else {
                return typename Term<T>::TermPtr(
                    new X<T,U>(term));
            }
        }

    protected:
        
        X(const typename Term<U>::TermPtr& term) :
            UnaryOperator<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "X";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return X<T,U>::create(arg);
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("X not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return X<T,U>::create(arg);
        }
};

template <>
inline double X<double,Vector::Vector2D<double> >::functionEvaluation
    (const Vector::Vector2D<double>& argVal) const
{
    return argVal.x();
}

}
}

#endif

