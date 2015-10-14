#ifndef LEPH_SYMBOLIC_Y_HPP
#define LEPH_SYMBOLIC_Y_HPP

#include "Symbolic/src/UnaryOperator.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Y
 */
template <class T, class U>
class Y : public UnaryOperator<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            if (term->toString() == BaseSymbol::zero()) {
                return Symbol<T>::create(BaseSymbol::zero());
            } else {
                return UnaryBase<T,U>::checkCst(
                    new Y<T,U>(term));
            }
        }

    protected:
        
        Y(const typename Term<U>::TermPtr& term) :
            UnaryOperator<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "Y";
        }

        virtual inline typename Term<T>::TermPtr functionDerivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return Y<T,U>::create(arg);
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("Y not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return Y<T,U>::create(arg);
        }
};

template <>
inline double Y<double,Vector::Vector2D<double> >::functionEvaluation
    (const Vector::Vector2D<double>& argVal) const
{
    return argVal.y();
}

}
}

#endif

