#ifndef LEPH_SYMBOLIC_IM_HPP
#define LEPH_SYMBOLIC_IM_HPP

#include "Symbolic/src/UnaryOperator.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Im
 */
template <class T, class U>
class Im : public UnaryOperator<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            return typename Term<T>::TermPtr(
                new Im<T,U>(term));
        }

    protected:
        
        Im(const typename Term<U>::TermPtr& term) :
            UnaryOperator<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "Im";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return Im<T,U>::create(arg);
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("Im not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return Im<T,U>::create(arg);
        }
};

template <>
inline double Im<double,Vector::Vector2D<double> >::functionEvaluation
    (const Vector::Vector2D<double>& argVal) const
{
    return argVal.y();
}

}
}

#endif
