#ifndef LEPH_SYMBOLIC_RE_HPP
#define LEPH_SYMBOLIC_RE_HPP

#include "Symbolic/src/UnaryOperator.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Re
 */
template <class T, class U>
class Re : public UnaryOperator<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            typename Term<U>::TermPtr term)
        {
            return typename Term<T>::TermPtr(
                new Re<T,U>(term));
        }

    protected:
        
        Re(const typename Term<U>::TermPtr& term) :
            UnaryOperator<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "Re";
        }

        virtual inline typename Term<T>::TermPtr functionderivative
            (const typename Term<U>::TermPtr& arg) const
        {
            return Re<T,U>::create(arg);
        }

        virtual inline T functionEvaluation(const U& argVal) const
        {
            throw std::logic_error("Re not implemented");
        }
        
        virtual inline typename Term<T>::TermPtr functionCreate
            (const typename Term<U>::TermPtr& arg) const
        {
            return Re<T,U>::create(arg);
        }
};

template <>
inline double Re<double,Vector::Vector2D<double> >::functionEvaluation
    (const Vector::Vector2D<double>& argVal) const
{
    return argVal.x();
}

}
}

#endif

