#ifndef LEPH_SYMBOLIC_RE_HPP
#define LEPH_SYMBOLIC_RE_HPP

#include "Symbolic/src/UnaryFunction.hpp"
#include "Vector/src/Vector2D.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Re
 */
template <class T, class U>
class Re : public UnaryFunction<T,U>
{
    public:

        static inline typename Term<T>::TermPtr create(
            const typename Term<U>::TermPtr& term)
        {
            return typename Term<T>::TermPtr(
                new Re<T,U>(term));
        }

    protected:
        
        Re(const typename Term<U>::TermPtr& term) :
            UnaryFunction<T,U>(term)
        {
        }
        
        virtual inline std::string functionString() const
        {
            return "Re";
        }

        /**
         * @Inherit
         */
        virtual inline typename Term<T>::TermPtr computeDerivative
            (const BaseSymbol::BaseSymbolPtr& sym)
        {
            return functionderivative(UnaryFunction<T,U>::_arg->derivate(sym));
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

