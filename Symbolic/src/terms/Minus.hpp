#ifndef LEPH_SYMBOLIC_MINUS_HPP
#define LEPH_SYMBOLIC_MINUS_HPP

#include "Symbolic/src/UnaryOperator.hpp"

namespace Leph {
namespace Symbolic {

/**
 * Minus
 */
template <class T>
class Minus : public UnaryOperator<T,T>
{
    public:

        static inline typename Term<T>::TermPtr create(
            const typename Term<T>::TermPtr& term)
        {
            //Test -(-(term)) == term case
            const Minus<T>* pt = dynamic_cast<const Minus<T>*>
                (term.getPointer());
            if (pt != NULL) {
                return pt->UnaryOperator<T,T>::_arg;
            } else {
                return typename Term<T>::TermPtr(
                    new Minus<T>(term));
            }
        }
    protected:
        
        Minus(const typename Term<T>::TermPtr& term) :
            UnaryOperator<T,T>(term)
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
            return Minus<T>::create(arg);
        }
};

}
}

#endif

