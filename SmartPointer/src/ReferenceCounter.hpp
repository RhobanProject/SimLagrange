#ifndef LEPH_SMARTPOINTER_REFERENCECOUNTER_HPP
#define LEPH_SMARTPOINTER_REFERENCECOUNTER_HPP

#include <stdexcept>

namespace Leph {
namespace SmartPointer {

/**
 * ReferenceCounter
 *
 * Handle a counter of pointer references
 * for custom smart pointer implementation
 */
class ReferenceCounter
{
    public:

        /**
         * Initialization
         */
        ReferenceCounter() :
            _count(1)
        {
        }

        /**
         * Return the counter value
         */
        inline unsigned int count() const
        {
            return _count;
        }

        /**
         * Increment and decrement the counter
         */
        inline void incr()
        {
            _count++;
        }
        inline void decr()
        {
            if (_count == 0) {
                throw std::logic_error("ReferenceCounter invalid decr");
            }
            _count--;
        }

    private:

        /**
         * The counter
         */
        unsigned int _count;
};

}
}
#endif

