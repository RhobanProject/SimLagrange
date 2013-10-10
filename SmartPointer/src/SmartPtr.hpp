#ifndef LEPH_SMARTPOINTER_SMARTPTR_HPP
#define LEPH_SMARTPOINTER_SMARTPTR_HPP

#include <stdexcept>
#include "SmartPointer/src/ReferenceCounter.hpp"

namespace Leph {
namespace SmartPointer {

/**
 * SmartPtr
 *
 * Simple smart pointer implementation for
 * type T
 */
template <class T>
class SmartPtr
{
    public:

        /**
         * Initialization with the newly allocated Object
         */
        explicit SmartPtr(T* obj) :
            _pointer(obj),
            _counter(NULL)
        {
            if (_pointer == NULL) {
                throw std::bad_alloc();
            }
            _counter = new ReferenceCounter();
        }

        /**
         * Destruction
         */
        virtual ~SmartPtr()
        {
            _counter->decr();
            if (_counter->count() == 0) {
                desallocate();
            }
        }

        /**
         * Copy
         */
        SmartPtr(const SmartPtr<T>& ptr) :
            _pointer(ptr._pointer),
            _counter(ptr._counter)
        {
            _counter->incr();
        }

        /**
         * Assignment
         */
        inline SmartPtr<T>& operator=(const SmartPtr<T>& ptr)
        {
            if (&ptr != this) {
                //Old referenced pointer
                _counter->decr();
                if (_counter->count() == 0) {
                    desallocate();
                }
                //New referenced pointer
                _pointer = ptr._pointer;
                _counter = ptr._counter;
                _counter->incr();
            }

            return *this;
        }

        /**
         * Access
         */
        inline const T& operator*() const
        {
            return *_pointer;
        }
        inline const T* operator->() const
        {
            return _pointer;
        }
        inline T& operator*()
        {
            return *_pointer;
        }
        inline T* operator->()
        {
            return _pointer;
        }

    private:

        /**
         * The referenced pointer
         */
        T* _pointer;

        /**
         * Reference counter
         */
        ReferenceCounter* _counter;

        /**
         * Desalloc the referenced pointer
         */
        inline void desallocate()
        {
            if (_pointer != NULL) {
                delete _pointer;
                _pointer = NULL;
            }
            if (_counter != NULL) {
                delete _counter;
                _counter = NULL;
            }
        }
};

/**
 * Comparison
 */
template <class T>
inline bool operator==(const SmartPtr<T>& ptr1, const SmartPtr<T>& ptr2)
{
    return (ptr1.operator->() == ptr2.operator->());
}

}
}

#endif

