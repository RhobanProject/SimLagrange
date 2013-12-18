#ifndef LEPH_SMARTPOINTER_SMARTPTR_HPP
#define LEPH_SMARTPOINTER_SMARTPTR_HPP

#include <stdexcept>
#include <typeinfo>
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
         * Initialize an empty pointer
         */
        SmartPtr() :
            _pointer(NULL),
            _counter(NULL)
        {
            _counter = new ReferenceCounter();
        }

        /**
         * Initialization with the newly allocated Object
         */
        SmartPtr(T* obj) :
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
         * Copy constructor for inheritance
         * Throw bad_cast on error
         * (T is not a base class of U)
         */
        template <class U>
        SmartPtr(const SmartPtr<U>& ptr) :
            _counter(ptr._counter)
        {
            T* pt = dynamic_cast<T*>(ptr._pointer);
            if (ptr._pointer != NULL && pt == NULL) {
                throw std::bad_cast();
            }

            _pointer = pt;
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
            if (_pointer == NULL) {
                throw std::logic_error("SmartPtr null pointer");
            }

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

        /**
         * Explicit getter
         */
        inline const T* getPointer() const
        {
            return _pointer;
        }
        inline T* getPointer()
        {
            return _pointer;
        }
        inline const T& getReference() const
        {
            return *_pointer;
        }
        inline T& getReference()
        {
            return *_pointer;
        }

        /**
         * Return true if the pointer is null
         */
        inline bool isNull() const
        {
            return _pointer == NULL;
        }

        /**
         * Convertion for inheritance purpose
         * Throw bad_cast on error
         * (U is not a base class of T)
         */
        template <class U>
        operator SmartPtr<U>() const
        {
            return SmartPtr<U>(*this);
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

        /**
         * Friend inter SmartPtr
         */
        template <class U>
        friend class SmartPtr;
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

