#ifndef LEPH_ANY_ANY_HPP
#define LEPH_ANY_ANY_HPP

#include <typeinfo>
#include <stdexcept>

namespace Leph {
namespace Any {

/**
 * Any
 *
 * A container for any typed object
 * The object is contained by value (copy)
 * and must have default constructor and be copyable
 */
class Any
{
    public:

        /**
         * Create an empty Any Object
         * (its state is invalid for use)
         */
        Any() :
            _pointer(NULL),
            _type(NULL),
            _desallocateFunction(NULL),
            _copyFunction(NULL)
        {
        }

        /**
         * Initialization with an
         * object of type T
         */
        template <class T>
        Any(const T& elt) :
            _pointer(new T(elt)),
            _type(NULL),
            _desallocateFunction(NULL),
            _copyFunction(NULL)
        {
            _type = const_cast<std::type_info*>(&(typeid(elt)));
            _desallocateFunction = &Any::desallocate<T>;
            _copyFunction = &Any::copy<T>;
        }

        /**
         * Destructor
         * Call desallocate function associated with
         * the contained type
         */
        ~Any()
        {
            if (_desallocateFunction != NULL) {
                (this->*_desallocateFunction)();
            }
        }

        /**
         * Copy constructor
         */
        Any(const Any& any) :
            _pointer(NULL),
            _type(any._type),
            _desallocateFunction(any._desallocateFunction),
            _copyFunction(any._copyFunction)
        {
            if (any._pointer != NULL) {
                _pointer = (any.*(any._copyFunction))();
            }
        }

        /**
         * Assignement
         */
        inline Any& operator=(const Any& any)
        {
            if (this != &any) {
                //Desallocation of old value
                if (_desallocateFunction != NULL) {
                    (this->*_desallocateFunction)();
                }
                //Allocation of new value
                _type = any._type;
                _desallocateFunction = any._desallocateFunction;
                _copyFunction = any._copyFunction;
                _pointer = NULL;
                if (any._pointer != NULL) {
                    _pointer = (any.*(any._copyFunction))();
                }
            }

            return *this;
        }

        /**
         * Return the contained element
         */
        template <class T>
        inline T& get() const
        {
            return *(getPointer<T>());
        }

        /**
         * Return the contained pointer casted into
         * type T
         * (An exception is throw if the type is invalid)
         */
        template <class T>
        inline T* getPointer() const
        {
            if (*_type != typeid(T)) {
                throw std::logic_error("Any bad type");
            }
            if (_pointer == NULL) {
                throw std::logic_error("Any empty");
            }

            return static_cast<T*>(_pointer);
        }

    private:

        /**
         * Pointer to the contained element
         */
        void* _pointer;

        /**
         * The type info of the contained element
         * (const)
         */
        std::type_info* _type;

        /**
         * The function pointer to desallocation and copy method
         * (associated with the right type)
         */
        void (Any::*_desallocateFunction)();
        void* (Any::*_copyFunction)() const;

        /**
         * Desallocation the contained pointer
         * of type T
         */
        template <class T>
        inline void desallocate()
        {
            if (_pointer != NULL) {
                T* ptr = getPointer<T>();
                delete ptr;
                _pointer == NULL;
            }
        }

        /**
         * Return a newly allocated copy of
         * the contained pointer of type T
         */
        template <class T>
        inline void* copy() const
        {
            return (void*)(new T(get<T>()));
        }
};

}
}

#endif

