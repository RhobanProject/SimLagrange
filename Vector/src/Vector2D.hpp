#ifndef LEPH_VECTOR_VECTOR2D_HPP
#define LEPH_VECTOR_VECTOR2D_HPP

#include <cmath>
#include <iostream>

namespace Leph {
namespace Vector {

/**
 * Vector2D
 *
 * Simple implementation of
 * two dimensional vectors
 *
 * Template parameter is scalar type
 */
template <class scalar>
class Vector2D
{
    public:

        /**
         * Initialization
         */
        Vector2D() :
            _x(0),
            _y(0)
        {
        }
        Vector2D(scalar val) :
            _x(val),
            _y(val)
        {
        }
        Vector2D(scalar x, scalar y) :
            _x(x),
            _y(y)
        {
        }

        /**
         * Components access
         */
        inline const scalar& x() const
        {
            return _x;
        }
        inline scalar& x()
        {
            return _x;
        }
        inline const scalar& y() const
        {
            return _y;
        }
        inline scalar& y()
        {
            return _y;
        }

        /**
         * Return the vector norm
         * and squared norm
         */
        inline scalar norm() const
        {
            return sqrt(squaredNorm());
        }
        inline scalar squaredNorm() const
        {
            return _x*_x + _y*_y;
        }

        /**
         * Compute the dot product between two vectors
         */
        static inline scalar dot
            (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
        {
            return v1.x()*v2.x() + v1.y()*v2.y();
        }

        /**
         * Compute the distance and squared distance between two vectors
         */
        static inline scalar dist
            (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
        {
            return sqrt(squaredDist(v1, v2));
        }
        static inline scalar squaredDist
            (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
        {
            return (v1.x()-v2.x())*(v1.x()-v2.x()) 
                + (v1.y()-v2.y())*(v1.y()-v2.y());
        }

        /**
         * Display
         */
        inline void print() const
        {
            std::cout << "[" << _x << " " << _y << "]" << std::endl;
        }

    private:

        /**
         * Vector components
         */
        scalar _x;
        scalar _y;
};

/**
 * Vector-Vector operator
 */
template <class scalar>
inline Vector2D<scalar> operator+
    (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
{
    return Vector2D<scalar>(v1.x()+v2.x(), v1.y()+v2.y());
}
template <class scalar>
inline Vector2D<scalar> operator-
    (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
{
    return Vector2D<scalar>(v1.x()-v2.x(), v1.y()-v2.y());
}
template <class scalar>
inline scalar operator*
    (const Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
{
    return Vector2D<scalar>::dot(v1, v2);
}

/**
 * Vector operator
 */
template <class scalar>
inline Vector2D<scalar> operator+(const Vector2D<scalar>& v)
{
    return Vector2D<scalar>(v.x(), v.y());
}
template <class scalar>
inline Vector2D<scalar> operator-(const Vector2D<scalar>& v)
{
    return Vector2D<scalar>(-v.x(), -v.y());
}

/**
 * Compound assignment operators
 */
template <class scalar>
inline void operator+=
    (Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
{
    v1 = v1 + v2;
}
template <class scalar>
inline void operator-=
    (Vector2D<scalar>& v1, const Vector2D<scalar>& v2)
{
    v1 = v1 - v2;
}

/**
 * Scalar-Vector operator
 */
template <class scalar>
inline Vector2D<scalar> operator*
    (const scalar& s, const Vector2D<scalar>& v)
{
    return Vector2D<scalar>(s*v.x(), s*v.y());
}
template <class scalar>
inline Vector2D<scalar> operator*
    (const Vector2D<scalar>& v, const scalar& s)
{
    return Vector2D<scalar>(s*v.x(), s*v.y());
}
template <class scalar>
inline Vector2D<scalar> operator/
    (const Vector2D<scalar>& v, const scalar& s)
{
    return Vector2D<scalar>(v.x()/s, v.y()/s);
}

template <class scalar>
inline std::ostream& operator<<
    (std::ostream& stream, const Vector2D<scalar>& v)
{
    stream << "[" << v.x() << " " << v.y() << "]";
    return stream;
}

}
}

#endif

