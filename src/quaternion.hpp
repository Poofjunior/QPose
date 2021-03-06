/**
 * Templated Quaternion Class
 * \author Joshua Vasquez
 * \date August 31, 2014
 */
#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <iostream>
#include <ctgmath>
#include "commonMath.hpp"



/**
 * \class Quaternion
 * \brief a templated quaternion class that also enables quick storage and
 *        retrieval of rotations encoded as a vector3 and angle.
 * \details All angles are in radians.
 * \warning This template is intended to be instantiated with a floating point
 *          data type.
 */
template <typename T> class Quaternion
{
    public:
        Quaternion()
        : w_(1), x_(0), y_(0), z_(0)
        {}

        Quaternion( T w, T x, T y, T z)
        : w_(w), x_(x), y_(y), z_(z)
        {}

        static Quaternion fromAngleAxis(T theta, T x, T y, T z)
        {
            Quaternion q;
            q.encodeRotation(theta, x, y, z);
            return q;
        }

        ~Quaternion()
        {}


/**
 * Quaternion Rotation Properties for straightforward usage of quaternions
 *  to store rotations.
 */

/**
 * \brief Store a normalized rotation in the quaternion encoded as a rotation
 *        of theta about the vector (x,y,z).
 */
        void encodeRotation( const T theta, const T x, const T y, const T z)
        {
            w_ = cos(theta / 2);
            x_ = x * sin(theta / 2);
            y_ = y * sin(theta / 2);
            z_ = z * sin(theta / 2);
            normalize();
        }

/**
 * \brief same as encodeRotation
 */
        void setAngleAxis( const T theta, const T x, const T y, const T z)
        {
            encodeRotation(theta, x, y, z);
        }

/**
 * \brief Retrieve the rotation (angle and vector3) stored in the quaternion.
 * \warning only unit quaternions represent rotation.
 * \details A quaternion:
 *          Q = cos(alpha) + Usin(alpha), where U is a vector3, stores a
            rotation
 *          of 2*alpha about the 3D axis U. This member function retrieves
            theta and U, where theta = 2*alpha is the amount of rotation
            about the vector U.
 * \note the angle retrieved is in radians.
 */
    void getAngleAxis( T& theta, T& x, T& y, T& z)
    {
        // Acquire the amount of rotation. Prevent against rounding error.
        if ((w_ > 1) || (w_ < -1))
            theta = 2 * acos(1);
        else
            theta = 2 * acos(w_);
        /// The following is 'more numerically stable' according to Wikipedia:
        /// but it doesn't work as well for small angles.
        //theta = 2 * atan2( norm(), w_);

        T commonVal = sin(theta /2);

        // Acquire rotational axis. Guard agains division by 0.
        if (commonVal != 0)
        {
            x = x_ / commonVal;
            y = y_ / commonVal;
            z = z_ / commonVal;
        }
        else // Guard against division by zero. Values are bogus but ignored.
        {
            x = x_;
            y = y_;
            z = z_;
        }
    }


/**
 * \brief rotate a vector3 (x,y,z) by the angle theta about the axis
 * (U_x, U_y, U_z) stored in the quaternion.
 */
    void rotateVector(T& x, T& y, T& z)
    {
        Quaternion q = (*this);
        Quaternion qStar = conjugate();
        Quaternion rotatedVal = q * Quaternion(0, x, y, z) * qStar;

        x = rotatedVal.x_;
        y = rotatedVal.y_;
        z = rotatedVal.z_;
    }


/**
 * Quaternion Mathematical Properties
 * implemented below */

/// Addition
        Quaternion operator+(const Quaternion& q2)
        {
            return Quaternion(  (w_ + q2.w_),
                                (x_ + q2.x_),
                                (y_ + q2.y_),
                                (z_ + q2.z_));
        }

/// Subtraction
        Quaternion operator-(const Quaternion& q2)
        {
            return Quaternion(  (w_ - q2.w_),
                                (x_ - q2.x_),
                                (y_ - q2.y_),
                                (z_ - q2.z_));
        }

/// equality
        bool operator==(const Quaternion& q2) const
        {
            return (w_ == q2.w_) && (x_ == q2.x_)
                && (y_ == q2.y_) && (z_ == q2.z_);
        }

/// (left) Scalar Multiplication
/**
 * \brief implements scalar multiplication for arbitrary scalar types.
 */
        template <typename U> friend Quaternion operator*(const U scalar,
                                                      const Quaternion& q)
        {
            return Quaternion(  (scalar * q.w_),
                                (scalar * q.x_),
                                (scalar * q.y_),
                                (scalar * q.z_));
        }


/// Quaternion Product
        Quaternion operator*(const Quaternion& q2)
        {
            return Quaternion(
                        ((w_*q2.w_) - (x_*q2.x_) - (y_*q2.y_) - (z_*q2.z_)),
                        ((w_*q2.x_) + (x_*q2.w_) + (y_*q2.z_) - (z_*q2.y_)),
                        ((w_*q2.y_) - (x_*q2.z_) + (y_*q2.w_) + (z_*q2.x_)),
                        ((w_*q2.z_) + (x_*q2.y_) - (y_*q2.x_) + (z_*q2.w_)));
        }

/// "Product" Assignment
// TODO: test --> check multiplication between two different quaternions.
// TODO: test --> check self-multiplication.
        Quaternion& operator*=(const Quaternion& q)
        {
            T w = w_;
            T x = x_;
            T y = y_;
            T z = z_;

            if (this == &q)
            {
                T w2 = w_;
                T x2 = x_;
                T y2 = y_;
                T z2 = z_;

                w_ = (w*w2) - (x*x2) - (y*y2) - (z*z2);
                x_ = (w*x2) + (x*w2) + (y*z2) - (z*y2);
                y_ = (w*y2) - (x*z2) + (y*w2) + (z*x2);
                z_ = (w*z2) + (x*y2) - (y*x2) + (z*w2);
            }
            else
            {
                w_ = (w*q.w_) - (x*q.x_) - (y*q.y_) - (z*q.z_);
                x_ = (w*q.x_) + (x*q.w_) + (y*q.z_) - (z*q.y_);
                y_ = (w*q.y_) - (x*q.z_) + (y*q.w_) + (z*q.x_);
                z_ = (w*q.z_) + (x*q.y_) - (y*q.x_) + (z*q.w_);
            }
            return *this;
        }


/// Quaternion Power function
/**
 * \brief perform the power operation on a quaternion
 * \details A quaternion Q = (w, x, y, z) may be written as the
 * product of a scalar and a unit quaternion: Q = N*q =
 * N[sin(theta) + U_x*cos(theta) + U_y*cos(theta) + U_k*cos(theta)], where N is
 * a scalar and U is a vector3 (U_x, U_y, U_z) representing the normalized
 * vector component of the original quaternion, aka: (x,y,z). Raising a
 * quaternion to a p._v.w*_v.y - rhs._v.x*_v.z + rhs._v.y*_v.w + rhs._v.z*_v.x,wer can be done most easily in this form.
 */
        static Quaternion power(const Quaternion q1, T p)
        {
            T magnitude = q1.norm();

            Quaternion unitQuaternion = q1;
            unitQuaternion.normalize();

            // unitQuaternion.w_ will always be less than 1, so no domain
            // error.
            T theta = acos(unitQuaternion.w_);


         // Perform math:
         // N^p * [cos(p * theta)  + U*sin(p * theta)], where U is a vector.
            T poweredMag = pow(magnitude, p);  // N^p
            T cospTheta = cos(p * theta);
            T sinpTheta = sin(p * theta);
/*
            std::cout << "poweredMag is " << poweredMag << std::endl;
            std::cout << "cospTheta is " << cospTheta << std::endl;
            std::cout << "p * Theta is " << p * theta << std::endl;
            std::cout << "sinpTheta is " << sinpTheta << std::endl;
*/

            // Note: U_x, U_y, U_z exist in normalized q1.

            return Quaternion( poweredMag * cospTheta,
                               poweredMag * unitQuaternion.x_ * sinpTheta,
                               poweredMag * unitQuaternion.y_ * sinpTheta,
                               poweredMag * unitQuaternion.z_ * sinpTheta);
        }

/**
 * \brief returns the dot product of two quaternions.
 */
        static T dotProduct(const Quaternion q1, const Quaternion q2)
        {
            T result = 0.5 * ((q1.conjugate() * q2)
                       + (q1 * q2.conjugate()) ).w_;
            return result;
        }

/// Conjugate
        Quaternion conjugate()
        {
            return Quaternion(  w_, -x_, -y_, -z_);
        }

/// Norm
        T norm()
        {
            return sqrt((w_ * w_) + (x_ * x_) + (y_ * y_) + (z_ * z_));
        }

/// inverse
        Quaternion inverse()
        {
            return (1/(*this).norm()) * (*this).conjugate();
        }

// Normalization
/**
 * \fn void normalize()
 * \brief normalizes the quaternion to magnitude 1
 */
        void normalize()
        {
            // should never happen unless the quaternion wasn't initialized
            // correctly.
            assert( !((w_ == 0) && (x_ == 0) && (y_ == 0) && (z_ == 0)));
            T theNorm = norm();
            assert(theNorm > 0);
            (*this) = (1.0/theNorm) * (*this);
            return;
        }


/**
 * \brief return a quaternion that is a linear interpolation between q1 and q2
 *        where percentage (from 0 to 1) defines the amount of interpolation
 * \details morph one quaternion into the other with constant 'velocity.'
 *          Implementation details from Wikipedia article on Slerp.
 */
        static Quaternion slerp(const Quaternion q1, const Quaternion q2,
                                T percentage)
        {
            try
            {
                if ((percentage > 1) || (percentage < 0))
                    throw 0;
            }
            catch(int e)
            {
                std::cout << "error: interpolation factor outside 0 to 1 "
                          << "bound." << std::endl;
            }

            Quaternion result;

            T theta = acos(dotProduct(q1, q2));
            T leftCoeff;
            T rightCoeff;

            if (CommonMath::almostEqual(theta, 0, 1000))
            {
                /// as theta --> 0, SLERP becomes LERP.
                /// see Wikipedia slerp article.
                leftCoeff = (1 - percentage);
                rightCoeff = percentage;
            }
            else
            {
                leftCoeff = sin((1 - percentage) * theta)/
                              sin(theta);
                rightCoeff  = sin(percentage * theta) /
                                sin(theta);
            }

            result = leftCoeff * q1 + rightCoeff * q2;
            result.normalize();
            return result;
        }

/**
 * \fn template <typename U> friend std::ostream& operator <<
 *                                  (std::ostream& os, const Quaternion<U>& q);
 * \brief a templated friend function for printing quaternions.
 * \details T cannot be used as dummy parameter since it would be shared by
 *          the class, and this function is not a member function.
 */
    template <typename U> friend std::ostream& operator << (std::ostream& os,
                                                    const Quaternion<U>& q);

        T w_;
        T x_;
        T y_;
        T z_;
};



template <typename T> std::ostream& operator<< (std::ostream& os,
                                const Quaternion<T>& q)
{
    os << "(" << q.w_ << ", " <<
                 q.x_ << ", " <<
                 q.y_ << ", " <<
                 q.z_ << ")";
    return os;
}

#endif // QUATERNION_HPP
