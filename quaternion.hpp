/**
 * Templated Quaternion Class
 * \author Joshua Vasquez
 * \date August 31, 2014
 */
#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <iostream>
#include <ctgmath>



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
        Quaternion(){}
        Quaternion( T w, T x, T y, T z):
            w_(w), x_(x), y_(y), z_(z)
        {}

        ~Quaternion()
        {}


/** 
 * Quaternion Rotation Properties for straightforward usage of quaternions
 *  to store rotations.
 */

/**
 * \fn void encodeRotation( T theta, T x, T y, T z)
 * \brief Store a normalized rotation in the quaternion
 */
        void encodeRotation( T theta, T x, T y, T z)
        {
            w_ = cos(theta / 2);
            x_ = x * sin(theta / 2);
            y_ = y * sin(theta / 2);
            z_ = z * sin(theta / 2);
            normalize();
        }

/**
 * \fn void getRotation( T& theta, T& x, T& y, T& z)
 * \brief Retrieve the rotation (vector3 and angle ) stored in the quaternion.
 */
    void getRotation( T& theta, T& x, T& y, T& z)
    {
        theta = 2 * acos(w_);
        
        T commonVal = sin(theta /2);
        
        x = x_ / commonVal;
        y = y_ / commonVal;
        z = z_ / commonVal;
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

/// (left) Scalar Multiplication
/**
 * \fn template <typename U> friend Quaternion operator*(const U scalar,
 *                                                       const Quaternion& q)
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

/// Conjugate
        Quaternion conj()
        {
            return Quaternion(  w_, -x_, -y_, -z_);
        }

/// Norm
        T norm()
        {
            return sqrt((w_ * w_) + (x_ * x_) + (y_ * y_) + (z_ * z_));
        }

// Normalization
/**
 * \fn void normalize()
 * \brief normalizes the quaternion to magnitude 1
 */
        void normalize()
        {
            T magnitude = norm();
            // TODO: is default assignment operator OK?
            (*this) = (1/magnitude) * (*this); 
            return;
        }


// Unit Quaternion
/** 
 * \fn Quaternion unit()
 * \brief returns the normalized version of the quaternion
 */
        Quaternion unit()
        {
            T magnitude = norm();
            return (1/magnitude) * (*this);
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
