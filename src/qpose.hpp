/**
 * qpose.hpp
 * \author Joshua Vasquez
 * \date September 1, 2014
 */

#ifndef QPOSE_HPP
#define QPOSE_HPP
#include "quaternion.hpp"
#include <ctgmath>

/**
 * \brief a dual quaternion class for encoding transformations.
 * \details transformations are stored as first a translation; then a
 *          rotation. It is possible to switch the order. See this paper:
 *  https://www.thinkmind.org/download.php?articleid=intsys_v6_n12_2013_5
 */

template <typename T> class QPose
{
    public:
/**
 * \brief default constructor.
 */
        QPose()
        {}
/*
        : real_(1, 0, 0, 0)
        {
            dual_ = 0.5 * Quaternion<T>(0, 0, 0, 0) * real_;
        }
*/

/**
 * \brief constructor that takes cartesian coordinates and Euler angles as
 *        arguments.
 */
        QPose( T x, T y, T z, T roll, T pitch, T yaw)
        {
            // convert here.
            real_.w_ = cos(roll/2)*cos(pitch/2)*cos(yaw/2) +
                                        sin(roll/2)*sin(pitch/2)*sin(yaw/2);
            real_.x_ = sin(roll/2)*cos(pitch/2)*cos(yaw/2) -
                                        cos(roll/2)*sin(pitch/2)*sin(yaw/2);
            real_.y_ = cos(roll/2)*sin(pitch/2)*cos(yaw/2) +
                                        sin(roll/2)*cos(pitch/2)*sin(yaw/2);
            real_.z_ = cos(roll/2)*cos(pitch/2)*sin(yaw/2) -
                                        sin(roll/2)*cos(pitch/2)*cos(yaw/2);

            dual_ = 0.5 * Quaternion<T>(0, x, y, z) * real_;
        }

/**
 * \brief constructor that takes two quaternions as arguments.
 * \details The rotation
 *          quaternion has the conventional encoding for a rotation as a
 *          quaternion. The translation quaternion is a quaternion with
 *          cartesian coordinates encoded as (0, x, y, z)
 */
        QPose( Quaternion<T> translation, Quaternion<T> rotation)
        {
            real_ = rotation;
            dual_ = 0.5 * translation * rotation;
        }

        // Other constructors here.

        ~QPose()
        {}

/**
 * \brief store a rotation
 * \param angle is in radians
 */
    void encodeRotation(T angle, T x, T y, T z)
    {
        real_.encodeRotation(angle, x, y, z);
    }

    void encodeTranslation(T x, T y, T z)
    {
        dual_ = 0.5 * Quaternion<T>(0, x, y, z) * real_;
    }

/// handle accumulating error.
    void normalizeRotation()
    {
        T x, y, z;
        getTranslation(x, y, z);

        real_.normalize();

        encodeTranslation(x, y, z);
    }


/// Extracting Translation
/**
 * \brief a reference-based method for acquiring the latest
 *        translation data.
 */
        void getTranslation( T& x, T& y, T& z)
        {
            Quaternion<T> result = 2 * dual_ * real_.conjugate();
            /// note: inverse of a quaternion is the same as the conjugate.
            x = result.x_;
            y= result.y_;
            z = result.z_;
        }

        Quaternion<T> getTranslation()
        {
            return 2 * dual_ * real_.conjugate();
        }

/// Extracting Rotation
/**
 * \brief a reference-based method for acquiring the latest rotation data.
 */
        void getEuler( T& roll, T& pitch, T& yaw)
        {
/// FIXME: breaks for some value around PI.
            roll = getRoll();
            pitch = getPitch();
            yaw = getYaw();
        }

        Quaternion<T> getRotation()
        {
            return real_;
        }


/// Extraction everything (in a nice format)
    void get6DOF( T& x, T& y, T& z, T& roll, T& pitch, T& yaw)
    {
        getTranslation(x, y, z);
        getEuler(roll, pitch, yaw);
    }

/// Addition:
        QPose operator+(const QPose& q2)
        {
            QPose result;
            result.real_ = real_ + q2.real_;
            result.dual_ = dual_ + q2.dual_;
            return result;
        }

/// Subtraction:
        QPose operator-(const QPose& q2)
        {
            QPose result;
            result.real_ = real_ - q2.real_;
            result.dual_ = dual_ - q2.dual_;
            return result;
        }

/// (left) Scalar Multiplication
/**
 * \fn template <typename U> friend Quaternion operator*(const U scalar,
 * \brief implements scalar multiplication for arbitrary scalar types
 */
        template <typename U> friend QPose operator*(const U scalar,
                                                      const QPose& q)
        {
            QPose result;
            // Luckily, left-scalar multiplication is implemented for
            // Quaternions.
            result.real_ = scalar * q.real_;
            result.dual_ = scalar * q.dual_;
            return result;
        }


/// Dual Quaternion Product:
        QPose operator*(const QPose& q2)
        {
            QPose result;
            result.real_ = real_ * q2.real_;
            result.dual_ = (real_ * q2.dual_) + (dual_ * q2.real_);
            return result;
        }

/// Conjugate
        QPose conjugate()
        {
            QPose result;
            result.real_ = real_.conjugate();
            result.dual_ = dual_.conjugate();
        }

/// Magnitude
//TODO: verify this!
        T magnitude()
        {
            QPose result = (*this) * (*this).conjugate();
            std::cout << result.real_;
            return result.real_.w_;
        }


    private:
        Quaternion<T> real_;
        Quaternion<T> dual_;

        T position_[3] = {};    /// default initialize vector to zeros.

        T rotAxis_[3] = {};     /// default initialize vector to zeros.
        T rotAngle_;


        T getRoll()
        {
            // TODO: verify this!
            return atan2(2*((real_.w_ * real_.x_) + (real_.y_ * real_.z_)),
                        (1 - 2*((real_.x_*real_.x_) + (real_.y_*real_.y_))));
        }

        T getPitch()
        {
            return asin(2*(real_.w_ * real_.y_ - real_.z_ * real_.x_));
        }

        T getYaw()
        {
            return atan2(2*((real_.w_ * real_.z_) + (real_.x_ * real_.y_)),
                        (1 - 2*((real_.y_*real_.y_) + (real_.z_*real_.z_))));
        }

/**
 * \brief a friend function for printing
 */
        template <typename U> friend std::ostream& operator<<
                                                    (std::ostream& os,
                                                     const QPose<U>& q);
};

template <typename T> std::ostream& operator<< (std::ostream& os,
                                const QPose<T>& q)
{
    os << "[" << q.real_ << ", " <<
                 q.dual_ << ", " << "]" << std::endl;
    return os;
}

#endif //QPOSE_HPP
