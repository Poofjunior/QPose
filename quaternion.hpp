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
 * \warning only unit quaternions represent rotation. 
 * \details A quaternion:
 * Q = cos(alpha) + Usin(alpha), where U is a vector3, stores a rotation
 * of 2*alpha about the 3D axis U.
 */
    void getRotation( T& theta, T& x, T& y, T& z)
    {
        // Acquire the amount of rotation.
        theta = 2 * acos(w_);   
        
        T commonVal = sin(theta /2);

        // Acquire rotational axis. 
        x = x_ / commonVal;
        y = y_ / commonVal;
        z = z_ / commonVal;
    }


/**
 * \fn void rotate( T& x, T& y, T& z)
 * \brief rotate a vector3 (x,y,z) by the angle theta about the axis 
 * (U_x, U_y, U_z) stored in the quaternion.
 */
    void rotate(T& x, T& y, T& z)
    {
        Quaternion q = (*this);
        Quaternion qStar = (*this).conj();
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

/// Quaternion Power function
/**
 * \fn static Quaternion power(const Quaternion q1, T p)
 * \brief perform the power operation on a quaternion
 * \details A quaternion Q = (w, x, y, z) may be written as the
 * product of a scalar and a unit quaternion: Q = N*q = 
 * N[sin(theta) + U_x*cos(theta) + U_y*cos(theta) + U_k*cos(theta)], where N is
 * a scalar and U is a vector3 (U_x, U_y, U_z) representing the normalized
 * vector component of the original quaternion, aka: (x,y,z). Raising a 
 * quaternion to a power can be done most easily in this form.
 */
        static Quaternion power(Quaternion q1, T p)
        {
            T magnitude = q1.norm();

            Quaternion unitQuaternion = q1;
            unitQuaternion.normalize();
            
            T theta = acos(unitQuaternion.w_);

         // Perform math:
         // N^p * [cos(p * theta)  + U*sin(p * theta)], where U is a vector.
            T poweredMag = pow(magnitude, p);  // N^p
            T cospTheta = cos(p * theta);   
            T sinpTheta = sin(p * theta);
            // Note: U_x, U_y, U_z exist in normalized q1.

            return Quaternion( poweredMag * cospTheta,
                               poweredMag * unitQuaternion.x_ * sinpTheta,
                               poweredMag * unitQuaternion.y_ * sinpTheta,
                               poweredMag * unitQuaternion.z_ * sinpTheta);
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

/// magnitude
        T magnitude()
        {
            //return ((*this) * (*this).conj()).w_; 
            return (*this).norm();
        }

/// inverse 
        Quaternion inverse()
        {
            return (1/(*this).norm()) * (*this).conj(); 
        }

// Normalization
/**
 * \fn void normalize()
 * \brief normalizes the quaternion to magnitude 1
 */
        void normalize()
        {
            T theNorm = norm();
            // TODO: is default assignment operator OK?
            (*this) = (1/theNorm) * (*this); 
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
 * \fn static Quaternion slerp( Quaternion q1 Quaternion q2, 
 *                                 T percentage)
 * \brief return a quaternion that is a linear interpolation between q1 and q2
 *        where percentage (from 0 to 1) defines the amount of interpolation
 * \details morph one quaternion into the other with constant 'velocity.'
 */
        static Quaternion slerp( Quaternion q1, Quaternion q2, T percentage)
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

            // math is trivial once the power function is defined.
            result = q1 * power( ( (1/q1.norm()) * q1.conj() * q2), 
                                 percentage);
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
