/**
 * qpose.hpp
 * \author Joshua Vasquez
 * \date September 1, 2014
 */

#ifndef QPOSE_HPP
#define QPOSE_HPP
#include "quaternion.hpp"
#include <ctgmath>

template <typename T> class QPose 
{
    public:
        QPose()
        {}

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
        
        QPose( Quaternion<T> rotation, Quaternion<T> translation)
        {
            real_ = rotation;
            dual_ = 0.5 * translation * rotation;
        }

        // Other constructors here.
        
        ~QPose()
        {}

        void computeTranslation()
        {
            Quaternion<T> result = 2 * dual_ * real_.conj(); 
            /// note: inverse of a quaternion is the same as the conjugate.
            posX_ = result.x_;
            posY_ = result.y_;
            posZ_ = result.z_;
        }

        T getX()
        {
            return posX_;
        }

        T getY()
        {
            return posY_;
        }

        T getZ()
        {
            return posZ_;
        }
        
        T getRoll()
        {
            // TODO: verify this!
            return atan(2*((real_.w_ * real_.x_) + (real_.y_ * real_.z_)),
                        (1 - 2*((real_.x_*real_.x_) + (real_.y_*real_.y_))));
        }

        T getPitch()
        {
            return asin(2*(real_.w_ * real_.y_ - real_.z_ * real_.x_));
        }

        T getYaw()
        {
            return atan(2*((real_.w_ * real_.z_) + (real_.x_ * real_.y_)),
                        (1 - 2*((real_.y_*real_.y_) + (real_.z_*real_.z_))));
        }

    private:
        Quaternion<T> real_;
        Quaternion<T> dual_;
        
        T position_[3] = {};    /// default initialize vector to zeros.

        T rotAxis_[3] = {};     /// default initialize vector to zeros.
        T rotAngle_;

        T roll_; 
        T pitch_; 
        T yaw_;

        T posX_;
        T posY_;
        T posZ_;
};
#endif //QPOSE_HPP
