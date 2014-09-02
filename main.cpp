#include "quaternion.hpp"
#include "qpose.hpp"
#include <iostream>


int main(void)
{
    Quaternion<float>* myQuat = new Quaternion<float>(1.0, 2.0, 3.0, 4.0);
    Quaternion<float> myOtherQuat(1.0, 1.0, 0.0, 2.0);

    std::cout << "My Quaternion: " << *myQuat << std::endl;
    std::cout << "My other Quaternion: " << myOtherQuat << std::endl;

    Quaternion<float> qMultProd = *myQuat * myOtherQuat;
    std::cout << "My Quaternion product: " << qMultProd << std::endl;

    std::cout << "Scalar multiplication: " << (2 * myOtherQuat) << std::endl;

    std::cout << "Unit quaternion: " << myOtherQuat.unit() << std::endl;

    myOtherQuat.normalize();
    std::cout << "Quaternion now normalized." << std::endl;
    std::cout << "normalized quaternion: " << myOtherQuat << std::endl;
    std::cout << "normalized quaternion magnitude: " << myOtherQuat.norm() 
              << std::endl;
    
    delete myQuat;


    QPose<float> myPose(1, 2, 3, 0, 0, 0);  // x, y, z, roll, pitch, yaw;
    myPose.computeTranslation();
    std::cout << "dual quaternion x should be 1, and it's: " 
              << myPose.getX() << std::endl;
    std::cout << "dual quaternion y should be 2, and it's: " 
              << myPose.getY() << std::endl;
    std::cout << "dual quaternion z should be 3, and it's: " 
              << myPose.getZ() << std::endl;
    return 0;
}
