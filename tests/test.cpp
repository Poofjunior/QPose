#include <gtest/gtest.h>
#include <sstream>

#include "../src/quaternion.hpp"
#include "../src/qpose.hpp"
#include <iostream>
#include <ctgmath>


TEST (quaternion, defaultConstructor)
{
    Quaternion<float> myOtherQuat(1.0, 1.0, 0.0, 2.0);
}

TEST (quaternion, printing)
{
    std::stringstream buffer;
    std::stringstream other_buffer;
    Quaternion<float> my_quat(1.0, 1.0, 0.0, 2.0);
    Quaternion<float> my_other_quat(1.6, 2.3, 0.0, -2.2);

    buffer << my_quat;
    ASSERT_STREQ(buffer.str().c_str(), "(1, 1, 0, 2)");


    other_buffer << my_other_quat;
    ASSERT_STREQ(other_buffer.str().c_str(), "(1.6, 2.3, 0, -2.2)");
}


TEST (quaternion, norm)
{
    Quaternion<float> my_quat(1, 2, 3, 4);
    /// note quaternion components x,y,z returned from
    /// my_quat * my_quat.conjugate() are zero.
    ASSERT_FLOAT_EQ(sqrt((my_quat * my_quat.conjugate()).w_), my_quat.norm());
}

TEST (quaternion, equality)
{
    Quaternion<float> q1(1, 2, 3, 4);
    Quaternion<float> q2 = q1;
    ASSERT_EQ(q1, q2);

}


TEST (quaternion, product)
{
    Quaternion<float> q1(.182574, .365148, .547723, .730297);
    Quaternion<float> q2(.17609, .440225, .880451, 0);
    Quaternion<float> q3 = q1 * q2;

/// expected product checked against Wolfram alpha
    Quaternion<float> expected_q3(-.610841, -.4983182, .578691, .20897159);

    q1.normalize();
    q2.normalize();

    ASSERT_FLOAT_EQ(q3.w_, expected_q3.w_);
    ASSERT_FLOAT_EQ(q3.x_, expected_q3.x_);
    ASSERT_FLOAT_EQ(q3.y_, expected_q3.y_);
    ASSERT_FLOAT_EQ(q3.z_, expected_q3.z_);



    Quaternion<float>q4;
    Quaternion<float>q5;
    Quaternion<float>rotation;
    q4.encodeRotation(30 *(M_PI/180.), 1, 0, 0);
    std::cout << q4 << std::endl;
    q5.encodeRotation(20 *(M_PI/180.), 0, 0, 1);
    std::cout << q5 << std::endl;

    rotation = q4 * q5;
    float angle;
    float axis[3];

    rotation.getAngleAxis(angle, axis[0], axis[1], axis[2]);
    std::cout << rotation << std::endl;
    std::cout << "angle (deg): " << angle * (180./M_PI) << std::endl;
    std::cout << "x: " << axis[0] << std::endl;
    std::cout << "y: " << axis[1] << std::endl;
    std::cout << "z: " << axis[2] << std::endl;
}

/*
TEST (quaternion, power)
{
    Quaternion<float> my_quat(1, 2, 3, 4);
    std::cout << my_quat << std::endl;

    std::cout << "my_quat to the power of 3 " 
              << Quaternion<float>::power(my_quat, 3) 
              << std::endl;
}
*/



/*
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

    /// Rotation storage tests:

    Quaternion<float> rotQuat;

    // encode a 180-deg rotation about the z-axis.
    rotQuat.encodeRotation(M_PI/2, 0, 0, 1);

    float rotQuatTheta, rotQuatX, rotQuatY, rotQuatZ;
    rotQuat.getAngleAxis(rotQuatTheta, rotQuatX, rotQuatY, rotQuatZ);

    std::cout << "rotQuatTheta should be pi/2 and it's: " << rotQuatTheta 
              << std::endl;
    std::cout << "rotQuatX should be 0 and it's: " << rotQuatX << std::endl;
    std::cout << "rotQuatY should be 0 and it's: " << rotQuatY << std::endl;
    std::cout << "rotQuatZ should be 1 and it's: " << rotQuatZ << std::endl;



    QPose<float> myPose(1, 2, 3, 0, 0, 0);  // x, y, z, roll, pitch, yaw;

    float x, y, z;

    myPose.getTranslation(x, y, z);
    std::cout << "posX should be 1 and it's: " << x << std::endl;
    std::cout << "posY should be 2 and it's: " << y << std::endl;
    std::cout << "posZ should be 3 and it's: " << z << std::endl;

    
    std::cout << "Quaternion slerp" << std::endl;
    Quaternion<float> qa(0, 0, 0, 1);
    Quaternion<float> qb(0, 0, 1, 0);
    Quaternion<float> result;

    for (float i = 0; i <= 1.001; i += 0.05)
    {
        result = Quaternion<float>::slerp(qa, qb, i);
        std::cout << "slerp result from " << i << "is " 
                  << result << std::endl;
    }


    float v_x, v_y, v_z;
    v_x = 0;
    v_y = 1;
    v_z = 0;
    
    // Rotation of 2 * (M_PI/4) about (0,0,1)
    Quaternion<float> rotationZ;
    rotationZ.encodeRotation(-M_PI/2, 0, 0, 1);
    
    std::cout << "Vector3 before rotation: (" << v_x << ", " << v_y << ", " 
              << v_z << ")" << std::endl;

    rotationZ.rotateVector(v_x, v_y, v_z);

    std::cout << "Vector3 after rotation: (" << v_x << ", " << v_y << ", " 
              << v_z << ")" << std::endl;


    Quaternion<float> qRot;
    qRot.encodeRotation(M_PI/2, 0, 1, 0);
    Quaternion<float> qTrans(0, 1, 0, 0);

    QPose<float> myNewPose(qRot, qTrans);
    std::cout << std::endl;
    std::cout << "myNewPose is: " << myNewPose << std::endl;

    std::cout << " Input Rotation is: " << qRot << std::endl;
    std::cout << "Rotation quaternion is: " << myNewPose.getAngleAxis()  
              << std::endl;
    
    std::cout << " Input Translation is: " << qTrans << std::endl;
    std::cout << "Translation quaternion is: " << myNewPose.getTranslation()  
              << std::endl;
*/

    
// testing!
/*
    myQuat->encodeRotation(0, 0, 1, 0);
    myOtherQuat.encodeRotation(M_PI, 0, 1, 0);

    myQuat->normalize();
    myOtherQuat.normalize();

    Quaternion<float> slerpOut;
    for (float i = 0; i <= 1; i += 0.05)
    {
        slerpOut = Quaternion<float>::slerp(*myQuat, myOtherQuat, i);
        std::cout << slerpOut << std::endl;
        //std::cout << "percentage: " << i << std::endl;
        std::cout << "slerpOut magnitude: " << slerpOut.norm() << std::endl;
    }
*/

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
