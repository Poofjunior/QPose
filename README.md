QPose
=====


## What is QPose?
A templated C++ library for manipulating pose data through dual quaternions

## What (the heck) is a Dual Quaternion? 
We can think of a dual quaternion as simply two quaternions that go together, 
where the individual quaternions are named the __real__ and the __dual__
component. A defined set of mathematical properties
come with working with dual quaternions, including some semblance of 
 * dual-quaternion addition
 * dual-quaetnion subtraction
 * dual-quaternion scalar multiplication (scalar times a Dual Quaternion)
 * dual-quaternion products (Dual Quaternion times a Dual Quaternion)
 
Dual Quaternions play a vital role in manipulating objects in 3D. 
Traditionally, homogeneous transformation matrices and linear algebra are used 
to encode and 
manipulate an object's position and orientation. Dual Quaternions are less 
operation-intensive (they require less calculations) and avoid some 
singularities that occur when encoding 3D position and orientation with 
Matrices. They can also be smoothly interpolated from one Dual Quaternion to 
another, a quality that is excellent for animation or kinematics and very 
cumbersome with homogeneous transformation matrices.

## Ok, Can I do math with this library?
Yes! Dual Quaternion addition, subtraction, scalar multiplication, and 
quaternion multiplication have been implemented. Since Dual Quaternions are 
commonly used for encoding objects with 6 degrees of freedom (position and 
orientation), member functions have also been implemented to set and get this 
information.

### Dual-Quaternion Addition

    QPose<float> myPose;    // default quaternion.
    QPose<float> myOtherPose;    // default quaternion.
    
    QPose<float> myNewPose = myPose + myOtherPose;   // OK!
    
### Dual-Quaternion Subtraction

    QPose<float> myPose;    // default quaternion.
    QPose<float> myOtherPose;    // default quaternion.
    
    QPose<float> myNewPose = myPose - myOtherPose;   // OK!
    
### Dual-Quaternion Scalar Multiplication

    QPose<float> myPose;    // default quaternion.
    int scalar = 10;
    
    QPose<float> myNewPose = scalar * myPose;   // OK!
    
### Dual-Quaternion Product 

    QPose<float> myPose;    // default quaternion.
    QPose<float> myOtherPose;    // default quaternion.
    
    QPose<float> myNewPose = myPose * myOtherPose;   // OK!


### Setting a Pose  
     
    float x, y, z, roll, pitch, yaw;

    ... //Assign values to above variables.

    QPose<float> myPose(x, y, z, roll, pitch, yaw);
    

### Getting a Pose  
     
    QPose<float> myPose;
    
    ... // myPose gets a pose assigned to it.

    float x, y, z, roll, pitch, yaw;
    
    myPose.get6DOF(x, y, z, roll, pitch, yaw);

    // x, y, z, roll, pitch, yaw now have values from myPose.
    

## Great, any caveats?
A few. Here they are:

### Left-Scalar Multiplication Only
Only __left__ scalar multiplication has been implemented. In other words:

    QPose<float> myPose;    // default quaternion.
    int scalar = 10;
    
    QPose<float> myNewPose = scalar * myPose;   // OK!
    QPose<float> myNewPose = myPose * scalar;   // NOT OK!
    
Luckily, the Compiler will complain about this practice and refuse to compile 
it.

### Compiling requires C++11
QPose must be compiled with the -std=c++11 or -std=g++11 flag.
