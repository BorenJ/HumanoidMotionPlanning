// This file is the header file of Nao roobt. 
//It contains the name of a NaoRobot class and necessary self-defined calculation funtions.
#pragma once
#include <Eigen/Dense>

// deg2rad using template, can transfer matrix, vector, and scalar
template<typename T>
auto deg2rad(const T& degree) {
    return degree * M_PI / 180.0;
}

// Function to rotate a 3x3 matrix around the Z-axis
Eigen::Matrix3d rotz(double angle);
    
// Function to rotate a 3x3 matrix around the Y-axis
Eigen::Matrix3d roty(double angle); 

// Function to rotate a 3x3 matrix around the X-axis
Eigen::Matrix3d rotx(double angle); 

// Assign angle11 to 16*3 JointAngle
template<typename T>
Eigen::Matrix<double,16,3> JointAngle11to48(const T& q)
{   
    Eigen::Matrix<double,16,3> JointAngle = Eigen::Matrix<double, 16, 3>::Zero();
    // Pre assign Arm's angle
    JointAngle(2,1) = deg2rad(110);
    JointAngle(5,1) = deg2rad(110);
    JointAngle(3,0) = deg2rad(86);        
    JointAngle(6,0) = deg2rad(-86);
    JointAngle(3,2) = deg2rad(90);
    JointAngle(6,2) = deg2rad(-90);
    // Right Leg
    JointAngle(8,2)  = q[0];  // R45
    JointAngle(9,1) = q[1];
    JointAngle(9,2) = q[2];
    JointAngle(10,1) = q[3];
    JointAngle(11,1) = q[4];
    JointAngle(11,2) = q[5];
    // Left Leg
    JointAngle(12,2) = q[0];  // L45
    JointAngle(13,1) = q[6];
    JointAngle(13,2) = q[7];
    JointAngle(14,1) = q[8];
    JointAngle(15,1) = q[9];
    JointAngle(15,2) = q[10];

    return JointAngle;
}

// Assign 16*3 JointAngle to angle11
Eigen::VectorXd JointAngle48to11(const Eigen::Matrix<double,16,3>&  JointAngle);

// Nao robot class
class NaoRobot{
// model parameters of the robot 
private:
    const Eigen::Matrix<double, 16, 3> JointPos0;
    const Eigen::Matrix<double, 16, 3> JointMassLocOffset0;
    const Eigen::Matrix<double, 16, 3> LinkMassLocOffset0;
    const Eigen::Matrix<double, 16, 1> JointMass;
    const Eigen::Matrix<double, 16, 1> LinkMass;
    const Eigen::Matrix<double, 4, 3> FootCornerRight;
    const Eigen::Matrix<double, 4, 3> FootCornerLeft;
    const Eigen::Matrix<double, 16, 3> JointLim0;
    const Eigen::Matrix<double, 16, 3> JointLim1;
    const Eigen::Matrix<double, 16, 1> LinkRadians;

public:
    std::vector<Eigen::Matrix<double, 16, 3>> MassLoc0;
    std::vector<Eigen::Matrix<double, 16, 1>> Mass;       // Mass.reserve(2);
    std::vector<Eigen::Matrix<double, 16, 3>> JointLim;   // JointLim.reserve(2);
    std::vector<Eigen::Matrix<double, 4, 3>> FootCorner0; //FootCorner0.reserve(2);
    
    //constructor
    NaoRobot();

    // calculate forward kinematics of Nao robot
    std::tuple<
            std::vector<Eigen::Matrix3d>,
            Eigen::Matrix<double,16,3>, 
            std::vector<Eigen::Matrix<double,4,3>>, 
            Eigen::Matrix<double, 5, 3>
        > NaoForwardKinematicsFullBody(const Eigen::Matrix<double, 16, 3>& JointAngle); 
    
    // calculate center of mass of Nao
    std::tuple<
            Eigen::Vector3d,
            std::vector<Eigen::Matrix<double,16,3>>
        > NaoCOMLoc(
            const Eigen::Matrix<double,16,3>& JointPos,
            const std::vector<Eigen::Matrix3d>& R);

};
