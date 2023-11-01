// This is the source file of Nao robot.
//It contains the implementation of the class and functions defined in the header file nao.h.
#include <Eigen/Dense>
#include <iostream>
#include "nao.hpp"

// Function to rotate a 3x3 matrix around the Z-axis
Eigen::Matrix3d rotz(double angle) 
{
    Eigen::Matrix3d rotation;
    double c = cos(angle);
    double s = sin(angle);
    rotation << c, -s, 0,
                s, c, 0,
                0, 0, 1;
    return rotation;
}

    // Function to rotate a 3x3 matrix around the Y-axis
Eigen::Matrix3d roty(double angle) 
{
    Eigen::Matrix3d rotation;
    double c = cos(angle);
    double s = sin(angle);
    rotation << c, 0, s,
                0, 1, 0,
                -s, 0, c;
    return rotation;
}

    // Function to rotate a 3x3 matrix around the X-axis
Eigen::Matrix3d rotx(double angle) 
{
    Eigen::Matrix3d rotation;
    double c = cos(angle);
    double s = sin(angle);
    rotation << 1, 0, 0,
                0, c, -s,
                0, s, c;
    return rotation;
}

// Assign 16*3 JointAngle to angle11
Eigen::VectorXd JointAngle48to11(const Eigen::Matrix<double,16,3>&  JointAngle)
{
    Eigen::VectorXd q(11);
    // Pelvis
    q(0) = JointAngle(8,2);
    // Right Leg
    q(1) = JointAngle(9,1);
    q(2) = JointAngle(9,2);
    q(3) = JointAngle(10,1);
    q(4) = JointAngle(11,1);
    q(5) = JointAngle(11,2);
    // Left Leg
    q(6) = JointAngle(13,1);
    q(7) = JointAngle(13,2);
    q(8) = JointAngle(14,1);
    q(9) = JointAngle(15,1);
    q(10) = JointAngle(15,2);
    return q;
}

// implement the NaoRobot class 
NaoRobot::NaoRobot() :
    JointPos0((Eigen::Matrix<double, 16, 3>() <<
        0     , 0    , 0    ,     // 1 Torso
        0     , 0    , 126.5,     // 2 Head
        0     ,-98  , 100  ,     // 3 RShoulder
        105   ,-113 , 100  ,     // 4 RElbow
        160.95,-113 , 100  ,     // 5 RWrist
        0     , 98  , 100  ,     // 6 LShoulder
        105   , 113 , 100  ,     // 7 RShoulder
        160.95, 113  , 100  ,    // 8 LWrist
        0     ,-50  , -85  ,     // 9 RHip45
        0     ,-50  , -85  ,     // 10 RHip
        0     ,-50  , -185 ,     // 11 RKnee
        0     ,-50  ,-287.9,    // 12 RAnkle
        0     , 50  , -85  ,     // 13 LHip45
        0     , 50  , -85  ,     // 14 LHip
        0     , 50  , -185 ,     // 15 LKnee
        0     , 50  ,-287.9      // 16 LAnkle      
    ).finished()),

    JointMassLocOffset0((Eigen::Matrix<double, 16, 3> () << 
        -0.00413,         0,           0.04342,   // 1 Torso
        -0.00112,         0,           0.05258,   // 2 Head
        -0.00165,     0.02663,         0.00014,   // 3 Rshoulder
        -0.02744,         0,          -0.00014,   // 4 RElbow
        0,                0,                 0,   // 5 RWrist
        -0.00165,    -0.02663,         0.00014,   // 6 Lshoulder
        -0.02744,         0,          -0.00014,   // 7 LElbow
        0,                0,                 0,   // 8 LWrist
        0,                0,                 0,   // 9 RHip45
        -0.01549,    -0.00029,        -0.00515,   //10 RHip
        0,                0,                 0,   //11 Rknee
        0.00045,    -0.00029,          0.00685,   //12 RAnkle
        0,                0,                 0,   //13 LHip45
        -0.01549,     0.00029,        -0.00515,   //14 LHip
        0,                0,                 0,   //15 Lknee
        0.00045,      0.00029,         0.00685    //16 LAnkle
    ).finished()),

    LinkMassLocOffset0((Eigen::Matrix<double, 16, 3> ( ) <<
        0,                0,                 0,        // 1 TorLink -> Torso
        -1e-5,            0,                -0.02742,   // 2 Neck    -> Head
        0.02455,         -0.00563,         0.0033,     // 3 RBiceps -> RShoulder
        0.02556,         -0.00281,         0.00076,    // 4 RForeArm-> RElbow
        0.03434,          0.00088,          0.00308,   // 5 RHand   -> RWrist
        0.02455,          0.00563,          0.0033,    // 6 LBiceps -> LShoulder
        0.02556,          0.00281,          0.00076,   // 7 LForeArm-> LElbow
        0.03434,         -0.00088,         0.00308,    // 8 LHand   -> LWrist
        -0.00781,         0.01114,          0.02661,    // 9 RPelvis -> RHip45
        0.00138,         -0.00221,        -0.05373,    // 10 RThigh  -> RHip
        0.00453,         -0.00225,        -0.04936,    // 11 RTibia  -> Rknee
        0.02542,         -0.0033,         -0.03239,    // 12 RFoot   -> RAnkle
        -0.00781,        -0.01114,         0.02661,     // 13 LPelvis -> LHip45
        0.00138,          0.00221,         -0.05373,   // 14 LThigh  -> LHip
        0.00453,          0.00225,         -0.04936,   // 15 LTibia  -> Lknee
        0.02542,          0.0033,          -0.03239   // 16 LFoot   -> LAnkle
    ).finished()),

    JointMass((Eigen::Matrix<double, 16, 1>() <<
        1.0496,    // 1 Torso
        0.60533,   // 2 Head
        0.09304,   // 3 Rshoulder
        0.06483,   // 4 RElbow
        0,         // 5 RWrist
        0.09304,   // 6 Lshoulder
        0.06483,   // 7 LElbow
        0,         // 8 LWrist
        0,         // 9 RHip45
        0.14053,   // 10 RHip
        0,         // 11 Rknee
        0.13416,   // 12 RAnkle
        0,         // 13 LPelvis -> LHip45
        0.14053,   // 14 LHip
        0,         // 15 Lknee
        0.13416   // 16 LAnkle
    ).finished()),

    LinkMass(( Eigen::Matrix<double, 16, 1>() << 
        0 ,          // 1 TorLink -> Torso
        0.07842,     // 2 Neck    -> Head
        0.15777,     // 3 RBiceps -> RShoulder
        0.07761,     // 4 RForeArm-> RElbow
        0.18533,     // 5 RHand   -> RWrist
        0.15777,     // 6 LBiceps -> LShoulder
        0.07761,     // 7 LForeArm-> LElbow
        0.18533,     // 8 LHand   -> LWrist
        0.06981,     // 9 RPelvis -> RHip45
        0.38968,     //10 RThigh  -> RHip
        0.30142,     //11 RTibia  -> Rknee
        0.17184,     //12 RFoot   -> RAnkle
        0.06981,     //13 LPelvis -> LHip45
        0.38968,     //14 LThigh  -> LHip
        0.30142,     //15 LTibia  -> Lknee
        0.17184     //16 LFoot   -> LAnkle
    ).finished()),

    FootCornerRight(( Eigen::Matrix<double, 4, 3>() <<
        70.25, +23.1, 0,
        70.25, -29.9, 0,
        -30.25, -29.9, 0,
        -29.65, +23.1, 0
    ).finished()),

    FootCornerLeft(( Eigen::Matrix<double, 4, 3>() <<
        70.25, +29.9, 0,
        70.25, -23.1, 0,
        -29.65, -23.1, 0,
        -30.25, +29.9, 0
    ).finished()),
        
    JointLim0(deg2rad((Eigen::Matrix<double, 16, 3>() <<
        0, 0, 0,
        -119.5, -38.5, 0,
        -76, -119.5, 0,
        2, 0, -119.5,
        0, 0, -104.5,
        -18, -119.5, 0,
        -88.5, 0, -119.5,
        0, 0, -104.5,
        0, 0, -65.62,
        0, -88, -45.29,
        0, -5.9, 0,
        0, -67.79, -44.06,
        0, 0, 0,
        0, -88, -21.74,
        0, -5.29, 0,
        0, -68.15, -22.79
    ).finished())),
    
    JointLim1(deg2rad((Eigen::Matrix<double, 16, 3>() << 
        0, 0, 0,
        119.5, 29.5, 0,
        18, 119.5, 0,
        88.5, 0, 119.5,
        0, 0, 104.5,
        76, 119.5, 0,
        -2, 0, 119.5,
        0, 0, 104.5,
        0, 0, 42.44,
        0, 27.73, 21.74,
        0, 121.47, 0,
        0, 53.40, 22.8,
        0, 0, 42.44,
        0, 27.73, 45.29,
        0, 121.04, 0,
        0, 52.86, 44.06
    ).finished())),
    
    LinkRadians((Eigen::Matrix<double, 16, 1>() << 
        70,   // 1 TorLink -> Torso
        0,    // 2 Neck    -> Head
        0,    // 3 RBiceps -> RShoulder
        70/2, // 4 RForeArm-> RElbow
        0,    // 5 RHand   -> RWrist
        0,    // 6 LBiceps -> LShoulder
        70/2, // 7 LForeArm-> LElbow
        0,    // 8 LHand   -> LWrist
        0,    // 9 RPelvis -> RHip45
        75/2, // 10 RThigh  -> RHip
        75/2, // 11 RTibia  -> Rknee
        0,    // 12 RFoot   -> RAnkle
        0,    // 13 LPelvis -> LHip45
        75/2, // 14 LThigh  -> LHip
        75/2, // 15 LTibia  -> Lknee
        0    // 16 LFoot   -> LAnkle
        ).finished()) 
    // MassLoc0({JointMassLocOffset0*1000, LinkMassLocOffset0*1000}),
    // Mass({JointMass, LinkMass}),
    // FootCorner0({FootCornerRight, FootCornerLeft}),
    // JointLim({JointLim0,JointLim1}),
    {
        MassLoc0.reserve(2);
        MassLoc0.emplace_back(JointMassLocOffset0*1000);
        MassLoc0.emplace_back(LinkMassLocOffset0*1000);
        Mass.reserve(2);
        Mass.emplace_back(JointMass);
        Mass.emplace_back(LinkMass);
        JointLim.reserve(2);
        JointLim.emplace_back(JointLim0);
        JointLim.emplace_back(JointLim1);
        FootCorner0.reserve(2);
        FootCorner0.emplace_back(FootCornerRight);
        FootCorner0.emplace_back(FootCornerLeft);
    }
// Nao Forward Kinematics 
std::tuple<
    std::vector<Eigen::Matrix3d>,
    Eigen::Matrix<double,16,3>, 
    std::vector<Eigen::Matrix<double,4,3>>, 
    Eigen::Matrix<double, 5, 3>
> NaoRobot::NaoForwardKinematicsFullBody(const Eigen::Matrix<double, 16, 3>& JointAngle) 
{
    std::vector<Eigen::Matrix3d> R; R.reserve(16);
    Eigen::Matrix<double,16,3> JointPos;
    std::vector<Eigen::Matrix<double,4,3>> FootCorner; FootCorner.reserve(2); 
    Eigen::Matrix<double, 5, 3> EndeffLoc;

    // Torso --> Head
    JointPos.row(0) << JointPos0.row(0);
    R.emplace_back(Eigen::Matrix3d::Identity());
    JointPos.row(1) = JointPos.row(0) + (R[0]*(JointPos0.row(1)-JointPos0.row(0)).transpose()).transpose();
    R.emplace_back(R[0]*rotz(JointAngle(1,0))*roty(JointAngle(1,1)));

    // RShoulder --> RElbow --> RWrist
    R.emplace_back(roty(JointAngle(2,1))*rotz(JointAngle(2,0)));
    R.emplace_back(R[2]*rotx(JointAngle(3,2))*rotz(JointAngle(3,0)));
    R.emplace_back(R[3]*rotx(JointAngle(4,2)));

    JointPos.row(2) = JointPos0.row(2);
    JointPos.row(3) = JointPos.row(2) + (R[2]*(JointPos0.row(3)-JointPos0.row(2)).transpose()).transpose();
    JointPos.row(4) = JointPos.row(3) + (R[3]*(JointPos0.row(4)-JointPos0.row(3)).transpose()).transpose();

    // LShoulder --> LElbow --> LWrist
    R.emplace_back(roty(JointAngle(5,1))*rotz(JointAngle(5,0)));
    R.emplace_back(R[5]*rotx(JointAngle(6,2))*rotz(JointAngle(6,0)));
    R.emplace_back(R[6]*rotx(JointAngle(7,2)));
    JointPos.row(5) = JointPos0.row(5);
    JointPos.row(6) = JointPos.row(5) + (R[5]*(JointPos0.row(6)-JointPos0.row(5)).transpose()).transpose();
    JointPos.row(7) = JointPos.row(6) + (R[6]*(JointPos0.row(7)-JointPos0.row(6)).transpose()).transpose();
    
    //RHip45 --> RHip --> RKnee --> RAnkle
    double t = JointAngle(8,2);
    Eigen::Matrix3d R9;
    R9 <<
        cos(t),        -0.7071*sin(t),          0.7071*sin(t),
        0.7071*sin(t),  0.5000*cos(t) + 0.5000, 0.5000 - 0.5000*cos(t),
        -0.7071*sin(t),  0.5000 - 0.5000*cos(t), 0.5000*cos(t) + 0.5000;
    R.emplace_back(R9);
    R.emplace_back(R[8]*rotx(JointAngle(9,2))*roty(JointAngle(9,1)));
    R.emplace_back(R[9]*roty(JointAngle(10,1)));
    R.emplace_back(R[10]*roty(JointAngle(11,1))*rotx(JointAngle(11,2)));
    JointPos.row(8) = JointPos0.row(8);
    JointPos.row(9) = JointPos.row(8) + (R[8]*(JointPos0.row(9)-JointPos0.row(8)).transpose()).transpose();
    JointPos.row(10) = JointPos.row(9) + (R[9]*(JointPos0.row(10)-JointPos0.row(9)).transpose()).transpose();
    JointPos.row(11) = JointPos.row(10) + (R[10]*(JointPos0.row(11)-JointPos0.row(10)).transpose()).transpose();

    //LHip45 --> LHip --> LKnee --> LAnkle
    // double t = JointAngle(12,2); they share the same joint45
    Eigen::Matrix3d R13;
    R13 <<
        cos(t),         0.7071*sin(t),           0.7071*sin(t),
        -0.7071*sin(t),  0.5000*cos(t) + 0.5000, -0.5000 + 0.5000*cos(t),
        -0.7071*sin(t), -0.5000 + 0.5000*cos(t),  0.5000*cos(t) + 0.5000;

    R.emplace_back(R13);
    R.emplace_back(R[12]*rotx(JointAngle(13,2))*roty(JointAngle(13,1)));
    R.emplace_back(R[13]*roty(JointAngle(14,1)));
    R.emplace_back(R[14]*roty(JointAngle(15,1))*rotx(JointAngle(15,2)));
    JointPos.row(12) = JointPos0.row(12);
    JointPos.row(13) = JointPos.row(12) + (R[12]*(JointPos0.row(13)-JointPos0.row(12)).transpose()).transpose();
    JointPos.row(14) = JointPos.row(13) + (R[13]*(JointPos0.row(14)-JointPos0.row(13)).transpose()).transpose();
    JointPos.row(15) = JointPos.row(14) + (R[14]*(JointPos0.row(15)-JointPos0.row(14)).transpose()).transpose();
    
    // Whole body rotation and translation to the left foot center frame
    // Get the rotation matrix defining the left foot body frame, which = the
    // frame of the LAnkle
    
    Eigen::Matrix3d LfootR_temp = R[15];

    // Rotate all the rotation matrix
    for(int i = 0;i < 16;i++){
        R[i] = LfootR_temp.transpose()*R[i];
    }

    // Rotate all the joints to the new LFoot frame
    JointPos = (LfootR_temp.transpose()*JointPos.transpose()).transpose(); 

    // Get the translation between the torso and the center of the left foot
    Eigen::RowVector3d T = JointPos.row(15) + (R[15]* Eigen::Vector3d(0, 0, -45.19)).transpose() - JointPos.row(0);

    // Translate all the joint location according to the above translation
    JointPos = JointPos - T.replicate(16,1);

    // Get the updated endeffector location (Note head center is only for drawing)
    EndeffLoc.row(0) = JointPos.row(1) + (R[1]*Eigen::Vector3d(0, 0, 57.205) ).transpose();
    EndeffLoc.row(1) = JointPos.row(4) + (R[4]*Eigen::Vector3d(57.75, 0, -12.31) ).transpose();
    EndeffLoc.row(2) = JointPos.row(7) + (R[7]*Eigen::Vector3d(57.75, 0, -12.31) ).transpose();
    EndeffLoc.row(3) = JointPos.row(11) + (R[11]*Eigen::Vector3d(0, 0, -45.19) ).transpose();
    EndeffLoc.row(4) = JointPos.row(15) + (R[15]*Eigen::Vector3d(0, 0, -45.19) ).transpose();
    
    // Construct the feet polygons
    FootCorner.emplace_back((R[11] * FootCorner0[0].transpose()).transpose() + EndeffLoc.row(3).replicate(4,1)); 
    FootCorner.emplace_back((R[15] * FootCorner0[1].transpose()).transpose() + EndeffLoc.row(4).replicate(4,1)); 

    return std::make_tuple(R, JointPos, FootCorner, EndeffLoc);
}
// Nao Center of Mass
std::tuple<
    Eigen::Vector3d,
    std::vector<Eigen::Matrix<double,16,3>>
> NaoRobot::NaoCOMLoc(
    const Eigen::Matrix<double,16,3>& JointPos,
    const std::vector<Eigen::Matrix3d>& R)
{
    std::vector<Eigen::Matrix<double,16,3>> MassLoc;
    MassLoc.reserve(2);
    MassLoc.emplace_back(Eigen::Matrix<double, 16, 3>::Zero());
    MassLoc.emplace_back(Eigen::Matrix<double, 16, 3>::Zero());

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 16; j++){
            MassLoc[i].row(j) = (R[j]*MassLoc0[i].row(j).transpose() + JointPos.row(j).transpose()).transpose();
        }
    }
    Eigen::Vector3d Com = (MassLoc[0].transpose() * Mass[0] + MassLoc[1].transpose() * Mass[1]) / ((Mass[0] + Mass[1]).sum()); 
    return std::make_tuple(Com, MassLoc);
}
