#include "robot.h"
#include "joint.h"

#include <QGroupBox>
#include <QGridLayout>
#include <QFormLayout>
#include <QLabel>
#include <QButtonGroup>
#include <QRadioButton>

using Eigen::MatrixXd;

Robot::Robot(QString n)
{
    this->name = n;

    this->pLink = new Link();
    this->pJoint = new Joint("Controller");
    this->pRealJoint = new Joint("Real");
    Affine3d pose = this->FK(pJoint->get_joints_rad());
    this->pPose = new Pose(pose);

    this->pbFK = new QPushButton("FK ->");
    this->pbIK = new QPushButton("<- IK");

    // Group Front/Back
    //this->gbFrontBack = new QGroupBox(tr("Front / Back"));
    this->gbFrontBack = new QGroupBox();
    this->cbFront = new QRadioButton("&Front");
    this->cbBack = new QRadioButton("&Back");
    cbFront->setChecked(true);

    QVBoxLayout *vboxFrontBack = new QVBoxLayout;
    vboxFrontBack->addWidget(cbFront);
    vboxFrontBack->addWidget(cbBack);
    vboxFrontBack->addStretch(1);
    gbFrontBack->setLayout(vboxFrontBack);

    // Group Up/Down
    //this->gbUpDown = new QGroupBox(tr("Up / Down"));
    this->gbUpDown = new QGroupBox();
    this->cbUp = new QRadioButton("&Up");
    this->cbDown = new QRadioButton("&Down");
    cbUp->setChecked(true);

    QVBoxLayout *vboxUpDown = new QVBoxLayout;
    vboxUpDown->addWidget(cbUp);
    vboxUpDown->addWidget(cbDown);
    vboxUpDown->addStretch(1);
    gbUpDown->setLayout(vboxUpDown);

    // Group Positive/Negative
    //this->gbPosNeg = new QGroupBox(tr("Positive / Negative"));
    this->gbPosNeg = new QGroupBox();
    this->cbPositive = new QRadioButton("&Positive");
    this->cbNegative = new QRadioButton("&Negative");
    cbPositive->setChecked(true);

    QVBoxLayout *vboxPosNeg = new QVBoxLayout;
    vboxPosNeg->addWidget(cbPositive);
    vboxPosNeg->addWidget(cbNegative);
    vboxPosNeg->addStretch(1);
    gbPosNeg->setLayout(vboxPosNeg);

    // Group Main
    this->gbGroup = new QGroupBox("Robot");
    QHBoxLayout *layoutGroup = new QHBoxLayout();
    layoutGroup->addWidget(pJoint->gbJoints);

    QGroupBox *gbButtons = new QGroupBox();
    QVBoxLayout *vboxButtons = new QVBoxLayout;
    vboxButtons->addWidget(pbFK);
    vboxButtons->addWidget(pbIK);
    gbButtons->setLayout(vboxButtons);
    layoutGroup->addWidget(gbButtons);

    layoutGroup->addWidget(pPose->gbPose);

    QGroupBox *gbConf = new QGroupBox("Configuration");
    QVBoxLayout *vboxConf = new QVBoxLayout;
    vboxConf->addWidget(gbFrontBack);
    vboxConf->addWidget(gbUpDown);
    vboxConf->addWidget(gbPosNeg);
    gbConf->setLayout(vboxConf);
    layoutGroup->addWidget(gbConf);
    
    layoutGroup->addWidget(pRealJoint->gbJoints);

    layoutGroup->addWidget(pLink->gbLinks);
    gbGroup->setLayout(layoutGroup);

    connect(pbFK, &QPushButton::released, this, &Robot::pbFK_released);
    connect(pbIK, &QPushButton::released, this, &Robot::pbIK_released);
}

Robot::~Robot(){
    if (pJoint) {
        delete pJoint;
        pJoint = nullptr;
    }
    if (pLink) {
        delete pLink;
        pLink = nullptr;
    }
    if (pPose) {
        delete pPose;
        pPose = nullptr;
    }
}

Affine3d Robot::FK(Array<double, 6, 1> j)
{
    double J1, J2, J3, J4, J5, J6;
    J1 = j(0);
    J2 = j(1);
    J3 = j(2);
    J4 = j(3);
    J5 = j(4);
    J6 = j(5);

    // FK Step #1: from Frame 6 to Frame 5
    // Homogeneous transformation T_56
    // Joint 6
    // Translation
    Affine3d T_56t(Translation3d(Vector3d(this->pLink->get_a6x(), 0, 0)));
    // Rotation around X
    Affine3d T_56r(AngleAxisd(J6, Vector3d::UnitX()));
    Affine3d T_56 = T_56t * T_56r;

    // FK Step #2: from Frame 5 to Frame 4
    // Homogeneous transformation T_45
    // Joint 5
    // Translation
    Affine3d T_45t(Translation3d(Vector3d(this->pLink->get_a5x(), 0, 0)));
    // Rotation around Y
    Affine3d T_45r(AngleAxisd(J5, Vector3d::UnitY()));
    Affine3d T_45 = T_45t * T_45r;

    // FK Step #3: from Frame 4 to Frame 3
    // Homogeneous transformation T_34
    // Joint 4
    // Translation
    Affine3d T_34t(Translation3d(Vector3d(this->pLink->get_a4x(), 0, this->pLink->get_a4z())));
    // Rotation around X
    Affine3d T_34r(AngleAxisd(J4, Vector3d::UnitX()));
    Affine3d T_34 = T_34t * T_34r;

    // FK Step #4: from Frame 3 to Frame 2
    // Homogeneous transformation T_23
    // Joint 3
    // Translation
    Affine3d T_23t(Translation3d(Vector3d(0, 0, this->pLink->get_a3z())));
    // Rotation around Y
    Affine3d T_23r(AngleAxisd(J3, Vector3d::UnitY()));
    Affine3d T_23 = T_23t * T_23r;

    // FK Step #5: from Frame 2 to Frame 1
    // Homogeneous transformation T_12
    // Joint 2
    // Translation
    Affine3d T_12t(Translation3d(Vector3d(this->pLink->get_a2x(), 0, this->pLink->get_a2z())));
    // Rotation around Y
    Affine3d T_12r(AngleAxisd(J2, Vector3d::UnitY()));
    Affine3d T_12 = T_12t * T_12r;

    // FK Step #6: from Frame 1 to Frame 0
    // Homogeneous transformation T_01
    // Joint 1
    // Translation
    Affine3d T_01t(Translation3d(Vector3d(0, 0, this->pLink->get_a1z())));
    // Rotation around Y
    Affine3d T_01r(AngleAxisd(J1, Vector3d::UnitZ()));
    Affine3d T_01 = T_01t * T_01r;

    // Combined transformation: from Frame 6 to Frame 0
    Affine3d T_06 = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    return T_06;
}

ARCCode_t Robot::IK(Affine3d p, FrontBack fb, UpDown ud, PosNeg pn, Array<double, 6, 1>& joint){
    double J1, J2, J3, J4, J5, J6;
    double Rwrist11, Rwrist21, Rwrist31, Rwrist12, Rwrist13, Rwrist32, Rwrist33;
    double WPxy, l, h;
    double rho, b4x;
    double alpha, beta, gamma, delta;
    double cos_beta, sin_beta;
    double cos_gamma, sin_gamma;
    Affine3d MP, pJ1, pJ23;
    Vector3d x_hat, WP;
    Matrix3d Rarm, Rwrist;

    //Pose of the mounting point (center of the axe 6)
    MP = p;
    /*
    MP = UF * p * UT.inverse();

    switch(this->brand)
    {
        case IR:
            std::cout << "IK IR\n";
            break;
        case ABB:
            std::cout << "IK ABB\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
        case KUKA:
            std::cout << "IK KUKA\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
    }*/

    x_hat = MP.rotation() * Vector3d::UnitX();
    WP = MP.translation() - (this->pLink->get_a6x() * x_hat.normalized());

    // Find J1
    // Check if there is a shoulder singularity
    if((abs(WP(0)) < 0.001) && (abs(WP(1)) < 0.001)){
        // In this case we have a shoulder singularity.
        // Fix J1 as the actual value of J1
        J1 = joint(0);
    }
    else{
        // FRONT solution
        J1 = atan2(WP(1), WP(0));
        // To have the BACK solution I need to add or substract pi
        if(fb == Back){
            // BACK solution is selected
            if(J1 > 0)
                J1 -= M_PI;
            else
                J1 += M_PI;
        }
    }

    // Find J2 and J3
    WPxy = sqrt( pow(WP(0),2) + pow(WP(1),2) );

    switch (fb){
        case Front:
            // FRONT solution
            l = WPxy - this->pLink->get_a2x();
            break;
        case Back:
            // BACK solution
            l = WPxy + this->pLink->get_a2x();
            break;
    }
    h = WP(2) - this->pLink->get_a1z() - this->pLink->get_a2z();

    rho = sqrt( pow(h,2) + pow(l,2) );
    b4x = sqrt( pow(this->pLink->get_a4z(),2) + pow(this->pLink->get_a4x()+this->pLink->get_a5x(),2) );
    if(rho >= (this->pLink->get_a3z()+b4x)){
        // It is not possible to reach that point
        return ARC_ERR_KIN__POSE_NOT_REACHABLE;
    }
    if(rho <= abs(pLink->get_a3z()-b4x)){
        // J2 too close to the robot himself
        return ARC_ERR_APP_J2_TOO_CLOSE;
    }

    alpha = atan2(h, l);
    cos_beta = (pow(rho,2) + pow(this->pLink->get_a3z(),2) - pow(b4x,2)) / (2*rho*this->pLink->get_a3z());
    sin_beta = sqrt(1 - pow(cos_beta,2));
    beta = atan2(sin_beta, cos_beta);

    switch (ud){
        case Up:
            // UP solution
            J2 = M_PI_2 - alpha - beta;
            break;
        case Down:
            // DOWN solution
            J2 = M_PI_2 - alpha + beta;
            break;
    }

    cos_gamma = (pow(this->pLink->get_a3z(),2) + pow(b4x,2) - pow(rho,2)) / (2*this->pLink->get_a3z()*b4x);
    sin_gamma = sqrt(1 - pow(cos_gamma,2));
    gamma = atan2(sin_gamma, cos_gamma);
    delta = atan2(this->pLink->get_a4x()+this->pLink->get_a5x(), this->pLink->get_a4z());

    J3 = M_PI - gamma - delta;

    // Calculate Rarm from the values of J1, J2, J3
    pJ1 = AngleAxisd(J1, Vector3d::UnitZ());
    pJ23 = AngleAxisd(J2+J3, Vector3d::UnitY());
    Rarm = pJ1.rotation() * pJ23.rotation();
    // R = Rarm * Rwrist -> Rwrist = Rarm^T * R
    Rwrist = Rarm.transpose() * MP.rotation();

    //Find J4, J5, J6 from Rwrist
    Rwrist11 = Rwrist(0,0);
    Rwrist21 = Rwrist(1,0);
    Rwrist31 = Rwrist(2,0);
    Rwrist12 = Rwrist(0,1);
    Rwrist13 = Rwrist(0,2);
    Rwrist32 = Rwrist(2,1);
    Rwrist33 = Rwrist(2,2);

    if (Rwrist11 < 0.9999999) {
        if (Rwrist11 > -0.9999999) {
            switch (pn){
                case Positive:
                    J5 = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                    J4 = atan2(Rwrist21,-Rwrist31);
                    J6 = atan2(Rwrist12,Rwrist13);
                    break;
                case Negative:
                    J5 = atan2( -sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                    J4 = atan2(-Rwrist21,Rwrist31);
                    J6 = atan2(-Rwrist12,-Rwrist13);
                    break;
            }
        }
        else // Rwrist11 = −1 
        {
            // Wrist singularity. J5 = 180 -> This condition is not
            // possible because the spherical wrist cannot rotate J5 = 180.
            // Not a unique solution: J6 − J4 = atan2(Rwrist32,Rwrist33)
            J5 = M_PI;
            J4 = joint(3);
            J6 = atan2(Rwrist32,Rwrist33) + J4;
        }
    }
    else // Rwrist11 = +1
    {
        // Wrist singularity. J5 = 0
        // Not a unique solution: J4 + J6 = atan2(Rwrist32,Rwrist33)
        J5 = 0;
        J4 = joint(3);
        J6 = atan2(Rwrist32,Rwrist33) - J4;
    }

    //switch(this->brand)
    //{
    //    case IR:
    //        break;
    //    case ABB:
    //        break;
    //    case KUKA:
    //        J1 = -J1;
    //        J2 -= M_PI_2;
    //        J3 += M_PI_2;
    //        J4 = -J4;
    //        J6 = -J6;
    //        break;
    //}

    joint << J1, J2, J3, J4, J5, J6;
    return ARC_CODE_OK;
}

ARCCode_t Robot::IK_2(Affine3d p, Array<double, 6, 1>& joint){
    double J1, J2, J3, J4, J5, J6;
    double J1_front, J1_back;
    double J2_up, J2_down;
    double J5_positive, J5_negative;
    double Rwrist11, Rwrist21, Rwrist31, Rwrist12, Rwrist13, Rwrist32, Rwrist33;
    double WPxy, l, h;
    double rho, b4x;
    double alpha, beta, gamma, delta;
    double cos_beta, sin_beta;
    double cos_gamma, sin_gamma;
    Affine3d MP, pJ1, pJ23;
    Vector3d x_hat, WP;
    Matrix3d Rarm, Rwrist;
    FrontBack fb;
    UpDown ud;
    PosNeg pn;

    //Pose of the mounting point (center of the axe 6)
    MP = p;
    /*
    MP = UF * p * UT.inverse();

    switch(this->brand)
    {
        case IR:
            std::cout << "IK IR\n";
            break;
        case ABB:
            std::cout << "IK ABB\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
        case KUKA:
            std::cout << "IK KUKA\n";
            // Mounting point MP (Center of the flange of axis 6)      
            MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
            break;
    }*/

    x_hat = MP.rotation() * Vector3d::UnitX();
    WP = MP.translation() - (this->pLink->get_a6x() * x_hat.normalized());

    // Find J1
    J1 = joint(0);
    // Check if there is a shoulder singularity
    if((abs(WP(0)) < 0.001) && (abs(WP(1)) < 0.001)) {
        // In this case we have a shoulder singularity.
        // Fix J1 as the actual value of J1
        J1 = joint(0);
        fb = Front;
    } else {
        // FRONT solution
        J1_front = atan2(WP(1), WP(0));
        // To have the BACK solution I need to add or substract pi
        // BACK solution is selected
        if(J1_front > 0)
            J1_back = J1_front - M_PI;
        else
            J1_back = J1_front + M_PI;
        
        // Check if the robot is more close
        // to the Front or to the Back solution
        if (abs(J1_front - J1) < abs(J1_back - J1)) {
            // The robot is near to the Front solution
            fb = Front;
            J1 = J1_front;
        } else {
            // The robot is near to the Back solution
            fb = Back;
            J1 = J1_back;
        }
    }

    // Find J2 and J3
    WPxy = sqrt( pow(WP(0),2) + pow(WP(1),2) );

    switch (fb){
        case Front:
            // FRONT solution
            l = WPxy - this->pLink->get_a2x();
            break;
        case Back:
            // BACK solution
            l = WPxy + this->pLink->get_a2x();
            break;
    }
    h = WP(2) - this->pLink->get_a1z() - this->pLink->get_a2z();

    rho = sqrt( pow(h,2) + pow(l,2) );
    b4x = sqrt( pow(this->pLink->get_a4z(),2) + pow(this->pLink->get_a4x()+this->pLink->get_a5x(),2) );
    if(rho > (this->pLink->get_a3z()+b4x)){
        // It is not possible to reach that point
        return ARC_ERR_KIN__POSE_NOT_REACHABLE;
    }
    if(rho < abs(pLink->get_a3z()-b4x)){
        // J2 too close to the robot himself
        return ARC_ERR_APP_J2_TOO_CLOSE;
    }

    alpha = atan2(h, l);
    cos_beta = (pow(rho,2) + pow(this->pLink->get_a3z(),2) - pow(b4x,2)) / (2*rho*this->pLink->get_a3z());
    sin_beta = sqrt(1 - pow(cos_beta,2));
    beta = atan2(sin_beta, cos_beta);

    // Up solution
    J2_up = M_PI_2 - alpha - beta;
    // Down solution
    J2_down = M_PI_2 - alpha + beta;

    // Check if the robot is more close
    // to the Up or to the Down solution
    J2 = joint(1);
    if (abs(J2_up - J2) < abs(J2_down - J2)) {
        // The robot is near to the Up solution
        ud = Up;
        J2 = J2_up;
    } else {
        // The robot is near to the Down solution
        ud = Down;
        J2 = J2_down;
    }

    cos_gamma = (pow(this->pLink->get_a3z(),2) + pow(b4x,2) - pow(rho,2)) / (2*this->pLink->get_a3z()*b4x);
    sin_gamma = sqrt(1 - pow(cos_gamma,2));
    gamma = atan2(sin_gamma, cos_gamma);
    delta = atan2(this->pLink->get_a4x()+this->pLink->get_a5x(), this->pLink->get_a4z());

    J3 = M_PI - gamma - delta;

    // Calculate Rarm from the values of J1, J2, J3
    pJ1 = AngleAxisd(J1, Vector3d::UnitZ());
    pJ23 = AngleAxisd(J2+J3, Vector3d::UnitY());
    Rarm = pJ1.rotation() * pJ23.rotation();
    // R = Rarm * Rwrist -> Rwrist = Rarm^T * R
    Rwrist = Rarm.transpose() * MP.rotation();

    //Find J4, J5, J6 from Rwrist
    Rwrist11 = Rwrist(0,0);
    Rwrist21 = Rwrist(1,0);
    Rwrist31 = Rwrist(2,0);
    Rwrist12 = Rwrist(0,1);
    Rwrist13 = Rwrist(0,2);
    Rwrist32 = Rwrist(2,1);
    Rwrist33 = Rwrist(2,2);

    if (Rwrist11 < 0.9999999) {
        if (Rwrist11 > -0.9999999) {
            // Positive solution
            J5_positive = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
            // Negative solution
            J5_negative = atan2( -sqrt(1-pow(Rwrist11,2)) , Rwrist11 );

            // Check if the robot is more close
            // to the Positive or to the Negative solution
            J5 = joint(4);
            if (abs(J5_positive - J5) < abs(J5_negative - J5)) {
                // The robot is near to the Positive solution
                pn = Positive;
                J5 = J5_positive;
                J4 = atan2(Rwrist21,-Rwrist31);
                J6 = atan2(Rwrist12,Rwrist13);
            } else {
                // The robot is near to the Negative solution
                pn = Negative;
                J5 = J5_negative;
                J4 = atan2(-Rwrist21,Rwrist31);
                J6 = atan2(-Rwrist12,-Rwrist13);
            }
        }
        else // Rwrist11 = −1 
        {
            // Wrist singularity. J5 = 180 -> This condition is not
            // possible because the spherical wrist cannot rotate J5 = 180.
            // Not a unique solution: J6 − J4 = atan2(Rwrist32,Rwrist33)
            J5 = M_PI;
            J4 = joint(3);
            J6 = atan2(Rwrist32,Rwrist33) + J4;
            pn = Positive;
        }
    }
    else // Rwrist11 = +1
    {
        // Wrist singularity. J5 = 0
        // Not a unique solution: J4 + J6 = atan2(Rwrist32,Rwrist33)
        J5 = 0;
        J4 = joint(3);
        J6 = atan2(Rwrist32,Rwrist33) - J4;
        pn = Positive;
    }

    //switch(this->brand)
    //{
    //    case IR:
    //        break;
    //    case ABB:
    //        break;
    //    case KUKA:
    //        J1 = -J1;
    //        J2 -= M_PI_2;
    //        J3 += M_PI_2;
    //        J4 = -J4;
    //        J6 = -J6;
    //        break;
    //}

    joint << J1, J2, J3, J4, J5, J6;
    return ARC_CODE_OK;
}

void Robot::pbFK_released()
{
  // 
  Affine3d pose = this->FK(this->pJoint->get_joints_rad());
  pPose->set_pose(pose);
}

void Robot::pbIK_released()
{
    // 
    FrontBack fb = Front;
    UpDown ud = Up;
    PosNeg pn  = Positive;
    if(this->cbBack->isChecked()) fb = Back;
    if(this->cbDown->isChecked()) ud = Down;
    if(this->cbNegative->isChecked()) pn = Negative;

    Array<double, 6, 1> j = pJoint->get_joints_rad();
    //if(this->IK(pPose->get_pose(), fb, ud, pn, j) == ARC_CODE_OK)
    //    pJoint->set_joints_rad(j);
    if(this->IK_2(pPose->get_pose(), j) == ARC_CODE_OK)
        pJoint->set_joints_rad(j);
}

/*
Array<double, 6, 1> Robot::IK(Affine3d p, Array<double, 6, 1> joint_act){
    double J1, J2, J3, J4, J5, J6;
    double Rwrist11, Rwrist21, Rwrist31, Rwrist12, Rwrist13, Rwrist32, Rwrist33;
    double WPxy, l, h;
    double rho, b4x;
    double alpha, beta, gamma, delta;
    double cos_beta, sin_beta;
    double cos_gamma, sin_gamma;
    Affine3d MP, pJ1, pJ23;
    Vector3d x_hat, WP;
    Matrix3d Rarm, Rwrist;

    Robot::FrontBack FB = Front;
    Robot::UpDown UD = Up;
    Robot::PosNeg PN = Positive;

    //Pose of the mounting point (center of the axe 6)
    MP = p;

    //MP = UF * p * UT.inverse();
    //switch(this->brand)
    //{
    //    case IR:
    //        std::cout << "IK IR\n";
    //        break;
    //    case ABB:
    //        std::cout << "IK ABB\n";
    //        // Mounting point MP (Center of the flange of axis 6)      
    //        MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
    //        break;
    //    case KUKA:
    //        std::cout << "IK KUKA\n";
    //        // Mounting point MP (Center of the flange of axis 6)      
    //        MP.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).inverse());
    //        break;
    //}

    x_hat = MP.rotation() * Vector3d::UnitX();
    WP = MP.translation() - (this->pLink->get_a6x() * x_hat.normalized());

    // Find J1
    // Check if there is a shoulder singularity
    if((abs(WP(0)) < 0.001) && (abs(WP(1)) < 0.001)){
        // In this case we have a shoulder singularity.
        // Fix J1 as the actual value of J1
        J1 = joint_act(0);
    }
    else{
        // FRONT solution
        J1 = atan2(WP(1), WP(0));
        // To have the BACK solution I need to add or substract pi
        if(FB == Back){
            // BACK solution is selected
            if(J1 > 0)
                J1 -= M_PI;
            else
                J1 += M_PI;
        }
    }

    // Find J2 and J3
    WPxy = sqrt( pow(WP(0),2) + pow(WP(1),2) );

    switch (FB)
    {
    case Front:
        // FRONT solution
        l = WPxy - this->pLink->get_a2x();
        break;
    case Back:
        // BACK solution
        l = WPxy + this->pLink->get_a2x();
        break;
    }
    h = WP(2) - this->pLink->get_a1z() - this->pLink->get_a2z();

    rho = sqrt( pow(h,2) + pow(l,2) );
    b4x = sqrt( pow(this->pLink->get_a4z(),2) + pow(this->pLink->get_a4x()+this->pLink->get_a5x(),2) );
    if(rho >= (this->pLink->get_a3z()+b4x)){
        // It is not possible to reach that point
        //std::cout << "Error: impossible to reach that point" << std::endl;
        return joint_act;
    }
    if(rho <= abs(pLink->get_a3z()-b4x)){
        // J2 too close to the robot himself
        //std::cout << "Error: J2 too small" << std::endl;
        return joint_act;
    }

    alpha = atan2(h, l);
    cos_beta = (pow(rho,2) + pow(this->pLink->get_a3z(),2) - pow(b4x,2)) / (2*rho*this->pLink->get_a3z());
    sin_beta = sqrt(1 - pow(cos_beta,2));
    beta = atan2(sin_beta, cos_beta);

    switch (UD)
    {
    case Up:
        // UP solution
        J2 = M_PI_2 - alpha - beta;
        break;
    case Down:
        // DOWN solution
        J2 = M_PI_2 - alpha + beta;
        break;
    }

    cos_gamma = (pow(this->pLink->get_a3z(),2) + pow(b4x,2) - pow(rho,2)) / (2*this->pLink->get_a3z()*b4x);
    sin_gamma = sqrt(1 - pow(cos_gamma,2));
    gamma = atan2(sin_gamma, cos_gamma);
    delta = atan2(this->pLink->get_a4x()+this->pLink->get_a5x(), this->pLink->get_a4z());

    J3 = M_PI - gamma - delta;

    // Calculate Rarm from the values of J1, J2, J3
    pJ1 = AngleAxisd(J1, Vector3d::UnitZ());
    pJ23 = AngleAxisd(J2+J3, Vector3d::UnitY());
    Rarm = pJ1.rotation() * pJ23.rotation();
    // R = Rarm * Rwrist -> Rwrist = Rarm^T * R
    Rwrist = Rarm.transpose() * MP.rotation();

    //Find J4, J5, J6 from Rwrist
    Rwrist11 = Rwrist(0,0);
    Rwrist21 = Rwrist(1,0);
    Rwrist31 = Rwrist(2,0);
    Rwrist12 = Rwrist(0,1);
    Rwrist13 = Rwrist(0,2);
    Rwrist32 = Rwrist(2,1);
    Rwrist33 = Rwrist(2,2);

    if (Rwrist11 < 0.9999999) {
        if (Rwrist11 > -0.9999999) {
            switch (PN)
            {
            case Positive:
                J5 = atan2( sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                J4 = atan2(Rwrist21,-Rwrist31);
                J6 = atan2(Rwrist12,Rwrist13);
                break;
            case Negative:
                J5 = atan2( -sqrt(1-pow(Rwrist11,2)) , Rwrist11 );
                J4 = atan2(-Rwrist21,Rwrist31);
                J6 = atan2(-Rwrist12,-Rwrist13);
                break;
            }
        }
        else // Rwrist11 = −1 
        {
            // Wrist singularity. J5 = 180 -> This condition is not
            // possible because the spherical wrist cannot rotate J5 = 180.
            // Not a unique solution: J6 − J4 = atan2(Rwrist32,Rwrist33)
            J5 = M_PI;
            J4 = joint_act(3);
            J6 = atan2(Rwrist32,Rwrist33) + J4;
        }
    }
    else // Rwrist11 = +1
    {
        // Wrist singularity. J5 = 0
        // Not a unique solution: J4 + J6 = atan2(Rwrist32,Rwrist33)
        J5 = 0;
        J4 = joint_act(3);
        J6 = atan2(Rwrist32,Rwrist33) - J4;
    }

    //switch(this->brand)
    //{
    //    case IR:
    //        break;
    //    case ABB:
    //        break;
    //    case KUKA:
    //        J1 = -J1;
    //        J2 -= M_PI_2;
    //        J3 += M_PI_2;
    //        J4 = -J4;
    //        J6 = -J6;
    //        break;
    //}

    Array<double, 6, 1> j;
    j << J1, J2, J3, J4, J5, J6;
    return j;
}
*/
