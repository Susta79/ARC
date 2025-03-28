#include "pose.h"

Pose::Pose(){
    // Group Links
    this->dsbX = new QDoubleSpinBox;
    this->dsbX->setRange(-9999.0, 9999.0);
    this->dsbX->setSingleStep(1.0);
    this->dsbX->setSuffix("mm");
    this->dsbX->setValue(0.0);
    this->dsbX->setDecimals(2);

    this->dsbY = new QDoubleSpinBox;
    this->dsbY->setRange(-9999.0, 9999.0);
    this->dsbY->setSingleStep(1.0);
    this->dsbY->setSuffix("mm");
    this->dsbY->setValue(0.0);
    this->dsbY->setDecimals(2);

    this->dsbZ = new QDoubleSpinBox;
    this->dsbZ->setRange(-9999.0, 9999.0);
    this->dsbZ->setSingleStep(1.0);
    this->dsbZ->setSuffix("mm");
    this->dsbZ->setValue(0.0);
    this->dsbZ->setDecimals(2);

    this->dsbA = new QDoubleSpinBox;
    this->dsbA->setRange(-360.0, 360.0);
    this->dsbA->setSingleStep(1.0);
    this->dsbA->setSuffix("°");
    this->dsbA->setValue(0.0);
    this->dsbA->setDecimals(3);

    this->dsbB = new QDoubleSpinBox;
    this->dsbB->setRange(-360.0, 360.0);
    this->dsbB->setSingleStep(1.0);
    this->dsbB->setSuffix("°");
    this->dsbB->setValue(0.0);
    this->dsbB->setDecimals(3);

    this->dsbC = new QDoubleSpinBox;
    this->dsbC->setRange(-360.0, 360.0);
    this->dsbC->setSingleStep(1.0);
    this->dsbC->setSuffix("°");
    this->dsbC->setValue(0.0);
    this->dsbC->setDecimals(3);

    this->gbPose = new QGroupBox("Pose");
    QFormLayout *layoutJoints = new QFormLayout;
    layoutJoints->addRow(new QLabel("X:"), dsbX);
    layoutJoints->addRow(new QLabel("Y:"), dsbY);
    layoutJoints->addRow(new QLabel("Z:"), dsbZ);
    layoutJoints->addRow(new QLabel("A(RX):"), dsbA);
    layoutJoints->addRow(new QLabel("B(RY):"), dsbB);
    layoutJoints->addRow(new QLabel("C(RZ):"), dsbC);
    this->gbPose->setLayout(layoutJoints);
}

Pose::~Pose(){
    if (dsbX) {
        delete dsbX;
        dsbX = nullptr;
    }
    if (dsbY) {
        delete dsbY;
        dsbY = nullptr;
    }
    if (dsbZ) {
        delete dsbZ;
        dsbZ = nullptr;
    }
    if (dsbA) {
        delete dsbA;
        dsbA = nullptr;
    }
    if (dsbB) {
        delete dsbB;
        dsbB = nullptr;
    }
    if (dsbC) {
        delete dsbC;
        dsbC = nullptr;
    }
    // Group
    if (gbPose) {
        delete gbPose;
        gbPose = nullptr;
    }
}

// pose
Affine3d Pose::get_pose(){
    Affine3d p = Eigen::Affine3d::Identity();
    p.translation() = Eigen::Vector3d(this->dsbX->value(), this->dsbY->value(), this->dsbZ->value());
    double sx = sin(this->dsbA->value() * M_PI / 180.0);
    double cx = cos(this->dsbA->value() * M_PI / 180.0);
    double sy = sin(this->dsbB->value() * M_PI / 180.0);
    double cy = cos(this->dsbB->value() * M_PI / 180.0);
    double sz = sin(this->dsbC->value() * M_PI / 180.0);
    double cz = cos(this->dsbC->value() * M_PI / 180.0);
    Matrix3d rot;
    rot(0,0) = cz*cy;
    rot(1,0) = sz*cy;
    rot(2,0) = -sy;
    rot(0,1) = cz*sy*sx-sz*cx;
    rot(1,1) = cz*cx+sz*sy*sx;
    rot(2,1) = cy*sx;
    rot(0,2) = sz*sx+cz*sy*cx;
    rot(1,2) = sz*sy*cx-cz*sx;
    rot(2,2) = cy*cx;
    p.linear() = rot;
    return p;
}

void Pose::set_pose(Affine3d m){
    double A1, A2, B1, B2, C1, C2;
    Vector3d t = m.translation();
    this->dsbX->setValue(t.x());
    this->dsbY->setValue(t.y());
    this->dsbZ->setValue(t.z());
    Matrix3d rot;
    rot = m.linear();
    // R=Rz(C)*Ry(B)*Rx(A)
    // if cos(B) <> to 1 and -1
    if (rot(2,0)<0.999 || rot(2,0)>-0.999) {
        B1 = atan2(-rot(2,0),sqrt(1-pow(rot(2,0),2)));
        B2 = atan2(-rot(2,0),-sqrt(1-pow(rot(2,0),2)));
        A1 = atan2(rot(2,1)/cos(B1), rot(2,2)/cos(B1));
        A2 = atan2(rot(2,1)/cos(B2), rot(2,2)/cos(B2));
        C1 = atan2(rot(1,0)/cos(B1), rot(0,0)/cos(B1));
        C2 = atan2(rot(1,0)/cos(B2), rot(0,0)/cos(B2));
    } else {
        C1 = 0;
        A2 = 0;
        B2 = 0;
        C2 = 0;
        if (rot(2,0) == -1) {
            B1 = M_PI_2;
            A1 = atan2(rot(0,1),rot(0,2));
        } else {
            B1 = -M_PI_2;
            A1 = atan2(-rot(0,1),-rot(0,2));
        }
    }
    // 
    if (A1 >= 0 || A1 < M_PI) {
        this->dsbA->setValue(A1 * 180.0 / M_PI);
        this->dsbB->setValue(B1 * 180.0 / M_PI);
        this->dsbC->setValue(C1 * 180.0 / M_PI);
    } else {
        this->dsbA->setValue(A2 * 180.0 / M_PI);
        this->dsbB->setValue(B2 * 180.0 / M_PI);
        this->dsbC->setValue(C2 * 180.0 / M_PI);
    }
    //qDebug() << "A1: " << A1 << "; A2: " << A2;
    //qDebug() << "B1: " << B1 << "; B2: " << B2;
    //qDebug() << "C1: " << C1 << "; C2: " << C2;
    //qDebug() << "";
}