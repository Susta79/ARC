#include "pose.h"

Pose::Pose(){
    this->Init();
}

Pose::Pose(Affine3d m){
    this->Init();
    this->set_pose(m);
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

void Pose::Init(){
    // Group Links
    this->dsbX = new QDoubleSpinBox;
    this->dsbX->setRange(-9999.0, 9999.0);
    this->dsbX->setSingleStep(1.0);
    this->dsbX->setSuffix("mm");
    this->dsbX->setValue(0.0);
    this->dsbX->setDecimals(1);

    this->dsbY = new QDoubleSpinBox;
    this->dsbY->setRange(-9999.0, 9999.0);
    this->dsbY->setSingleStep(1.0);
    this->dsbY->setSuffix("mm");
    this->dsbY->setValue(0.0);
    this->dsbY->setDecimals(1);

    this->dsbZ = new QDoubleSpinBox;
    this->dsbZ->setRange(-9999.0, 9999.0);
    this->dsbZ->setSingleStep(1.0);
    this->dsbZ->setSuffix("mm");
    this->dsbZ->setValue(0.0);
    this->dsbZ->setDecimals(1);

    this->dsbA = new QDoubleSpinBox;
    this->dsbA->setRange(-360.0, 360.0);
    this->dsbA->setSingleStep(1.0);
    this->dsbA->setSuffix("°");
    this->dsbA->setValue(0.0);
    this->dsbA->setDecimals(2);

    this->dsbB = new QDoubleSpinBox;
    this->dsbB->setRange(-360.0, 360.0);
    this->dsbB->setSingleStep(1.0);
    this->dsbB->setSuffix("°");
    this->dsbB->setValue(0.0);
    this->dsbB->setDecimals(2);

    this->dsbC = new QDoubleSpinBox;
    this->dsbC->setRange(-360.0, 360.0);
    this->dsbC->setSingleStep(1.0);
    this->dsbC->setSuffix("°");
    this->dsbC->setValue(0.0);
    this->dsbC->setDecimals(2);

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

// pose
Affine3d Pose::get_pose(){
    Affine3d p = Eigen::Affine3d::Identity();
    p.translation() = Eigen::Vector3d(this->dsbX->value(), this->dsbY->value(), this->dsbZ->value());
    Matrix3d rot;
    double sx, cx, sy, cy, sz, cz;
    //rot = AngleAxisd(this->dsbC->value() * M_PI / 180.0, Vector3d::UnitZ())
    //    * AngleAxisd(this->dsbB->value() * M_PI / 180.0, Vector3d::UnitY())
    //    * AngleAxisd(this->dsbA->value() * M_PI / 180.0, Vector3d::UnitX());
    sx = sin(this->dsbA->value() * M_PI / 180.0);
    cx = cos(this->dsbA->value() * M_PI / 180.0);
    sy = sin(this->dsbB->value() * M_PI / 180.0);
    cy = cos(this->dsbB->value() * M_PI / 180.0);
    sz = sin(this->dsbC->value() * M_PI / 180.0);
    cz = cos(this->dsbC->value() * M_PI / 180.0);
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
    //Vector3d ea = m.linear().eulerAngles(0, 1, 2) * 180.0 / M_PI;
    Matrix3d rot;
    rot = m.linear();
    //this->dsbA->setValue(ea(0));
    //this->dsbB->setValue(ea(1));
    //this->dsbC->setValue(ea(2));

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

// x
double Pose::get_x()
{
    return this->dsbX->value();
}
void Pose::set_x(double val)
{
    this->dsbX->setValue(val);
}

// y
double Pose::get_y()
{
    return this->dsbY->value();
}
void Pose::set_y(double val)
{
    this->dsbY->setValue(val);
}

// z
double Pose::get_z()
{
    return this->dsbZ->value();
}
void Pose::set_z(double val)
{
    this->dsbZ->setValue(val);
}

// a
double Pose::get_a_deg()
{
    return this->dsbA->value();
}
void Pose::set_a_deg(double val)
{
    this->dsbA->setValue(val);
}
double Pose::get_a_rad()
{
    return this->dsbA->value() * M_PI / 180.0;
}
void Pose::set_a_rad(double val)
{
    this->dsbA->setValue(val * 180.0 / M_PI);
}

// b
double Pose::get_b_deg()
{
    return this->dsbB->value();
}
void Pose::set_b_deg(double val)
{
    this->dsbB->setValue(val);
}
double Pose::get_b_rad()
{
    return this->dsbB->value() * M_PI / 180.0;
}
void Pose::set_b_rad(double val)
{
    this->dsbB->setValue(val * 180.0 / M_PI);
}

// c
double Pose::get_c_deg()
{
    return this->dsbC->value();
}
void Pose::set_c_deg(double val)
{
    this->dsbC->setValue(val);
}
double Pose::get_c_rad()
{
    return this->dsbC->value() * M_PI / 180.0;
}
void Pose::set_c_rad(double val)
{
    this->dsbC->setValue(val * 180.0 / M_PI);
}
