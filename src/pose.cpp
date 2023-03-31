#include "pose.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <Eigen/Dense>

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
    this->dsbA->setDecimals(1);

    this->dsbB = new QDoubleSpinBox;
    this->dsbB->setRange(-360.0, 360.0);
    this->dsbB->setSingleStep(1.0);
    this->dsbB->setSuffix("°");
    this->dsbB->setValue(0.0);
    this->dsbB->setDecimals(1);

    this->dsbC = new QDoubleSpinBox;
    this->dsbC->setRange(-360.0, 360.0);
    this->dsbC->setSingleStep(1.0);
    this->dsbC->setSuffix("°");
    this->dsbC->setValue(0.0);
    this->dsbC->setDecimals(1);

    this->gbPose = new QGroupBox("Pose");
    QFormLayout *layoutJoints = new QFormLayout;
    layoutJoints->addRow(new QLabel("X:"), dsbX);
    layoutJoints->addRow(new QLabel("Y:"), dsbY);
    layoutJoints->addRow(new QLabel("Z:"), dsbZ);
    layoutJoints->addRow(new QLabel("A:"), dsbA);
    layoutJoints->addRow(new QLabel("B:"), dsbB);
    layoutJoints->addRow(new QLabel("C:"), dsbC);
    this->gbPose->setLayout(layoutJoints);
}

// pose
Affine3d Pose::get_pose(){
    Affine3d p = Eigen::Affine3d::Identity();
    p.translation() = Eigen::Vector3d(this->dsbX->value(), this->dsbY->value(), this->dsbZ->value());
    Matrix3d rot;
    rot = AngleAxisd(this->dsbA->value() * M_PI / 180.0, Vector3d::UnitZ())
        * AngleAxisd(this->dsbB->value() * M_PI / 180.0, Vector3d::UnitY())
        * AngleAxisd(this->dsbC->value() * M_PI / 180.0, Vector3d::UnitX());
    p.linear() = rot;
    return p;
}
void Pose::set_pose(Affine3d m){
    Vector3d t = m.translation();
    this->dsbX->setValue(t.x());
    this->dsbY->setValue(t.y());
    this->dsbZ->setValue(t.z());
    Vector3d ea = m.rotation().eulerAngles(2, 1, 0) * 180.0 / M_PI;
    this->dsbA->setValue(ea.z());
    this->dsbB->setValue(ea.y());
    this->dsbC->setValue(ea.x());
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
double Pose::get_a()
{
    return this->dsbA->value();
}
void Pose::set_a(double val)
{
    this->dsbA->setValue(val);
}

// b
double Pose::get_b()
{
    return this->dsbB->value();
}
void Pose::set_b(double val)
{
    this->dsbB->setValue(val);
}

// c
double Pose::get_c()
{
    return this->dsbC->value();
}
void Pose::set_c(double val)
{
    this->dsbC->setValue(val);
}
