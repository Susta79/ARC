#include "joint.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>

Joint::Joint(QString n){
    this->name = n;

    // Group Joints
    this->dsbJoint1 = new QDoubleSpinBox;
    this->dsbJoint1->setRange(-360.0, 360.0);
    this->dsbJoint1->setSingleStep(0.01);
    this->dsbJoint1->setSuffix("°");
    this->dsbJoint1->setValue(0.0);

    this->dsbJoint2 = new QDoubleSpinBox;
    this->dsbJoint2->setRange(-360.0, 360.0);
    this->dsbJoint2->setSingleStep(1.0);
    this->dsbJoint2->setSuffix("°");
    this->dsbJoint2->setValue(0.0);

    this->dsbJoint3 = new QDoubleSpinBox;
    this->dsbJoint3->setRange(-360.0, 360.0);
    this->dsbJoint3->setSingleStep(1.0);
    this->dsbJoint3->setSuffix("°");
    this->dsbJoint3->setValue(0.0);

    this->dsbJoint4 = new QDoubleSpinBox;
    this->dsbJoint4->setRange(-360.0, 360.0);
    this->dsbJoint4->setSingleStep(1.0);
    this->dsbJoint4->setSuffix("°");
    this->dsbJoint4->setValue(0.0);

    this->dsbJoint5 = new QDoubleSpinBox;
    this->dsbJoint5->setRange(-360.0, 360.0);
    this->dsbJoint5->setSingleStep(1.0);
    this->dsbJoint5->setSuffix("°");
    this->dsbJoint5->setValue(0.0);

    this->dsbJoint6 = new QDoubleSpinBox;
    this->dsbJoint6->setRange(-360.0, 360.0);
    this->dsbJoint6->setSingleStep(1.0);
    this->dsbJoint6->setSuffix("°");
    this->dsbJoint6->setValue(0.0);

    this->gbJoints = new QGroupBox(this->name);
    QFormLayout *layoutJoints = new QFormLayout;
    layoutJoints->addRow(new QLabel("Joint 1:"), dsbJoint1);
    layoutJoints->addRow(new QLabel("Joint 2:"), dsbJoint2);
    layoutJoints->addRow(new QLabel("Joint 3:"), dsbJoint3);
    layoutJoints->addRow(new QLabel("Joint 4:"), dsbJoint4);
    layoutJoints->addRow(new QLabel("Joint 5:"), dsbJoint5);
    layoutJoints->addRow(new QLabel("Joint 6:"), dsbJoint6);
    this->gbJoints->setLayout(layoutJoints);

    //connect(dsbJoint1, SIGNAL(valueChanged(double)), this, SLOT(dsbJoint1ValueChanged));
}

Joint::~Joint(){
    // Group Joints
    if (dsbJoint1) {
        delete dsbJoint1;
        dsbJoint1 = nullptr;
    }
    if (dsbJoint2) {
        delete dsbJoint2;
        dsbJoint2 = nullptr;
    }
    if (dsbJoint3) {
        delete dsbJoint3;
        dsbJoint3 = nullptr;
    }
    if (dsbJoint4) {
        delete dsbJoint4;
        dsbJoint4 = nullptr;
    }
    if (dsbJoint5) {
        delete dsbJoint5;
        dsbJoint5 = nullptr;
    }
    if (dsbJoint6) {
        delete dsbJoint6;
        dsbJoint6 = nullptr;
    }
    // Group
    if (gbJoints) {
        delete gbJoints;
        gbJoints = nullptr;
    }
}

// All joints in radiant
Array<double, 6, 1> Joint::get_joints_rad()
{
    Array<double, 6, 1> j;
    j(0) = this->dsbJoint1->value() * M_PI / 180.0;
    j(1) = this->dsbJoint2->value() * M_PI / 180.0;
    j(2) = this->dsbJoint3->value() * M_PI / 180.0;
    j(3) = this->dsbJoint4->value() * M_PI / 180.0;
    j(4) = this->dsbJoint5->value() * M_PI / 180.0;
    j(5) = this->dsbJoint6->value() * M_PI / 180.0;
    return j;
}
void Joint::set_joints_rad(Array<double, 6, 1> j)
{
    this->dsbJoint1->setValue(j(0) * 180.0 / M_PI);
    this->dsbJoint2->setValue(j(1) * 180.0 / M_PI);
    this->dsbJoint3->setValue(j(2) * 180.0 / M_PI);
    this->dsbJoint4->setValue(j(3) * 180.0 / M_PI);
    this->dsbJoint5->setValue(j(4) * 180.0 / M_PI);
    this->dsbJoint6->setValue(j(5) * 180.0 / M_PI);
}
Array<double, 6, 1> Joint::get_joints_deg()
{
    Array<double, 6, 1> j;
    j(0) = this->dsbJoint1->value();
    j(1) = this->dsbJoint2->value();
    j(2) = this->dsbJoint3->value();
    j(3) = this->dsbJoint4->value();
    j(4) = this->dsbJoint5->value();
    j(5) = this->dsbJoint6->value();
    return j;
}
void Joint::set_joints_deg(Array<double, 6, 1> j)
{
    this->dsbJoint1->setValue(j(0));
    this->dsbJoint2->setValue(j(1));
    this->dsbJoint3->setValue(j(2));
    this->dsbJoint4->setValue(j(3));
    this->dsbJoint5->setValue(j(4));
    this->dsbJoint6->setValue(j(5));
}

// Joint1
double Joint::get_joint1_rad()
{
    return this->dsbJoint1->value() * M_PI / 180.0;
}
double Joint::get_joint1_deg()
{
    return this->dsbJoint1->value();
}
void Joint::set_joint1_rad(double val)
{
    this->dsbJoint1->setValue(val);
}
void Joint::set_joint1_deg(double val)
{
    this->dsbJoint1->setValue(val);
}

// Joint2
double Joint::get_joint2_rad()
{
    return this->dsbJoint2->value() * M_PI / 180.0;
}
double Joint::get_joint2_deg()
{
    return this->dsbJoint2->value();
}
void Joint::set_joint2_rad(double val)
{
    this->dsbJoint2->setValue(val);
}
void Joint::set_joint2_deg(double val)
{
    this->dsbJoint2->setValue(val);
}

// Joint3
double Joint::get_joint3_rad()
{
    return this->dsbJoint3->value() * M_PI / 180.0;
}
double Joint::get_joint3_deg()
{
    return this->dsbJoint3->value();
}
void Joint::set_joint3_rad(double val)
{
    this->dsbJoint3->setValue(val);
}
void Joint::set_joint3_deg(double val)
{
    this->dsbJoint3->setValue(val);
}

// Joint4
double Joint::get_joint4_rad()
{
    return this->dsbJoint4->value() * M_PI / 180.0;
}
double Joint::get_joint4_deg()
{
    return this->dsbJoint4->value();
}
void Joint::set_joint4_rad(double val)
{
    this->dsbJoint4->setValue(val);
}
void Joint::set_joint4_deg(double val)
{
    this->dsbJoint4->setValue(val);
}

// Joint5
double Joint::get_joint5_rad()
{
    return this->dsbJoint5->value() * M_PI / 180.0;
}
double Joint::get_joint5_deg()
{
    return this->dsbJoint5->value();
}
void Joint::set_joint5_rad(double val)
{
    this->dsbJoint5->setValue(val);
}
void Joint::set_joint5_deg(double val)
{
    this->dsbJoint5->setValue(val);
}

// Joint6
double Joint::get_joint6_rad()
{
    return this->dsbJoint6->value() * M_PI / 180.0;
}
double Joint::get_joint6_deg()
{
    return this->dsbJoint6->value();
}
void Joint::set_joint6_rad(double val)
{
    this->dsbJoint6->setValue(val);
}
void Joint::set_joint6_deg(double val)
{
    this->dsbJoint6->setValue(val);
}

