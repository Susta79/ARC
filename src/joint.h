#ifndef JOINT_H
#define JOINT_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

#include <Eigen/Dense>

class Joint : public QObject
{
private:
    Q_OBJECT

    QString name;
    // Group Joints
    QDoubleSpinBox *dsbJoint1;
    QDoubleSpinBox *dsbJoint2;
    QDoubleSpinBox *dsbJoint3;
    QDoubleSpinBox *dsbJoint4;
    QDoubleSpinBox *dsbJoint5;
    QDoubleSpinBox *dsbJoint6;

public:
    QGroupBox *gbJoints;

    Joint(QString n);
    ~Joint();

    // All joints in radiant
    Eigen::Array<double, 6, 1> get_joints_rad(){
        Eigen::Array<double, 6, 1> j;
        j(0) = this->dsbJoint1->value() * M_PI / 180.0;
        j(1) = this->dsbJoint2->value() * M_PI / 180.0;
        j(2) = this->dsbJoint3->value() * M_PI / 180.0;
        j(3) = this->dsbJoint4->value() * M_PI / 180.0;
        j(4) = this->dsbJoint5->value() * M_PI / 180.0;
        j(5) = this->dsbJoint6->value() * M_PI / 180.0;
        return j;
    }
    void set_joints_rad(Eigen::Array<double, 6, 1> j){
        this->dsbJoint1->setValue(j(0) * 180.0 / M_PI);
        this->dsbJoint2->setValue(j(1) * 180.0 / M_PI);
        this->dsbJoint3->setValue(j(2) * 180.0 / M_PI);
        this->dsbJoint4->setValue(j(3) * 180.0 / M_PI);
        this->dsbJoint5->setValue(j(4) * 180.0 / M_PI);
        this->dsbJoint6->setValue(j(5) * 180.0 / M_PI);
    }
    // All joints in degree
    Eigen::Array<double, 6, 1> get_joints_deg(){
        Eigen::Array<double, 6, 1> j;
        j(0) = this->dsbJoint1->value();
        j(1) = this->dsbJoint2->value();
        j(2) = this->dsbJoint3->value();
        j(3) = this->dsbJoint4->value();
        j(4) = this->dsbJoint5->value();
        j(5) = this->dsbJoint6->value();
        return j;
    }
    void set_joints_deg(Eigen::Array<double, 6, 1> j){
        this->dsbJoint1->setValue(j(0));
        this->dsbJoint2->setValue(j(1));
        this->dsbJoint3->setValue(j(2));
        this->dsbJoint4->setValue(j(3));
        this->dsbJoint5->setValue(j(4));
        this->dsbJoint6->setValue(j(5));
    }

    // Joint1
    double get_joint1_rad(){ return this->dsbJoint1->value() * M_PI / 180.0; }
    double get_joint1_deg(){ return this->dsbJoint1->value(); }
    void set_joint1_rad(double val){ this->dsbJoint1->setValue(val * 180.0 / M_PI); }
    void set_joint1_deg(double val){ this->dsbJoint1->setValue(val); }

    // Joint2
    double get_joint2_rad(){ return this->dsbJoint2->value() * M_PI / 180.0; }
    double get_joint2_deg(){ return this->dsbJoint2->value(); }
    void set_joint2_rad(double val){ this->dsbJoint2->setValue(val * 180.0 / M_PI); }
    void set_joint2_deg(double val){ this->dsbJoint2->setValue(val); }

    // Joint3
    double get_joint3_rad(){ return this->dsbJoint3->value() * M_PI / 180.0; }
    double get_joint3_deg(){ return this->dsbJoint3->value(); }
    void set_joint3_rad(double val){ this->dsbJoint3->setValue(val * 180.0 / M_PI); }
    void set_joint3_deg(double val){ this->dsbJoint3->setValue(val); }

    // Joint4
    double get_joint4_rad(){ return this->dsbJoint4->value() * M_PI / 180.0; }
    double get_joint4_deg(){ return this->dsbJoint4->value(); }
    void set_joint4_rad(double val){ this->dsbJoint4->setValue(val * 180.0 / M_PI); }
    void set_joint4_deg(double val){ this->dsbJoint4->setValue(val); }

    // Joint5
    double get_joint5_rad(){ return this->dsbJoint5->value() * M_PI / 180.0; }
    double get_joint5_deg(){ return this->dsbJoint5->value(); }
    void set_joint5_rad(double val){ this->dsbJoint5->setValue(val * 180.0 / M_PI); }
    void set_joint5_deg(double val){ this->dsbJoint5->setValue(val); }

    // Joint6
    double get_joint6_rad(){ return this->dsbJoint6->value() * M_PI / 180.0; }
    double get_joint6_deg(){ return this->dsbJoint6->value(); }
    void set_joint6_rad(double val){ this->dsbJoint6->setValue(val * 180.0 / M_PI); }
    void set_joint6_deg(double val){ this->dsbJoint6->setValue(val); }

};

#endif // JOINT_H