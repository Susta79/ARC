#ifndef JOINT_H
#define JOINT_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <Eigen/Dense>

using namespace Eigen;

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

//private slots:
//    void dsbJoint1ValueChanged(double d);

public:
    QGroupBox *gbJoints;

    Joint(QString n);
    ~Joint();

    // 
    Array<double, 6, 1> get_joints_rad();
    void set_joints_rad(Array<double, 6, 1>);
    Array<double, 6, 1> get_joints_deg();
    void set_joints_deg(Array<double, 6, 1>);

    // Joint1
    double get_joint1_rad();
    double get_joint1_deg();
    void set_joint1_rad(double val);
    void set_joint1_deg(double val);

    // Joint2
    double get_joint2_rad();
    double get_joint2_deg();
    void set_joint2_rad(double val);
    void set_joint2_deg(double val);

    // Joint3
    double get_joint3_rad();
    double get_joint3_deg();
    void set_joint3_rad(double val);
    void set_joint3_deg(double val);

    // Joint4
    double get_joint4_rad();
    double get_joint4_deg();
    void set_joint4_rad(double val);
    void set_joint4_deg(double val);

    // Joint5
    double get_joint5_rad();
    double get_joint5_deg();
    void set_joint5_rad(double val);
    void set_joint5_deg(double val);

    // Joint6
    double get_joint6_rad();
    double get_joint6_deg();
    void set_joint6_rad(double val);
    void set_joint6_deg(double val);

};

#endif // JOINT_H