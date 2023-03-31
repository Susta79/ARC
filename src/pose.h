#ifndef POSE_H
#define POSE_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <Eigen\Geometry>

using namespace Eigen;

class Pose
{
private:
    Quaterniond q = Quaterniond::Identity(); 
    // Group Pose
    QDoubleSpinBox *dsbX;
    QDoubleSpinBox *dsbY;
    QDoubleSpinBox *dsbZ;
    QDoubleSpinBox *dsbA;
    QDoubleSpinBox *dsbB;
    QDoubleSpinBox *dsbC;

    void Init();

public:
    QGroupBox *gbPose;

    Pose();
    Pose(Affine3d);
    ~Pose();

    // pose
    Affine3d get_pose();
    void set_pose(Affine3d);

    // x
    double get_x();
    void set_x(double val);

    // y
    double get_y();
    void set_y(double val);

    // z
    double get_z();
    void set_z(double val);

    // a
    double get_a();
    void set_a(double val);

    // b
    double get_b();
    void set_b(double val);

    // c
    double get_c();
    void set_c(double val);

};

#endif // POSE_H