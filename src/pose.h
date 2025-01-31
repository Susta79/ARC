#ifndef POSE_H
#define POSE_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <Eigen/Geometry>

using namespace Eigen;

class Pose
{
private:
    // The quaternion is not used for the moment
    Quaterniond q = Quaterniond::Identity(); 
    void Init();

protected:
    // Group Pose
    // Translation
    QDoubleSpinBox *dsbX;
    QDoubleSpinBox *dsbY;
    QDoubleSpinBox *dsbZ;
    // Tait-Bryan angles:
    QDoubleSpinBox *dsbA; // A = rotation angle around X
    QDoubleSpinBox *dsbB; // B = rotation angle around Y
    QDoubleSpinBox *dsbC; // C = rotation angle around Z

    QGroupBox *gbPose;

public:
    Pose();
    //Pose(Affine3d);
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
    double get_a_deg();
    double get_a_rad();
    void set_a_deg(double val);
    void set_a_rad(double val);

    // b
    double get_b_deg();
    double get_b_rad();
    void set_b_deg(double val);
    void set_b_rad(double val);

    // c
    double get_c_deg();
    double get_c_rad();
    void set_c_deg(double val);
    void set_c_rad(double val);
};

#endif // POSE_H