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
    // Group Pose
    // Translation
    QDoubleSpinBox *dsbX;
    QDoubleSpinBox *dsbY;
    QDoubleSpinBox *dsbZ;
    // Tait-Bryan angles:
    QDoubleSpinBox *dsbA; // A = rotation angle around X
    QDoubleSpinBox *dsbB; // B = rotation angle around Y
    QDoubleSpinBox *dsbC; // C = rotation angle around Z

public:
    QGroupBox *gbPose;

    Pose();
    //Pose(Affine3d);
    ~Pose();

    // pose
    Affine3d get_pose();
    void set_pose(Affine3d);

    // x
    double get_x(){ return this->dsbX->value(); }
    void set_x(double val){ this->dsbX->setValue(val); }

    // y
    double get_y(){ return this->dsbY->value(); }
    void set_y(double val){ this->dsbY->setValue(val); }

    // z
    double get_z(){ return this->dsbZ->value(); }
    void set_z(double val){ this->dsbZ->setValue(val); }

    // a
    double get_a_deg(){ return this->dsbA->value(); }
    void set_a_deg(double val){ this->dsbA->setValue(val); }
    double get_a_rad(){ return this->dsbA->value() * M_PI / 180.0; }
    void set_a_rad(double val){ this->dsbA->setValue(val * 180.0 / M_PI); }

    // b
    double get_b_deg(){ return this->dsbB->value(); }
    void set_b_deg(double val){ this->dsbB->setValue(val); }
    double get_b_rad(){ return this->dsbB->value() * M_PI / 180.0; }
    void set_b_rad(double val){ this->dsbB->setValue(val * 180.0 / M_PI); }

    // c
    double get_c_deg(){ return this->dsbC->value(); }
    void set_c_deg(double val){ this->dsbC->setValue(val); }
    double get_c_rad(){ return this->dsbC->value() * M_PI / 180.0; }
    void set_c_rad(double val){ this->dsbC->setValue(val * 180.0 / M_PI); }

};

#endif // POSE_H