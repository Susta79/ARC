#ifndef ROBOT_H
#define ROBOT_H

//#include <string>
#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>

#include <Eigen/Dense>

#include "joint.h"
#include "link.h"
#include "pose.h"
#include <ARC/error_def.h>

using namespace Eigen;

class Robot : public QObject
{
private:
    Q_OBJECT
    
    QString name;
    QPushButton *pbFK;
    QPushButton *pbIK;
    QGroupBox *gbFrontBack;
    QRadioButton *cbFront;
    QRadioButton *cbBack;
    QGroupBox *gbUpDown;
    QRadioButton *cbUp;
    QRadioButton *cbDown;
    QGroupBox *gbPosNeg;
    QRadioButton *cbPositive;
    QRadioButton *cbNegative;

    Link* pLink;
    Pose* pPose;
    //Brand brand;
    //string model;
    //string brand_string();

private slots:
    void pbFK_released();
    void pbIK_released();

public:
    enum FrontBack { Front, Back };
    Q_ENUM(FrontBack)

    enum UpDown { Up, Down };
    Q_ENUM(UpDown)

    enum PosNeg { Positive, Negative };
    Q_ENUM(PosNeg)

    Joint* pJoint;
    Joint* pRealJoint;
    QGroupBox *gbGroup;

    Robot(QString name);
    //Robot(Brand b, string model);
    ~Robot();
    //void setdimensions(double a1z, double a2x, double a2z, double a3z, double a4x, double a4z, double a5x, double a6x);
    //void setdimensionsIR(void);
    //void setdimensionsABB(void);
    //void setdimensionsKUKA(void);
    //void printAffine3d(Affine3d p);
    Affine3d FK(Array<double, 6, 1>);
    // IK: return the joint value 
    //Array<double, 6, 1> IK(Affine3d p, Array<double, 6, 1> joint_act);
    // Inverse kinematics IK: from pose to joint values.
    // paramenters:
    //      pose: desired robot pose
    //      joint: set the actual joint values and get the new values as output
    // 
    ARCCode_t IK(Affine3d p, Robot::FrontBack fb, Robot::UpDown ud, Robot::PosNeg pn, Array<double, 6, 1>& joint);
    ARCCode_t IK_2(Affine3d p, Array<double, 6, 1>& joint);
    //Joint IK(Affine3d p, Affine3d UT, Affine3d UF, Joint jAct, FrontBack FB, UpDown UD, PositiveNegative PN);
};

#endif // ROBOT_H