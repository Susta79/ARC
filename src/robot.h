#ifndef ROBOT_H
#define ROBOT_H

#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>

#include <Eigen/Dense>

#include "joint.h"
#include "link.h"
#include "rpose.h"
#include <include/ARC/error_def.h>

using namespace Eigen;

class Robot : public QObject
{
private:
    Q_OBJECT
    
    QString name;
    QPushButton *pbFK;
    QPushButton *pbIK;

    Link* pLink;
    Joint* pJoint;
    RPose* pRPose;

private slots:
    void pbFK_released();
    void pbIK_released();

public:
    QGroupBox *gbGroup;
    Robot(QString name);
    ~Robot();
    // FK: Forward Kinematic
    void FK();
    // Inverse kinematics IK: from pose to joint values.
    ARCCode_t IK();
    // det_conf: determine configuration.
    // Based on the values of pJoint, determine the
    // robot configuration (Front/Back, Up/down, Positive/Negative)
    // and write it in pRPose
    void det_conf();
};

#endif // ROBOT_H