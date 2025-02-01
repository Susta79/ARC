#ifndef RPOSE_H
#define RPOSE_H

#include <QRadioButton>
#include "pose.h"
//#define _USE_MATH_DEFINES
//#include <math.h>

class RPose : public Pose
{
private:
    QGroupBox *gbFrontBack;
    QRadioButton *cbFront;
    QRadioButton *cbBack;
    QGroupBox *gbUpDown;
    QRadioButton *cbUp;
    QRadioButton *cbDown;
    QGroupBox *gbPosNeg;
    QRadioButton *cbPositive;
    QRadioButton *cbNegative;

public:
    QGroupBox *gbRPose;

    RPose();
    ~RPose();

    bool get_front();
    void set_front();

    bool get_back();
    void set_back();

    bool get_up();
    void set_up();

    bool get_down();
    void set_down();

    bool get_positive();
    void set_positive();

    bool get_negative();
    void set_negative();
};

#endif // RPOSE_H