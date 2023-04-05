#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <QDialog>
#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>

class Kinematic : public QDialog
{
private:
    QLabel *lbTitle;
    // Group Joints
    QDoubleSpinBox *dsbA1z;
    QDoubleSpinBox *dsbA2x;
    QDoubleSpinBox *dsbA2z;
    QDoubleSpinBox *dsbA3z;
    QDoubleSpinBox *dsbA4x;
    QDoubleSpinBox *dsbA4z;
    QDoubleSpinBox *dsbA5x;
    QDoubleSpinBox *dsbA6x;

public:
    explicit Kinematic(QWidget *parent = 0);
    ~Kinematic();
};

#endif // KINEMATIC_H