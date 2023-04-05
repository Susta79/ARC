#ifndef LINK_H
#define LINK_H

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>

class Link
{
private:
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
    QGroupBox *gbLinks;

    Link();
    ~Link();

    // a1z
    double get_a1z();
    void set_a1z(double val);

    // a2x
    double get_a2x();
    void set_a2x(double val);

    // a2z
    double get_a2z();
    void set_a2z(double val);

    // a3z
    double get_a3z();
    void set_a3z(double val);

    // a4x
    double get_a4x();
    void set_a4x(double val);

    // a4z
    double get_a4z();
    void set_a4z(double val);

    // a5x
    double get_a5x();
    void set_a5x(double val);

    // a6x
    double get_a6x();
    void set_a6x(double val);

};

#endif // LINK_H