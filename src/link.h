#ifndef LINK_H
#define LINK_H

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

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
    double get_a1z(){ return this->dsbA1z->value(); }
    void set_a1z(double val){ this->dsbA1z->setValue(val); }

    // a2x
    double get_a2x(){ return this->dsbA2x->value(); }
    void set_a2x(double val){ this->dsbA2x->setValue(val); }

    // a2z
    double get_a2z(){ return this->dsbA2z->value(); }
    void set_a2z(double val){ this->dsbA2z->setValue(val); }

    // a3z
    double get_a3z(){ return this->dsbA3z->value(); }
    void set_a3z(double val){ this->dsbA3z->setValue(val); }

    // a4x
    double get_a4x(){ return this->dsbA4x->value(); }
    void set_a4x(double val){ this->dsbA4x->setValue(val); }

    // a4z
    double get_a4z(){ return this->dsbA4z->value(); }
    void set_a4z(double val){ this->dsbA4z->setValue(val); }

    // a5x
    double get_a5x(){ return this->dsbA5x->value(); }
    void set_a5x(double val){ this->dsbA5x->setValue(val); }

    // a6x
    double get_a6x(){ return this->dsbA6x->value(); }
    void set_a6x(double val){ this->dsbA6x->setValue(val); }

};

#endif // LINK_H