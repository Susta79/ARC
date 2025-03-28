#include "link.h"

Link::Link(){
    // Group Links
    this->dsbA1z = new QDoubleSpinBox;
    this->dsbA1z->setRange(-2000.0, 2000.0);
    this->dsbA1z->setSingleStep(1.0);
    this->dsbA1z->setSuffix("mm");
    this->dsbA1z->setValue(650.0);

    this->dsbA2x = new QDoubleSpinBox;
    this->dsbA2x->setRange(-2000.0, 2000.0);
    this->dsbA2x->setSingleStep(1.0);
    this->dsbA2x->setSuffix("mm");
    this->dsbA2x->setValue(400.0);

    this->dsbA2z = new QDoubleSpinBox;
    this->dsbA2z->setRange(-2000.0, 2000.0);
    this->dsbA2z->setSingleStep(1.0);
    this->dsbA2z->setSuffix("mm");
    this->dsbA2z->setValue(680.0);

    this->dsbA3z = new QDoubleSpinBox;
    this->dsbA3z->setRange(-2000.0, 2000.0);
    this->dsbA3z->setSingleStep(1.0);
    this->dsbA3z->setSuffix("mm");
    this->dsbA3z->setValue(1100.0);

    this->dsbA4x = new QDoubleSpinBox;
    this->dsbA4x->setRange(-2000.0, 2000.0);
    this->dsbA4x->setSingleStep(1.0);
    this->dsbA4x->setSuffix("mm");
    this->dsbA4x->setValue(766.0);

    this->dsbA4z = new QDoubleSpinBox;
    this->dsbA4z->setRange(-2000.0, 2000.0);
    this->dsbA4z->setSingleStep(1.0);
    this->dsbA4z->setSuffix("mm");
    this->dsbA4z->setValue(230.0);

    this->dsbA5x = new QDoubleSpinBox;
    this->dsbA5x->setRange(-2000.0, 2000.0);
    this->dsbA5x->setSingleStep(1.0);
    this->dsbA5x->setSuffix("mm");
    this->dsbA5x->setValue(345.0);

    this->dsbA6x = new QDoubleSpinBox;
    this->dsbA6x->setRange(-2000.0, 2000.0);
    this->dsbA6x->setSingleStep(1.0);
    this->dsbA6x->setSuffix("mm");
    this->dsbA6x->setValue(244.0);

    this->gbLinks = new QGroupBox("Links");
    QFormLayout *layoutJoints = new QFormLayout;
    layoutJoints->addRow(new QLabel("A1z:"), dsbA1z);
    layoutJoints->addRow(new QLabel("A2x:"), dsbA2x);
    layoutJoints->addRow(new QLabel("A2z:"), dsbA2z);
    layoutJoints->addRow(new QLabel("A3z:"), dsbA3z);
    layoutJoints->addRow(new QLabel("A4x:"), dsbA4x);
    layoutJoints->addRow(new QLabel("A4z:"), dsbA4z);
    layoutJoints->addRow(new QLabel("A5x:"), dsbA5x);
    layoutJoints->addRow(new QLabel("A6x:"), dsbA6x);
    this->gbLinks->setLayout(layoutJoints);
}

Link::~Link(){
    if (dsbA1z) {
        delete dsbA1z;
        dsbA1z = nullptr;
    }
    if (dsbA2x) {
        delete dsbA2x;
        dsbA2x = nullptr;
    }
    if (dsbA2z) {
        delete dsbA2z;
        dsbA2z = nullptr;
    }
    if (dsbA3z) {
        delete dsbA3z;
        dsbA3z = nullptr;
    }
    if (dsbA4x) {
        delete dsbA4x;
        dsbA4x = nullptr;
    }
    if (dsbA4z) {
        delete dsbA4z;
        dsbA4z = nullptr;
    }
    if (dsbA5x) {
        delete dsbA5x;
        dsbA5x = nullptr;
    }
    if (dsbA6x) {
        delete dsbA6x;
        dsbA6x = nullptr;
    }
    // Group
    if (gbLinks) {
        delete gbLinks;
        gbLinks = nullptr;
    }
}