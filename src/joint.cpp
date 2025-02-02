#include "joint.h"

Joint::Joint(QString n){
    this->name = n;

    // Group Joint1
    this->dsbJoint1 = new QDoubleSpinBox;
    this->dsbJoint1->setRange(-180.0, 180.0);
    this->dsbJoint1->setSingleStep(1.0);
    this->dsbJoint1->setSuffix("°");
    this->dsbJoint1->setValue(0.0);
    this->dsbJoint1->setDecimals(2);

    this->sldJoint1 = new QSlider(Qt::Horizontal);
    this->sldJoint1->setMinimum(-180);
    this->sldJoint1->setMaximum(180);

    QHBoxLayout *hBoxLayout1 = new QHBoxLayout();
    hBoxLayout1->addWidget(this->dsbJoint1);
    hBoxLayout1->addWidget(this->sldJoint1);

    // Group Joint2
    this->dsbJoint2 = new QDoubleSpinBox;
    this->dsbJoint2->setRange(-360.0, 360.0);
    this->dsbJoint2->setSingleStep(1.0);
    this->dsbJoint2->setSuffix("°");
    this->dsbJoint2->setValue(0.0);
    this->dsbJoint2->setDecimals(2);

    this->sldJoint2 = new QSlider(Qt::Horizontal);
    this->sldJoint2->setMinimum(-180);
    this->sldJoint2->setMaximum(180);

    QHBoxLayout *hBoxLayout2 = new QHBoxLayout();
    hBoxLayout2->addWidget(this->dsbJoint2);
    hBoxLayout2->addWidget(this->sldJoint2);

    // Group Joint3
    this->dsbJoint3 = new QDoubleSpinBox;
    this->dsbJoint3->setRange(-360.0, 360.0);
    this->dsbJoint3->setSingleStep(1.0);
    this->dsbJoint3->setSuffix("°");
    this->dsbJoint3->setValue(0.0);
    this->dsbJoint3->setDecimals(2);

    this->sldJoint3 = new QSlider(Qt::Horizontal);
    this->sldJoint3->setMinimum(-180);
    this->sldJoint3->setMaximum(180);

    QHBoxLayout *hBoxLayout3 = new QHBoxLayout();
    hBoxLayout3->addWidget(this->dsbJoint3);
    hBoxLayout3->addWidget(this->sldJoint3);

    // Group Joint4
    this->dsbJoint4 = new QDoubleSpinBox;
    this->dsbJoint4->setRange(-360.0, 360.0);
    this->dsbJoint4->setSingleStep(1.0);
    this->dsbJoint4->setSuffix("°");
    this->dsbJoint4->setValue(0.0);
    this->dsbJoint4->setDecimals(2);

    this->sldJoint4 = new QSlider(Qt::Horizontal);
    this->sldJoint4->setMinimum(-180);
    this->sldJoint4->setMaximum(180);

    QHBoxLayout *hBoxLayout4 = new QHBoxLayout();
    hBoxLayout4->addWidget(this->dsbJoint4);
    hBoxLayout4->addWidget(this->sldJoint4);

    // Group Joint5
    this->dsbJoint5 = new QDoubleSpinBox;
    this->dsbJoint5->setRange(-179.9, 179.9);
    this->dsbJoint5->setSingleStep(1.0);
    this->dsbJoint5->setSuffix("°");
    this->dsbJoint5->setValue(0.0);
    this->dsbJoint5->setDecimals(2);

    this->sldJoint5 = new QSlider(Qt::Horizontal);
    this->sldJoint5->setMinimum(-180);
    this->sldJoint5->setMaximum(180);

    QHBoxLayout *hBoxLayout5 = new QHBoxLayout();
    hBoxLayout5->addWidget(this->dsbJoint5);
    hBoxLayout5->addWidget(this->sldJoint5);

    // Group Joint6
    this->dsbJoint6 = new QDoubleSpinBox;
    this->dsbJoint6->setRange(-360.0, 360.0);
    this->dsbJoint6->setSingleStep(1.0);
    this->dsbJoint6->setSuffix("°");
    this->dsbJoint6->setValue(0.0);
    this->dsbJoint6->setDecimals(2);

    this->sldJoint6 = new QSlider(Qt::Horizontal);
    this->sldJoint6->setMinimum(-180);
    this->sldJoint6->setMaximum(180);

    QHBoxLayout *hBoxLayout6 = new QHBoxLayout();
    hBoxLayout6->addWidget(this->dsbJoint6);
    hBoxLayout6->addWidget(this->sldJoint6);

    // Group Joints
    this->gbJoints = new QGroupBox(this->name);
    QFormLayout *layoutJoints = new QFormLayout;
    layoutJoints->addRow(new QLabel("Joint 1:"), hBoxLayout1);
    layoutJoints->addRow(new QLabel("Joint 2:"), hBoxLayout2);
    layoutJoints->addRow(new QLabel("Joint 3:"), hBoxLayout3);
    layoutJoints->addRow(new QLabel("Joint 4:"), hBoxLayout4);
    layoutJoints->addRow(new QLabel("Joint 5:"), hBoxLayout5);
    layoutJoints->addRow(new QLabel("Joint 6:"), hBoxLayout6);
    this->gbJoints->setLayout(layoutJoints);

    connect(dsbJoint1, &QDoubleSpinBox::valueChanged, sldJoint1, &QSlider::setValue);
    connect(sldJoint1, &QSlider::valueChanged, dsbJoint1, &QDoubleSpinBox::setValue);
    connect(dsbJoint2, &QDoubleSpinBox::valueChanged, sldJoint2, &QSlider::setValue);
    connect(sldJoint2, &QSlider::valueChanged, dsbJoint2, &QDoubleSpinBox::setValue);
    connect(dsbJoint3, &QDoubleSpinBox::valueChanged, sldJoint3, &QSlider::setValue);
    connect(sldJoint3, &QSlider::valueChanged, dsbJoint3, &QDoubleSpinBox::setValue);
    connect(dsbJoint4, &QDoubleSpinBox::valueChanged, sldJoint4, &QSlider::setValue);
    connect(sldJoint4, &QSlider::valueChanged, dsbJoint4, &QDoubleSpinBox::setValue);
    connect(dsbJoint5, &QDoubleSpinBox::valueChanged, sldJoint5, &QSlider::setValue);
    connect(sldJoint5, &QSlider::valueChanged, dsbJoint5, &QDoubleSpinBox::setValue);
    connect(dsbJoint6, &QDoubleSpinBox::valueChanged, sldJoint6, &QSlider::setValue);
    connect(sldJoint6, &QSlider::valueChanged, dsbJoint6, &QDoubleSpinBox::setValue);
}

Joint::~Joint(){
    // Group Joints
    if (dsbJoint1) {
        delete dsbJoint1;
        dsbJoint1 = nullptr;
    }
    if (dsbJoint2) {
        delete dsbJoint2;
        dsbJoint2 = nullptr;
    }
    if (dsbJoint3) {
        delete dsbJoint3;
        dsbJoint3 = nullptr;
    }
    if (dsbJoint4) {
        delete dsbJoint4;
        dsbJoint4 = nullptr;
    }
    if (dsbJoint5) {
        delete dsbJoint5;
        dsbJoint5 = nullptr;
    }
    if (dsbJoint6) {
        delete dsbJoint6;
        dsbJoint6 = nullptr;
    }
    if (sldJoint1) {
        delete sldJoint1;
        sldJoint1 = nullptr;
    }
    // Group
    if (gbJoints) {
        delete gbJoints;
        gbJoints = nullptr;
    }
}
