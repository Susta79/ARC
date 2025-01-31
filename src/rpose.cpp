#include "rpose.h"

RPose::RPose() : Pose()
{
    // Group Front/Back
    this->gbFrontBack = new QGroupBox();
    this->cbFront = new QRadioButton("&Front");
    this->cbBack = new QRadioButton("&Back");
    cbFront->setChecked(true);

    QHBoxLayout *hboxFrontBack = new QHBoxLayout;
    hboxFrontBack->addWidget(cbFront);
    hboxFrontBack->addWidget(cbBack);
    hboxFrontBack->addStretch(1);
    gbFrontBack->setLayout(hboxFrontBack);

    // Group Up/Down
    this->gbUpDown = new QGroupBox();
    this->cbUp = new QRadioButton("&Up");
    this->cbDown = new QRadioButton("&Down");
    cbUp->setChecked(true);

    QHBoxLayout *hboxUpDown = new QHBoxLayout;
    hboxUpDown->addWidget(cbUp);
    hboxUpDown->addWidget(cbDown);
    hboxUpDown->addStretch(1);
    gbUpDown->setLayout(hboxUpDown);

    // Group Positive/Negative
    this->gbPosNeg = new QGroupBox();
    this->cbPositive = new QRadioButton("&Pos");
    this->cbNegative = new QRadioButton("&Neg");
    cbPositive->setChecked(true);

    QHBoxLayout *hboxPosNeg = new QHBoxLayout;
    hboxPosNeg->addWidget(cbPositive);
    hboxPosNeg->addWidget(cbNegative);
    hboxPosNeg->addStretch(1);
    gbPosNeg->setLayout(hboxPosNeg);

    QGroupBox *gbConf = new QGroupBox("Configuration");
    QVBoxLayout *vboxConf = new QVBoxLayout;
    vboxConf->addWidget(gbFrontBack);
    vboxConf->addWidget(gbUpDown);
    vboxConf->addWidget(gbPosNeg);
    gbConf->setLayout(vboxConf);

    QVBoxLayout *layoutGroup = new QVBoxLayout();
    layoutGroup->addWidget(this->gbPose);
    layoutGroup->addWidget(gbConf);

    this->gbRPose = new QGroupBox("RPose");
    gbRPose->setLayout(layoutGroup);
}

RPose::~RPose()
{
    if (gbFrontBack) {
        delete gbFrontBack;
        gbFrontBack = nullptr;
    }
    if (cbFront) {
        delete cbFront;
        cbFront = nullptr;
    }
    if (cbBack) {
        delete cbBack;
        cbBack = nullptr;
    }
    if (gbUpDown) {
        delete gbUpDown;
        gbUpDown = nullptr;
    }
    if (cbUp) {
        delete cbUp;
        cbUp = nullptr;
    }
    if (cbDown) {
        delete cbDown;
        cbDown = nullptr;
    }
    if (gbPosNeg) {
        delete gbPosNeg;
        gbPosNeg = nullptr;
    }
    if (cbPositive) {
        delete cbPositive;
        cbPositive = nullptr;
    }
    if (cbNegative) {
        delete cbNegative;
        cbNegative = nullptr;
    }
}

// Front
bool RPose::get_front()
{
    return this->cbFront->isChecked();
}
void RPose::set_front(bool val)
{
    this->cbFront->setChecked(val);
}

// Back
bool RPose::get_back()
{
    return this->cbBack->isChecked();
}
void RPose::set_back(bool val)
{
    this->cbBack->setChecked(val);
}

// Up
bool RPose::get_up()
{
    return this->cbUp->isChecked();
}
void RPose::set_up(bool val)
{
    this->cbUp->setChecked(val);
}

// Down
bool RPose::get_down()
{
    return this->cbDown->isChecked();
}
void RPose::set_down(bool val)
{
    this->cbDown->setChecked(val);
}

// Positive
bool RPose::get_positive()
{
    return this->cbPositive->isChecked();
}
void RPose::set_positive(bool val)
{
    this->cbPositive->setChecked(val);
}

// Negative
bool RPose::get_negative()
{
    return this->cbNegative->isChecked();
}
void RPose::set_negative(bool val)
{
    this->cbNegative->setChecked(val);
}

