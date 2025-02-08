#include "conf.h"

Conf::Conf()
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

    QVBoxLayout *vboxConf = new QVBoxLayout;
    vboxConf->addWidget(gbFrontBack);
    vboxConf->addWidget(gbUpDown);
    vboxConf->addWidget(gbPosNeg);
    
    this->gbConf = new QGroupBox("Configuration");
    gbConf->setLayout(vboxConf);
}

Conf::~Conf()
{
    if (cbFront) {
        delete cbFront;
        cbFront = nullptr;
    }
    if (cbBack) {
        delete cbBack;
        cbBack = nullptr;
    }
    if (cbUp) {
        delete cbUp;
        cbUp = nullptr;
    }
    if (cbDown) {
        delete cbDown;
        cbDown = nullptr;
    }
    if (cbPositive) {
        delete cbPositive;
        cbPositive = nullptr;
    }
    if (cbNegative) {
        delete cbNegative;
        cbNegative = nullptr;
    }
    if (gbFrontBack) {
        delete gbFrontBack;
        gbFrontBack = nullptr;
    }
    if (gbUpDown) {
        delete gbUpDown;
        gbUpDown = nullptr;
    }
    if (gbPosNeg) {
        delete gbPosNeg;
        gbPosNeg = nullptr;
    }
    if (gbConf) {
        delete gbConf;
        gbConf = nullptr;
    }
}