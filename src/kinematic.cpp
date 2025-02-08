#include "kinematic.h"

Kinematic::Kinematic(QWidget *parent) : QDialog(parent) {

    this->lbTitle = new QLabel("Label");
    this->pbPushButton = new QPushButton("Button");

    QHBoxLayout *HLayout = new QHBoxLayout(this);
    HLayout->addWidget (lbTitle);
    HLayout->addWidget (pbPushButton);

    this->setLayout (HLayout);

    connect(pbPushButton, &QPushButton::released, this, &Kinematic::pbPushButton_released);
}

Kinematic::~Kinematic() {
    if (lbTitle) {
        delete lbTitle;
        lbTitle = nullptr;
    }
    if (this->pbPushButton) {
        delete this->pbPushButton;
        this->pbPushButton = nullptr;
    }
    if (this->robi) {
        delete this->robi;
        this->robi = nullptr;
    }
}

void Kinematic::pbPushButton_released() {
    this->robi = new Robi();
    this->robi->test();
}
