#include "kinematic.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>

Kinematic::Kinematic(QWidget *parent) : QDialog(parent) {

    this->lbTitle = new QLabel("Label");

    QHBoxLayout *HLayout = new QHBoxLayout(this);
    HLayout->addWidget (lbTitle);

    this->setLayout (HLayout);
}

Kinematic::~Kinematic() {
    if (lbTitle) {
        delete lbTitle;
        lbTitle = nullptr;
    }
}
