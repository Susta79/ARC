#include "rpose.h"

RPose::RPose()
{
    this->pPose = new Pose();
    this->pConf = new Conf();

    QVBoxLayout *layoutGroup = new QVBoxLayout();
    layoutGroup->addWidget(this->pPose->gbPose);
    layoutGroup->addWidget(this->pConf->gbConf);

    this->gbRPose = new QGroupBox("RPose");
    gbRPose->setLayout(layoutGroup);
}

RPose::~RPose()
{
    if (pPose) {
        delete pPose;
        pPose = nullptr;
    }
    if (pConf) {
        delete pConf;
        pConf = nullptr;
    }
}
