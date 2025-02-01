#ifndef RPOSE_H
#define RPOSE_H

#include "pose.h"
#include "conf.h"

class RPose
{
private:

    Pose *pPose;
    Conf *pConf;

public:
    QGroupBox *gbRPose;

    RPose();
    ~RPose();

    // pPose
    Pose* get_pPose(){ return this->pPose; }
    void set_pPose(Pose *pPose){ this->pPose = pPose; }

    // pConf
    Conf* get_pConf(){ return this->pConf; }
    void set_pConf(Conf *pConf){ this->pConf = pConf; }
};

#endif // RPOSE_H