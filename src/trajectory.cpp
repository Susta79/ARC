#include "trajectory.h"

Trajectory::Trajectory(double Vmax, Path *path){
    this->set_Vmax(Vmax);
    this->path = path;
};

Trajectory::~Trajectory(){
    if (this->path != NULL)
        delete this->path;
    path = NULL;
};

