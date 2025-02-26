#include "trajectory.h"

Trajectory::Trajectory(double V, Path *path){
    this->set_V(V);
    this->path = path;
};

Trajectory::~Trajectory(){
    if (this->path != NULL)
        delete this->path;
    path = NULL;
};

