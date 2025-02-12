#include "trajectory.h"

Trajectory::~Trajectory(){
    if (this->path != NULL)
        delete this->path;
    path = NULL;
};