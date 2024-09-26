
#ifndef COMMAND_H_
#define COMMAND_H_

#include "common.h"


template<int Dim>
struct Command
{
    Vecf<Dim> pos;
    Vecf<Dim> vel;
    Vecf<Dim> acc;
    Vecf<Dim> jrk;
    double yaw;
    double yaw_dot;
    double t;
};

typedef Command<2> Command2d;
typedef Command<3> Command3d;

#endif
