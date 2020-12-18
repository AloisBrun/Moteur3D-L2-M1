/***********************************************
             CLASSE D'ECLAIRAGE

*************************************************/

#ifndef LIGHT_H_INCLUDED
#define LIGHT_H_INCLUDED

#include "types.h"

struct light{
    vertex Direction,
           pos;
    float intensite;

    void init(vertex p, vertex d, float intens);
};

#endif // LIGHT_H_INCLUDED
