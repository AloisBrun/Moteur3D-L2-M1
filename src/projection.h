/***********************************************
        FONCTIONS DE PROJECTION DES POINTS

*************************************************/

#ifndef PROJECTION_H_INCLUDED
#define PROJECTION_H_INCLUDED

#include "types.h"
#include "camera.h"
#include "const.h"

Vec2 projette(vertex p, camera C);

Vec2 projetteNeg(vertex p1, vertex p2, Vec2 v, Vec2 v2, camera C);

void demi_projette(vertex &dif, vertex p2, Vec2 &v2, Vec2 &v, float2 f, camera C);


#endif // PROJECTION_H_INCLUDED
