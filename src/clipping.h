/***********************************************
       FONCTIONS DE CLIPPING DE FRAGMENT

*************************************************/

#ifndef CLIPPING_H_INCLUDED
#define CLIPPING_H_INCLUDED

#include "types.h"
#include "camera.h"
#include "face.h"
#include "const.h"
#include "projection.h"
#include <vector>


Vec2 clip_point(Vec2 p1, Vec2 p2, int border);

int returnZone(Vec2 v);

bool isInMiddleZone(std::vector<int> Vi);

bool isInSameZone(std::vector<int> Vi);

bool clip_face(camera C, face f, std::vector<Vec2> list_coord, std::vector<float> list_angles, faceP &newFace);

#endif // CLIPPING_H_INCLUDED
