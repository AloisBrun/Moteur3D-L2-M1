/***********************************************
   FONCTIONS DE RASTERIZATIONS DES FRAGMENTS

*************************************************/

#ifndef RASTERIZER_H_INCLUDED
#define RASTERIZER_H_INCLUDED

#include "types.h"
#include "light.h"
#include "camera.h"
#include "myTexture.h"
#include "face.h"
#include <vector>

void interpolateY(std::vector<Vec2> &V, Vec2 S);

bool isFaceNull(std::vector<Vec2> v);

void draw_face(uint32_t **print_ref, pixel_profondeur ***z_ref, light L, camera C, myTexture *texture, faceP newFace);

#endif // RASTERIZER_H_INCLUDED
