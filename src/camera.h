/***********************************************
               CLASSE CAMERA

*************************************************/

#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

#include "types.h"
#include "function_global.h"
#include <math.h>

class camera{
public:
    vertex line_of_sight, sight_lim,
           posCam, angleCam,
           OX, OY;

    float Cx = 1, Cy = 1, Cz = 1,
          Sx = 0, Sy = 0, Sz = 0;

    void update();
};

#endif // CAMERA_INCLUDED
