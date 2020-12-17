#include "types.h"


vertex::vertex(){
    x = y = z = 0;
}

vertex::vertex(float dx, float dy, float dz){
    x = dx;
    y = dy;
    z = dz;
}

void vertex::init(float dx, float dy, float dz){
    x = dx;
    y = dy;
    z = dz;
}


float2::float2(float _x, float _y){
    x = _x;
    y = _y;
}

void float2::init(float _x, float _y){
    x = _x;
    y = _y;
}
