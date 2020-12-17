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

vertex operator+(vertex P1, vertex P2){
    vertex P;
    P.x = P1.x + P2.x;
    P.y = P1.y + P2.y;
    P.z = P1.z + P2.z;
    return P;
}

vertex operator-(vertex P1, vertex P2){
    vertex P;
    P.x = P1.x - P2.x;
    P.y = P1.y - P2.y;
    P.z = P1.z - P2.z;
    return P;
}

vertex operator-(vertex P1){
    vertex P;
    P.x = -P1.x;
    P.y = -P1.y;
    P.z = -P1.z;
    return P;
}

vertex operator*(vertex P1, float a){
    vertex P;
    P.x = P1.x * a;
    P.y = P1.y * a;
    P.z = P1.z * a;
    return P;
}

vertex operator*(float a, vertex P1){
    vertex P;
    P.x = P1.x * a;
    P.y = P1.y * a;
    P.z = P1.z * a;
    return P;
}

vertex operator/(vertex P1, float a){
    vertex P;
    P.x = P1.x / a;
    P.y = P1.y / a;
    P.z = P1.z / a;
    return P;
}

Vec2 operator+(Vec2 P1, Vec2 P2){
    Vec2 P;
    P.x = P1.x + P2.x;
    P.y = P1.y + P2.y;
    return P;
}

Vec2 operator-(Vec2 P1, Vec2 P2){
    Vec2 P;
    P.x = P1.x - P2.x;
    P.y = P1.y - P2.y;
    P.z = P1.z - P2.z;

    P.tx = P1.tx - P2.tx;
    P.ty = P1.ty - P2.ty;
    return P;
}

Vec2 operator-(Vec2 P1){
    Vec2 P;
    P.x = -P1.x;
    P.y = -P1.y;
    P.z = -P1.z;
    return P;
}

Vec2 operator*(float a, Vec2 P2){
    Vec2 P;
    P.x = a * P2.x;
    P.y = a * P2.y;
    return P;
}

Vec2 operator*(Vec2 P2, float a){
    Vec2 P;
    P.x = a * P2.x;
    P.y = a * P2.y;
    return P;
}
