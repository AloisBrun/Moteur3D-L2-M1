#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include <unistd.h>

class vertex{
public:
    float x, y, z;

    vertex();

    vertex(float dx, float dy, float dz);

    void init(float dx, float dy, float dz);

    ~vertex(){}
};

struct Vec2{
    int x,y;
    float z,
          tx, ty;
};

class float2{
    public:
    float x, y;

    float2(){}

    float2(float _x, float _y);

    void init(float _x, float _y);
};

struct pixel_profondeur{
    uint32_t rgba;
    unsigned int z;
};

vertex operator+(vertex P1, vertex P2);

vertex operator-(vertex P1, vertex P2);

vertex operator-(vertex P1);

vertex operator*(vertex P1, float a);

vertex operator*(float a, vertex P1);

vertex operator/(vertex P1, float a);

Vec2 operator+(Vec2 P1, Vec2 P2);

Vec2 operator-(Vec2 P1, Vec2 P2);

Vec2 operator-(Vec2 P1);

Vec2 operator*(float a, Vec2 P2);

Vec2 operator*(Vec2 P2, float a);

#endif // TYPES_H_INCLUDED
