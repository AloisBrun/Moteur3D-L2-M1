/***********************************************
             FICHIER DE TYPES DE BASE

*************************************************/

#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

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


#endif // TYPES_H_INCLUDED
