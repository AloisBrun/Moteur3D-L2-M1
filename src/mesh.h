/*****************
******************************
        CLASSE REPRESENTANT UN OBJET 3D

*************************************************/

#ifndef MESH_H_INCLUDED
#define MESH_H_INCLUDED

#include "types.h"
#include "face.h"
#include "myTexture.h"
#include "light.h"
#include "projection.h"
#include "clipping.h"
#include "rasterizer.h"
#include <vector>

static std::vector<vertex> List_vert;
static std::vector<face> List_face;
static int point_chain = 0;

class Mesh{
private:
    int nb_p = 0,
        nb_f = 0;
    vertex centre;
    vertex *tab_p;
    face *tab_f;

    myTexture *texture;

public:
    Mesh();

    Mesh(myTexture *tex, vertex c);

    int get_nb_p();

    int get_nb_f();

    void init(myTexture *t_c, vertex c);

    void draw(uint32_t **print_ref, pixel_profondeur ***z_ref, light L, camera C);

    void finish();
/*
    void point(float _x, float _y, float _z);

    void point(vertex ver);

    void break_point();
    */

    void point(float x, float y, float z);

    void point(vertex v);

    void triangle(short int v1, short int v2, short int v3, float2 tex1, float2 tex2, float2 tex3);

    void triangle(face f);

    ~Mesh();
};

#endif // MESH_H_INCLUDED
