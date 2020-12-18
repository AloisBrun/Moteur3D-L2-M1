/***********************************************
       CLASSE REPRESENTANT UN TRIANGLE 3D

*************************************************/

#ifndef FACE_H_INCLUDED
#define FACE_H_INCLUDED

#include "types.h"
#include "camera.h"
#include "const.h"

class face{
private:
    short int tab_nbp[3];
    float2* tab_tex;
    vertex** tab_p;
    vertex* norm = nullptr;

    void clear_face();


public:
    face();

    void updateNormal();

    void init(vertex *vert1, vertex *vert2, vertex *vert3,
              short int v1, short int v2, short int v3,
              float2 tex1, float2 tex2, float2 tex3);

    void init(vertex* t_p[3], short int t_n[3], float2 t_tex[3]);

    vertex GetNorm();

    float2 GetTex(int i);

    vertex GetVertex(int i);

    short int GetNbVertex(int i);

    bool is_visible(camera C);

    ~face();
};

struct faceP{
    std::vector<Vec2> tab_p;
    vertex norm;
};

#endif // FACE_H_INCLUDED
