#include "face.h"


void face::clear_face(){
    if(norm != nullptr)
        delete norm;
    if(tab_tex != nullptr)
        delete tab_tex;
    if(tab_p != nullptr)
        delete [] tab_p; ///on delete seulement les pointeurs sur vertex
}


face::face(){

}

void face::updateNormal(){
    *norm = getNormal(*tab_p[0] - *tab_p[1], *tab_p[2] - *tab_p[1]);
}

void face::init(vertex *vert1, vertex *vert2, vertex *vert3,
          short int v1, short int v2, short int v3,
          float2 tex1, float2 tex2, float2 tex3){
    vertex* t_p[3];
    t_p[0] = vert1;
    t_p[1] = vert2;
    t_p[2] = vert3;

    short int t_n[3];
    t_n[0] = v1;
    t_n[1] = v2;
    t_n[2] = v3;

    float2 t_tex[3];
    t_tex[0] = tex1;
    t_tex[1] = tex2;
    t_tex[2] = tex3;

    init(t_p, t_n, t_tex);
}

void face::init(vertex* t_p[3], short int t_n[3], float2 t_tex[3]){
//        clear_face();

    tab_p = new vertex*[3];
    tab_tex = new float2[3];
    for(int i = 0; i < 3; i++){
        tab_nbp[i] = t_n[i];
        tab_p[i] = t_p[i];
        tab_tex[i] = t_tex[i];
    }

    norm = new vertex;
    updateNormal();
}

vertex face::GetNorm(){
    return *norm;
}

float2 face::GetTex(int i){
    return tab_tex[i];
}

vertex face::GetVertex(int i){
    return *tab_p[i];
}

short int face::GetNbVertex(int i){
    return tab_nbp[i];
}

bool face::is_visible(camera C){
    vertex AC = C.posCam - *tab_p[0],
           AB = *norm;

    float theta = angleEntreVec3(AB, AC);

    if(theta < M_PI/2){
        return true;
    }
    return false;
}

face::~face(){
    ///clear_face();
}
