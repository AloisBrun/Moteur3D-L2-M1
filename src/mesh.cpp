#include "mesh.h"


Mesh::Mesh(){

}

Mesh::Mesh(myTexture *tex, vertex c){
    init(tex, c);
}

int Mesh::get_nb_p(){
    return nb_p;
}

int Mesh::get_nb_f(){
    return nb_f;
}

void Mesh::init(myTexture *t_c, vertex c){
    texture = t_c;
    centre = c;
}

void Mesh::draw(uint32_t **print_ref, pixel_profondeur ***z_ref, light L, camera C){
    std::vector<Vec2> list_coord;        ///la liste qui contiendras les coordonnées projetées des points
    std::vector<float> list_angles;      ///la liste qui contiendras les valeurs de l'angle entre la caméra et le vertex

    ///projection
    for(int i = 0; i < nb_p; i++){
        list_angles.push_back(angleEntreVec3(tab_p[i] - C.sight_lim, C.line_of_sight));

        if(list_angles[i] < PIsur2)
            list_coord.push_back(projette(tab_p[i], C));
        else{
            Vec2 vecTamp;
            list_coord.push_back(vecTamp);
        }
    }

    ///affichage de chaque face
    for(int i = 0; i < nb_f; i++){
        faceP newFace;
        if(clip_face(C, tab_f[i], list_coord, list_angles, newFace)){        ///si on a réussis le clipping du triangle...
            draw_face(print_ref, z_ref, L, C, texture, newFace);         ///...on affiche ses pixels dans le buffer
        }
    }
}

void Mesh::finish(){
    nb_p = List_vert.size();
    nb_f = List_face.size();
    tab_p = new vertex[nb_p];
    tab_f = new face[nb_f];

    for(int i = 0; i < nb_p; i++)
        tab_p[i] = List_vert[i];

    short int tab_tempnbp[3];
    vertex *tab_tempp[3];
    float2 tab_texp[3];
    for(int i = 0; i < nb_f; i++){
        for(int j = 0; j < 3; j++){
            tab_tempnbp[j] = List_face[i].GetNbVertex(j);
            tab_tempp[j] = &(tab_p[tab_tempnbp[j]]);
            tab_texp[j] = List_face[i].GetTex(j);
        }
        tab_f[i].init(tab_tempp, tab_tempnbp, tab_texp);
    }

    List_vert.clear();
    List_face.clear();
    //break_point();
}
/*
void Mesh::point(float _x, float _y, float _z){
    point(vertex(_x, _y, _z));
}

void Mesh::point(vertex ver){
    Create_vert(ver);
    point_chain++;
    int taille = List_vert.size();

    if(point_chain >= 3){
        face cycling_face;
        if(taille%2 != 0)
            cycling_face.init(&List_vert[taille - 1], &List_vert[taille - 3], &List_vert[taille - 2],
                                         taille - 1,             taille - 3,             taille - 2);
        else
            cycling_face.init(&List_vert[taille - 1], &List_vert[taille - 2], &List_vert[taille - 3],
                                         taille - 1,             taille - 2,             taille - 3);
        Create_face(cycling_face);
    }
}

void Mesh::break_point(){
    point_chain = 0;
}*/

void Mesh::point(float x, float y, float z){
    point(vertex(x,y,z));
}

void Mesh::point(vertex v){
    List_vert.push_back(v);
}

void Mesh::triangle(short int v1, short int v2, short int v3, float2 tex1, float2 tex2, float2 tex3){
    face f;
    f.init(&List_vert[v1], &List_vert[v2], &List_vert[v3], v1, v2, v3, tex1, tex2, tex3);
    triangle(f);
}

void Mesh::triangle(face f){
    List_face.push_back(f);
}

Mesh::~Mesh(){
    if(tab_p != NULL)
        delete [] tab_p;
    if(tab_f != NULL)
        delete [] tab_f;
}
