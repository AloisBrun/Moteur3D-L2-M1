#include <math.h>
#include <ctime>
#include <random>
#include <unistd.h>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include "Timer.h"



#include "const.h"
#include "types.h"
#include "function_global.h"
#include "camera.h"
#include "player.h"
#include "myTexture.h"









struct light{
    vertex Direction,
           pos;
    float intensite;

    void init(vertex p, vertex d, float intens){
        pos = p;

        Direction = d;

        intensite = intens;
    }
};





Vec2 projette(vertex p, camera C){
    vertex dif;         ///différence entre le point et la pos de la caméra
    Vec2   d,           ///vecteur de transition p -> b
           b;           ///pos voulue pour l'affichage sur l'écran 2D

    dif = p - C.posCam;

    float ///ABR1 = P.Sz*dif.y + P.Cz*dif.x,
          ABR2 = C.Cy*dif.z + C.Sy*dif.x;
          ///ABR3 = P.Cz*dif.y - P.Sz*ABR1;

    d.x = C.Cy*dif.x - C.Sy*dif.z;
    d.y = C.Sx*ABR2 + C.Cx*dif.y;
    d.z = C.Cx*ABR2 - C.Sx*dif.y;

    float ddz = depth / d.z;
    b.x = d.x * ddz + midX;
    b.y = d.y * ddz + midY;
    b.z = d.z;

    return b;
}

Vec2 projetteNeg(vertex p1, vertex p2, Vec2 v, Vec2 v2, camera C){ ///la même que projette pour le cas où z est négatif pour p1 mais pas pour p2
    vertex dif1 = p1 - C.posCam,
           dif2 = p2 - C.posCam,
           intersect;
    float da, db,
          s;
    Vec2 result;

    da = C.Cx*(C.Cy*dif1.z + C.Sy*dif1.x) - C.Sx*dif1.y - c_clip;
    db = C.Cx*(C.Cy*dif2.z + C.Sy*dif2.x) - C.Sx*dif2.y - c_clip;

    s = da/(da - db);
    if(s > 1)       ///dans le ca où da/(da -db) très petit la perte de précision peut faire déborder s, mieux vaut vérifier pour la sécurité
        s = 1;
    else if(s < 0)
        s = 0;

    intersect = p1 + s * (p2 - p1);

    result = projette(intersect, C);
    result.tx = v.tx + s * (v2.tx - v.tx);
    result.ty = v.ty + s * (v2.ty - v.ty);

    return result;
}

int HighestPoint(std::vector<Vec2> V){
    int j = 0,
        h_p = V[0].y;
    for(unsigned int i = 1; i < V.size(); i++){
        if(V[i].y > h_p){
            h_p = V[i].y;
            j = i;
        }
    }
    return j;
}

Vec2 clip_point(Vec2 p1, Vec2 p2, int border){    ///p2 est par défaut considéré comme celui dehors
    Vec2 dif, clip;
    dif = p2 - p1;
    int *clipX = &clip.x, *clipY = &clip.y,
        *p1X = &p1.x, *p1Y = &p1.y,
        *difX = &dif.x, *difY = &dif.y;

    if((border & 1) == 1){
        *clipX = borderXmin;
    }
    else if((border & 2) == 2){
        *clipX = borderXmax;
    }
    else{
        clipX = &clip.y;
        clipY = &clip.x;
        p1X = &p1.y;
        p1Y = &p1.x;
        difX = &dif.y;
        difY = &dif.x;

        if((border & 4) == 4){
            *clipX = borderYmin;
        }
        else{
            *clipX = borderYmax;
        }
    }

    if(*difX != 0){
        float coeff_alpha = (*clipX - *p1X) / (float)(*difX);
        *clipY = *p1Y + *difY * coeff_alpha;

        float coeff_beta = coeff_alpha * p1.z / ((1 - coeff_alpha) * p2.z + coeff_alpha * p1.z);
        clip.z = p1.z + dif.z * coeff_beta;
        clip.tx = p1.tx + dif.tx * coeff_beta;
        clip.ty = p1.ty + dif.ty * coeff_beta;
    }
    else{
        *clipY = *p1Y;
        clip.z = p1.z;

        clip.tx = p1.tx;
        clip.ty = p2.ty;
    }

    return clip;
}

int returnZone(Vec2 v){
    int result = 0;

    if(v.x < borderXmin){
        result += 1;
    }
    else if(v.x > borderXmax){
        result += 2;
    }
    if(v.y < borderYmin){
        result += 4;
    }
    else if(v.y > borderYmax){
        result += 8;
    }
    return result;
}








void interpolateY(std::vector<Vec2> &V, Vec2 S){
    Vec2 SV = V[V.size() - 1];
    V.pop_back();
    int difY = S.y - SV.y,
        difX = S.x - SV.x;

    float difZ = S.z - SV.z,
          difTx = S.tx - SV.tx,
          difTy = S.ty - SV.ty,
          coeff_alpha, coeff_beta;

    if(SV.y == S.y)
        V.pop_back();

    for(int y = difY - 1; y > 0; y--){
        Vec2 newVec;
        coeff_alpha = y/(float)difY;
        newVec.x = SV.x + difX * coeff_alpha;
        newVec.y = SV.y + y;

        coeff_beta = coeff_alpha * SV.z / ((1 - coeff_alpha) * S.z + coeff_alpha * SV.z);
        newVec.z = SV.z + difZ * coeff_beta;
        newVec.tx = SV.tx + difTx * coeff_beta;
        newVec.ty = SV.ty + difTy * coeff_beta;
        V.push_back(newVec);
    }
    V.push_back(SV);
}

bool isFaceNull(std::vector<Vec2> v){
    for(unsigned int i = 0; i < v.size() - 1; i++){
        if(v[i].y != v[i+1].y)
            return false;
    }
    if(v[v.size() - 1].y != v[0].y)
        return false;
    return true;
}

bool isInMiddleZone(std::vector<int> Vi){
    for(unsigned int i = 0; i < Vi.size(); i++){
        if(Vi[i] != 0)
            return false;
    }
    return true;
}

bool isInSameZone(std::vector<int> Vi){
    int t = (Vi[Vi.size() - 1] & Vi[0]);
    for(unsigned int i = 0; i < Vi.size(); i++){
        if((Vi[i] & t) == 0)
            return false;

        t = (Vi[i] & t);
    }
    return true;
}





class face{
private:
    short int tab_nbp[3];
    float2* tab_tex;
    vertex** tab_p;
    vertex* norm = NULL;

    void clear_face(){
        if(norm != NULL)
            delete norm;
        if(tab_tex != NULL)
            delete tab_tex;
        if(tab_p != NULL)
            delete [] tab_p; ///on delete seulement les pointeurs sur vertex
    }


public:
    face(){

    }

    void updateNormal(){
        *norm = getNormal(*tab_p[0] - *tab_p[1], *tab_p[2] - *tab_p[1]);
    }

    void init(vertex *vert1, vertex *vert2, vertex *vert3,
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

    void init(vertex* t_p[3], short int t_n[3], float2 t_tex[3]){
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

    vertex GetNorm(){
        return *norm;
    }

    float2 GetTex(int i){
        return tab_tex[i];
    }

    vertex GetVertex(int i){
        return *tab_p[i];
    }

    short int GetNbVertex(int i){
        return tab_nbp[i];
    }

    bool is_visible(camera C){
        vertex AC = C.posCam - *tab_p[0],
               AB = *norm;

        float theta = angleEntreVec3(AB, AC);

        if(theta < M_PI/2){
            return true;
        }
        return false;
    }

    ~face(){
        ///clear_face();
    }
};

struct faceP{
    std::vector<Vec2> tab_p;
    vertex norm;
};

void demi_projette(vertex &dif, vertex p2, Vec2 &v2, Vec2 &v, float2 f, camera C){
    v2.tx = f.x;
    v2.ty = f.y;

    v.z = C.Cx*(C.Cy*dif.z + C.Sy*dif.x) - C.Sx*dif.y;

    dif = p2 - C.posCam;
    v2.z = C.Cx*(C.Cy*dif.z + C.Sy*dif.x) - C.Sx*dif.y;
}

bool clip_face(camera C, face f, std::vector<Vec2> list_coord, std::vector<float> list_angles, faceP &newFace){
    ///ajout d'une nouvelle face
    newFace.norm = f.GetNorm();

    ///projection des vertexs
    int jp;
    for(int j = 0; j < tab_f_p_size; j++){
        jp = (j+1)%tab_f_p_size;
        Vec2 v = list_coord[f.GetNbVertex(j)];
        v.tx = f.GetTex(j).x;
        v.ty = f.GetTex(j).y;
        vertex p1 = f.GetVertex(j),
               p2 = f.GetVertex(jp);
        float theta1 = list_angles[f.GetNbVertex(j)],
              theta2 = list_angles[f.GetNbVertex(jp)];

        if(theta1 < PIsur2 && theta2 < PIsur2){ ///si les deux points sont devant la caméra, on rajoute simplement le point
            newFace.tab_p.push_back(v);
        }
        else{
            if(theta1 < PIsur2 && theta2 >= PIsur2){  ///si j2 est derriere la camera et j1 non
                Vec2 v2 = list_coord[f.GetNbVertex(jp)];
                vertex dif = p1 - C.posCam;

                demi_projette(dif, p2, v2, v, f.GetTex(jp), C);

                newFace.tab_p.push_back(v);
                newFace.tab_p.push_back(projetteNeg(p2, p1, v2, v, C));
            }
            else if(theta1 >= PIsur2 && theta2 < PIsur2){  ///si j1 est derriere la camera et j2 non
                Vec2 v2 = list_coord[f.GetNbVertex(jp)];
                vertex dif = p1 - C.posCam;

                demi_projette(dif, p2, v2, v, f.GetTex(jp), C);/*
                std::cout << v.tx << "   " << v.ty << "       "
                          << v2.tx << "   " << v2.ty << << std::endl;*/

                newFace.tab_p.push_back(projetteNeg(p1, p2, v, v2, C));
            }
        }
    }

    ///culling : si la face est invisible (norme dans même sens que le regard) on la passe
    if(newFace.tab_p.size() != 0 && f.is_visible(C)){
        ///clipping
        std::vector<int> list_zone;
        int zone_clip,
            j1, j2,
            zone1, zone2;

        for(unsigned int j = 0; j < newFace.tab_p.size(); j++)
            list_zone.push_back(returnZone(newFace.tab_p[j]));

        ///cout << "face : " << i << endl;
        if(isInSameZone(list_zone)){ ///on vérifie si tt les points sont dans une même zone
            return false;
        }
        else if(!isInMiddleZone(list_zone)){   ///si on ne se trouve pas dans un cas trivial, il faut vérifier chaque point en fonction de chaque bordure
            push_at(newFace.tab_p, newFace.tab_p[0], 0);
            push_at(list_zone, list_zone[0], 0);   ///on fait un double de la première case de tab_p et list_zone

            zone_clip = 1;
            for(int j = 0; j < 4; j++){     ///pour chacune des bordures
                j1 = 1;
                j2 = 2;
                zone1 = 0;
                zone2 = 0;

                ///cout << "bord : " << j << endl;

                do{                     ///pour chacun des points de la faces par rapport à la bordure actuelle
                    Vec2 pj1 = newFace.tab_p[j1],
                         pj2 = newFace.tab_p[j2];
                    zone1 = list_zone[j1];
                    zone2 = list_zone[j2];

                    ///si j1 et j2 dehors, on vérifie qu'ils ne coupent pas l'écran
                    if((zone1 & zone_clip) == zone_clip && (zone2 & zone_clip) == zone_clip){
                        pop_at(newFace.tab_p, j1);  ///on supprime j1
                        pop_at(list_zone, j1);
                        j1--;
                        j2--;
                    }
                    ///si j1 dedans et j2 dehors
                    else if((zone2 & zone_clip) == zone_clip){
                        Vec2 clip = clip_point(pj1, pj2, zone_clip);

                        push_at(newFace.tab_p, clip, j1);
                        push_at(list_zone, returnZone(clip), j1);

                        j1++;
                        j2++;
                    }
                    ///si j1 dehors et j2 dedans
                    else if((zone1 & zone_clip) == zone_clip){
                        Vec2 clip = clip_point(pj2, pj1, zone_clip);

                        newFace.tab_p[j1] = clip;
                        list_zone[j1] = returnZone(clip);
                    }
                    ///si les deux points se trouvent à l'intérieur il n'y a rien besoins de faire puisqu'ils sont déjà dans la liste


                    ++j1;
                    ++j2;
                    j1 %= newFace.tab_p.size();
                    j2 %= newFace.tab_p.size();
                }while(j1 != 0);

                newFace.tab_p[0] = newFace.tab_p[1];
                list_zone[0] = list_zone[1];

                zone_clip = zone_clip << 1;
            }
            pop_at(newFace.tab_p, 0);     ///une fois le clipping terminé, on supprime le doublon de tab_p
        }
    }
    else
        return false;     ///si la face n'est pas visible on peut arreter le code ici

    /*for(int i = 0; i < 3; i++){
        cout << "valeurs newFace : " newFace.tab_p[i].tx << " ; " << newFace.tab_p[i].ty << "               "  <<
                    newFace.tab_p[i].x << ", " << newFace.tab_p[i].y << ", " << newFace.tab_p[i].z << endl;
    }
    cout << endl;*/

    return true;
}

void draw_face(uint32_t **print_ref, pixel_profondeur ***z_ref, light L, camera C, myTexture *texture, faceP newFace){
    ///vérification que la face n'a pas deux de ses côtés fusionnés (on n'en voit qu'un trait) et qu'elle a plus de 2 côtés
    if((newFace.tab_p.size() < 3) || isFaceNull(newFace.tab_p))
        return;

    /*///calcul de l'intensité de la lumière refletée par la face
    float angle = cos(angleEntreVec3(-newFace.norm, L.Direction));
    if(angle < 0)
        angle = 0;
    float Id = L.intensite * angle + 1 - L.intensite;     ///Id = Ii * Kd * cos(theta), Kd étant le coefficient de réflection de la surface (ici = 1)
    */
    Vec2 S;             ///Sommets
    std::vector<Vec2> V1,    ///Voisin 1
                      V2;    ///Voisin 2
    int H = HighestPoint(newFace.tab_p),
        H2;
    S = newFace.tab_p[H];
    H2 = H;

    ///on remplit la liste V1 par interpolation de S au plus petit point (côté gauche)
   --H2;
    if(H2 < 0)
        H2 = newFace.tab_p.size() - 1;
    V1.push_back(S);
    while(newFace.tab_p[H2].y <= V1[V1.size() - 1].y){
        V1.push_back(newFace.tab_p[H2]);
        interpolateY(V1, V1[V1.size() - 2]);

        --H2;
        if(H2 < 0)
            H2 = newFace.tab_p.size() - 1;
    }

    ///pareil avec V2 mais dans l'autre sens (côté droit)
    H2 = H;
    ++H2 %= newFace.tab_p.size();
    V2.push_back(S);
    while(newFace.tab_p[H2].y <= V2[V2.size() - 1].y){
        V2.push_back(newFace.tab_p[H2]);
        interpolateY(V2, V2[V2.size() - 2]);

        ++H2 %= newFace.tab_p.size();
    }


    ///mipmapping
    int nb_m = 0,
        diff_m = nb_mip_map_max - texture->getMipMap();
    for(int i = 0; i < texture->getMipMap(); i++){
        if(V1[0].z > tab_mip_map[i + diff_m]){
            nb_m = i;
        }
    }

    ///rasterization
    char count_pixel;
    int HY = (DIMY - V1[0].y - 1) * DIMX,
        px, py,
        h = texture->getH(nb_m),
        w = texture->getW(nb_m),
        s_beta = 0,
        i_beta = 0;
    float interZ, inter_txW, inter_tyH,
          DifTxW, DifTyH, DifX, DifZ,
          pas_Z, pas_x,
          alpha, beta,
          DifBeta,
          tab_beta[2 + (int)(DIMX * pas_beta)],
          temp_float;

    tab_beta[0] = 0;
    for(unsigned int j = 0; j < V1.size(); j++){
        DifX = V2[j].x - V1[j].x;

        if(DifX != 0){
            ///réinitialisation des variables à chaque tour
            DifTxW = (V2[j].tx - V1[j].tx) * w - 0.0001;        ///on réduit légérement le coefficient pour éviter d'avoir des problèmes d'affichages
            DifTyH = (V2[j].ty - V1[j].ty) * h - 0.0001;        ///dû aux arrondis de valeur (bandes noires moches sur les bords)
            DifZ = V2[j].z - V1[j].z;
            pas_x = 1/DifX;

            ///on interpole les valeurs de Z, tx et ty entre chaque liste
            interZ = V1[j].z;
            inter_txW = V1[j].tx * w;
            inter_tyH = V1[j].ty * h;

            ///calcul des valeurs de beta à intervalles réguliers pour l'interpolation
            alpha = 0;
            temp_float = nb_beta * pas_x;
            s_beta = 1 + DifX * pas_beta;
            for(int i = 1; i < s_beta; i++){
                alpha += temp_float;
                tab_beta[i] = alpha * V1[j].z / ((1 - alpha) * V2[j].z + alpha * V1[j].z);
            }
            tab_beta[s_beta] = 1;
            i_beta =
            count_pixel = 0;
            beta = tab_beta[0];
            DifBeta = pas_beta * (tab_beta[1] - beta);

            ///init de pas_Z grâce à DifBeta
            pas_Z = DifZ * DifBeta;

//            std::cout << "nb_beta : " << nb_beta
//                      << ",    pas_beta : " << pas_beta
//                      << ",    temp_float : " << temp_float
//                      << ",    s_beta : " << s_beta
//                      << ",    DifBeta : " << DifBeta
//                      << ",    DifX : " << DifX
//                      << std::endl;
//
//            for(int i = 0; i <= s_beta; i++){
//                std::cout << "beta " << i << " : " << tab_beta[i] << std::endl;
//            }
        }

        ///pour chaque point (x,y) interpolé, on vérifie si interZ est inférieur à la valeur z stockée dans le buffer
        for(int k = V1[j].x; k < V2[j].x; k++){
            beta += DifBeta;
            interZ += pas_Z;
            ++count_pixel;
            if(count_pixel == nb_beta){
                ++i_beta;
                beta = tab_beta[i_beta];
                DifBeta = pas_beta * (tab_beta[i_beta + 1] - beta);
                pas_Z = DifBeta * DifZ;
                count_pixel = 0;
            }

            ///si interZ plus petit on remplace le pixel dans le buffer à afficher et met sa valeur en z dans le z_buffer
            if(interZ < (*z_ref)[k][V1[j].y].z){
                ///copie profondeur dans le buffer de profondeur
                (*z_ref)[k][V1[j].y].z = interZ;

                ///correction des coordonnées texture avec le paramètre beta
                px = inter_txW + DifTxW * beta;
                py = inter_tyH + DifTyH * beta;

                ///copie dans le buffer d'affichage
                (*print_ref)[HY + k] = texture->getTex(nb_m, px, py);

                ///affichage du z_buffer
//                float Z = interZ * 0.05;
//                if(Z > 255 )
//                    Z = 255;
//                if(Z < 0)
//                    Z = 0;
//
//                unsigned char c = Z;
//                (*print_ref)[HY + k] = (uint32_t)((((c << 8) + c) << 8) + c);
            }
        }
        HY += DIMX;
    }
}

void copyDownscaleBuffer(uint32_t *buffer, const int DIMX1, const int DIMY1,
                            uint32_t *&buffer_copy, const int DIMX2, const int DIMY2){
    int downscalingX = DIMX1/DIMX2,
        downscalingY = DIMY1/DIMY2,
        ox, oy;
    uint32_t tempCase,
             tempR,
             tempG,
             tempB,
             tempA,
             downscalingTotal = downscalingX * downscalingY;

    for(int x = 0; x < DIMX2; ++x){
      for(int y = 0; y < DIMY2; ++y){
        tempR = tempG = tempB = tempA = 0;
        ox = x * downscalingX;
        oy = y * downscalingY;

        for(int cx = 0; cx < downscalingX; cx++){
          for(int cy = 0; cy < downscalingY; cy++){
            tempCase = buffer[DIMX1*(oy + cy) + ox + cx];
            tempR += tempCase & 0x000000FF;
            tempG += (tempCase & 0x0000FF00) >> 8;
            tempB += (tempCase & 0x00FF0000) >> 16;
            tempA += (tempCase & 0xFF000000) >> 24;
          }
        }

        tempCase = ((tempR / downscalingTotal)) +
                   ((tempG / downscalingTotal) << 8) +
                   ((tempB / downscalingTotal) << 16) +
                   ((tempA / downscalingTotal) << 24);
        buffer_copy[y * DIMX2 + x] = (uint32_t)tempCase;
      }
    }
}




std::vector<vertex> List_vert;
std::vector<face> List_face;
int point_chain = 0;

class Mesh{
private:
    int nb_p = 0,
        nb_f = 0;
    vertex centre;
    vertex *tab_p;
    face *tab_f;

    myTexture *texture;

public:
    Mesh(){

    }

    Mesh(myTexture *tex, vertex c){
        init(tex, c);
    }

    int get_nb_p(){
        return nb_p;
    }

    int get_nb_f(){
        return nb_f;
    }

    void init(myTexture *t_c, vertex c){
        texture = t_c;
        centre = c;
    }

    void draw(uint32_t **print_ref, pixel_profondeur ***z_ref, light L, camera C){
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

    void finish(){
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
    void point(float _x, float _y, float _z){
        point(vertex(_x, _y, _z));
    }

    void point(vertex ver){
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

    void break_point(){
        point_chain = 0;
    }*/

    void point(float x, float y, float z){
        point(vertex(x,y,z));
    }

    void point(vertex v){
        List_vert.push_back(v);
    }

    void triangle(short int v1, short int v2, short int v3, float2 tex1, float2 tex2, float2 tex3){
        face f;
        f.init(&List_vert[v1], &List_vert[v2], &List_vert[v3], v1, v2, v3, tex1, tex2, tex3);
        triangle(f);
    }

    void triangle(face f){
        List_face.push_back(f);
    }

    ~Mesh(){
        if(tab_p != NULL)
            delete [] tab_p;
        if(tab_f != NULL)
            delete [] tab_f;
    }
};

Mesh* init_terrain(myTexture *pic){

}

Mesh* init_cube(myTexture *pic, vertex centre, int L = DIMX/2){
    ///initialisation du mesh (pos + texture)
    Mesh* m = new Mesh;
    m->init(pic, centre);
    float L2 = L/2,
          ftex = 0;
    int a = 0,
        b = 1,
        c = 2;

    for(int i = 0; i < 8; i++){
        m->point(centre.x + cube_grid[i][0] * L2,
                 centre.y + cube_grid[i][1] * L2,
                 centre.z + cube_grid[i][2] * L2);
    }


    for(int i = 0; i < 6; i++){
        switch(i){
            case 1:
                ftex = 0.33334f;
                break;
            case 5:
                ftex = 0.66667f;
                break;
        }
        a = 0;
        b = 1;
        c = 2;
        for(int j = 0; j < 2; j++){
            if(j == 1){ a = 1; b = 3; c = 2; }
            m->triangle(cube_f[i][a], cube_f[i][b], cube_f[i][c],
                        float2(ftex + 0.33333f * cube_tex[a][0], cube_tex[a][1]),
                        float2(ftex + 0.33333f * cube_tex[b][0], cube_tex[b][1]),
                        float2(ftex + 0.33333f * cube_tex[c][0], cube_tex[c][1]));
        }
    }

    ///on appel finish() pour basculer les vertex et faces déclarées dans le mesh, vidant les listes du même coup
    m->finish();
    return m;
}

void init_terrain_cube(std::vector<Mesh*> &list_m, myTexture *pic){
    int L = DIMX/4,
        H = -DIMY,
        conv = L / 2,
        Z = 0,
        tg = 5;

    list_m.push_back(init_cube(pic, vertex(-conv * 3, H + conv * 4, conv * 8), L));

    for(int i = -tg; i < tg; i++){
        for(int j = -tg; j < tg; j++){
            list_m.push_back(init_cube(pic, vertex(conv * (i * 2 + 1), H, conv * (j * 2 + Z * 4 + 1)), L));
        }
    }

    /*float theta = 0;
    for(int i = 0; i < 20; i++){
        theta += M_PI/10;
        list_m.push_back(t_c, L, 2000 * cos(theta), H, 2000 * sin(theta));
    }*/

    int x, z;
    for(int i = 0; i < 4; i++){
      switch(i){
        case 0 :
            x = -tg;
            break;
        case 1 :
            z = Z - tg;
            break;
        case 2 :
            x = tg - 1;
            break;
        case 3 :
            z = Z + tg - 1;
            break;
      }
      for(int j = -tg + 1; j < tg - 1; j++){
        if(i == 1 || i == 3)
            x = j;
        else
            z = Z + j;
        for(int y = 0; y < tg; y++){
            list_m.push_back(init_cube(pic, vertex(conv * (x * 2 + 1), H + conv * (y * 2 + 2), conv * (z * 2 + Z * 2 + 1)), L));
        }
      }
    }

}

class world{
private:
    player DUDE;
    std::vector<Mesh*> list_m;
    light LIGHT;
    myTexture grass;                ///structures utilisées pour stocker les textures

    SDL_Event events;
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *currentFrame;
    pixel_profondeur **z_buffer,
                     ***z_ref; ///pointeurception (O< O;)
    uint32_t *print_buffer,
             *resized_print_buffer,
             **print_ref,
             background_color = (uint32_t)((((127 << 8) + 186) << 8) + 255);    /// cyan


public :
    world(){
        init();
    }

    void init(){
        window = SDL_CreateWindow("AB&C renderer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, DIMXR, DIMYR, SDL_WINDOW_OPENGL);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        currentFrame = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, DIMXR, DIMYR);

        z_buffer = new pixel_profondeur*[DIMX];
        for(int i = 0; i < DIMX; i++)
            z_buffer[i] = new pixel_profondeur[DIMY];
        print_buffer = new uint32_t[DIMX*DIMY];
        resized_print_buffer = new uint32_t[DIMXR*DIMYR];

        z_ref = &z_buffer;
        print_ref = &print_buffer;


        ///initialisation textures
        /*liste chemins image :
            "data/debug.png"
            " .../grass_cut_16bits.png"
        */
        grass.init("data/grass_cut_16bits.png", window);


        ///initialisation des Mesh
        init_terrain_cube(list_m, &grass);
//        list_m.push_back(init_terrain(&grass));
//        list_m.push_back(init_cube(&grass, vertex(0, 0, 2000) ,DIMX/4));

        ///initialisation de la lumière
        vertex pos(0,0,0),
               dir(0,0,1);
        LIGHT.init(pos, dir, 0.8);
    }

    void draw(){
        ///vide les buffers
        for(int i = 0; i < DIMX * DIMY; i++){
            print_buffer[i] = background_color;
        }
        for(int i = 0; i < DIMX; i++){
            for(int j = 0; j < DIMY; j++){
                z_buffer[i][j].z = 65535;
            }
        }

        ///affiche chaque mesh
        for(unsigned int i = 0; i < list_m.size(); i++){
            list_m[i]->draw(print_ref, z_ref, LIGHT, DUDE.Cam);
        }

#if FSAA == 1
        SDL_UpdateTexture(currentFrame, NULL, print_buffer, DIMXR * sizeof (uint32_t));
#else
        copyDownscaleBuffer(print_buffer, DIMX, DIMY, resized_print_buffer, DIMXR, DIMYR);
        SDL_UpdateTexture(currentFrame, NULL, resized_print_buffer, DIMXR * sizeof (uint32_t));

#endif
        SDL_RenderCopy(renderer, currentFrame, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    void update(){
        ///update du joueur
        DUDE.update(events);
    }

    bool quit(){
        return events.type == SDL_QUIT;
    }

    ~world(){
        SDL_DestroyWindow(window);
        SDL_DestroyTexture(currentFrame);
        SDL_DestroyRenderer(renderer);

        delete[] print_buffer;
        delete print_ref;
        for(int i = 0; i < DIMX; i++)
            delete[] z_buffer[i];
        delete[] *z_buffer;
        delete z_ref;

        for(unsigned int i = 0; i < list_m.size(); i++){
            delete list_m[i];
        }
    }
};





int main (int, char**){
    srand(time(NULL));

    world w;

    Timer cap_fps,
          time_fps;
    int nb_frame = 0;

    while(!w.quit()){
        SDL_SetRelativeMouseMode(SDL_TRUE);
        SDL_ShowCursor(SDL_DISABLE);
        w.update();

        w.draw();

        nb_frame++;
        cap_fps.end();
        if(cap_fps.getTime() < tps_frame){
//            std::cout << "sleeping...." << std::endl;
            Timer::sleep(tps_frame - cap_fps.getTime());
        }
        cap_fps.start();

        time_fps.end();
        if(time_fps.getTime() >= 1000){
            time_fps.start();
            std::cout << "fps : " << nb_frame << std::endl;
            nb_frame = 0;
        }
    }

    return 0;
}




/**
Liste de choses à faire :
    -améliorer algorithme de mipmapping
    -bande bleue en bas de l'écran
    -scinder le projet en des zolis .h
    -commenter le code

    -corriger beta-interpolation
    -optimisation
        -fuites mémoires
        -algorithme de clipping
        -organisation, calculs prémédités
        -mémoire cache
        -fonctions SDL2
            -supprimer updateTexture & renderCopy, remplacer par SDL_RendererDrawPoints
            -lock
            -fast blitting
        -passage variables par référence si trop grande
        -limiter utilisation vector<>
        -accélération GPU
        -multi-threads!!!!!

encore + loins :
    -shadow mapping
    -anti-alliasing (x FSAA)
    -alpha channeling

    -shaders
    -bump mapping
    -bilinear filtering
    -anisotropic filtering

2ème partie (jeu???) :
    -editeur simpliste
    -charger des niveaux
    -animations simples
    -collisions



Finis :
    -ajouter une classe caméra séparée du joueur pour l'affichage
    -capage 60 fps ne marche pas!
    -corriger inputs
    -corriger textures (x fait)
    -nettoyage de code
        -passer les struct en class (x fait)
        -se débarasser de centre (x fait)
        -coord tex dans tableau dans chaque face (x fait)
        -référençage des face : passer de pointeurs à indice (useless)
        -se débarasser de namespace std + bibliotheques redondantes (x fait)
        -passer en SDL (x fait)
    -faciliter la création manuelle des meshs (x fait)
            -> déclaration "à l'openGL" : déclaration de l'ordre des vertexs, le programme se démmerde pour créer des faces entre eux
            -> passage du stockage des coordonnées textures des faces aux vertexs
            -> un peu moins bien optimisé niveau mémoire mais bien moins lourd à manipuler
    -bugs:
        -nb_beta = 12 -> affichage complètement cassé (origine : ???) (resolu????!!!)
        -débordement des valeurs pour l'interpolation des textures quand on regarde une face de côté -> affichage des textures hors de celle de la face (x résolu)
        -crash total du programme (source : ???, conditions : ???)  (résolu?)
        -glitch visuel quand on "sort" d'une face -> affichage éclair d'une texture sur tt l'écran (mauvais clipping) (x résolu)
**/

