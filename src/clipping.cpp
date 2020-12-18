#include "clipping.h"

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
