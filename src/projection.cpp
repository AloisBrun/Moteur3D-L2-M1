#include "projection.h"

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

void demi_projette(vertex &dif, vertex p2, Vec2 &v2, Vec2 &v, float2 f, camera C){
    v2.tx = f.x;
    v2.ty = f.y;

    v.z = C.Cx*(C.Cy*dif.z + C.Sy*dif.x) - C.Sx*dif.y;

    dif = p2 - C.posCam;
    v2.z = C.Cx*(C.Cy*dif.z + C.Sy*dif.x) - C.Sx*dif.y;
}
