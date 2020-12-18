#include "rasterizer.h"

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
