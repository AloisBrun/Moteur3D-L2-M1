/***********************************************
             FICHIER DE CONSTANTES

*************************************************/

#ifndef CONST_H_INCLUDED
#define CONST_H_INCLUDED

#include <math.h>

#define FSAA 1       ///Full Screen anti-alliasing

const int   DIMXR = 800,     ///dimensions (x, y) de la fenêtre d'affichage (après anti-alliasing)
            DIMYR = 450,
            DIMX = DIMXR * FSAA,    ///dimensions (x, y) du buffer de rendu (avant anti-alliasing)
            DIMY = DIMYR * FSAA,
            midX = DIMX/2, midY = DIMY/2,
            MAX_FPS = 60,      ///nombre maximal de fps atteignable
            tab_f_p_size = 3,   ///nombre de vertex par face
            depth = 600 * FSAA,    ///profondeur du champs de la caméra, plus elle est petite plus le champs de vision est large
            c_clip = 5,    ///distance pour laquelle les vertex sont considérés comme étant derrière la caméra
            nb_beta = 8,   ///nb_beta pour déterminer la fréquence d'interpolation de beta -> + il est grand
                            ///plus la rasterization est rapide mais elle perd en précision
            borderXmin = 0, borderXmax = DIMX,  ///valeurs (x, y) pour lesquelles le clipping est limité
            borderYmin = 0, borderYmax = DIMY - 1,
            nb_mip_map_max = 12,    ///nombre de rétrécissement maximal de textures pour le mipmapping
            tab_mip_map[nb_mip_map_max] = {50, 100, 250, 500, 1000, 2000, 4000, 7000, 12000, 20000, 28000, 40000};
                                ///distances d'affichage de la texture au rang correspondant pour mipmapping

const float Deg2Rad = M_PI / 180,   ///coefficient de conversion degree -> radiant
            Rad2Deg = 180 / M_PI,   ///coefficient de conversion radiant -> degree
            PIsur2 = M_PI/2,
            pas_beta = 1/(float)nb_beta,
            tps_frame = 1000/MAX_FPS,
            vm = 2, va = 0.1;       ///respectivement vitesse de marche et vitesse de mouvement de la caméra pour le joueur

const char  cube_grid[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1}},
                                                                                    ///coordonnées des vertex pour le cube
            cube_f[6][4] = { {4,7,5,6}, {0,4,1,5}, {1,5,2,6}, {3,7,0,4}, {2,6,3,7}, {3,0,2,1} },  ///numero des vertex pour chaque face du cube
            cube_tex[4][2] = {{0,0}, {0,1}, {1,0}, {1,1}};      ///ordre des textures sur la face d'un cube

#endif // CONST_H_INCLUDED
