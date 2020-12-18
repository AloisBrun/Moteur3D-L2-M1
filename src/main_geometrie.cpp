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
#include "light.h"
#include "projection.h"
#include "clipping.h"
#include "face.h"
#include "rasterizer.h"
#include "postProcessing.h"






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

