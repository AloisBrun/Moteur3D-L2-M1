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
#include "mesh.h"








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

    ///on appel finish() pour basculer les vertex et faces d�clar�es dans le mesh, vidant les listes du m�me coup
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
    myTexture grass;                ///structures utilis�es pour stocker les textures

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

        ///initialisation de la lumi�re
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
Liste de choses � faire :
    -am�liorer algorithme de mipmapping
    -bande bleue en bas de l'�cran
    -scinder le projet en des zolis .h
    -commenter le code

    -corriger beta-interpolation
    -optimisation
        -fuites m�moires
        -algorithme de clipping
        -organisation, calculs pr�m�dit�s
        -m�moire cache
        -fonctions SDL2
            -supprimer updateTexture & renderCopy, remplacer par SDL_RendererDrawPoints
            -lock
            -fast blitting
        -passage variables par r�f�rence si trop grande
        -limiter utilisation vector<>
        -acc�l�ration GPU
        -multi-threads!!!!!

encore + loins :
    -shadow mapping
    -anti-alliasing (x FSAA)
    -alpha channeling

    -shaders
    -bump mapping
    -bilinear filtering
    -anisotropic filtering

2�me partie (jeu???) :
    -editeur simpliste
    -charger des niveaux
    -animations simples
    -collisions



Finis :
    -ajouter une classe cam�ra s�par�e du joueur pour l'affichage
    -capage 60 fps ne marche pas!
    -corriger inputs
    -corriger textures (x fait)
    -nettoyage de code
        -passer les struct en class (x fait)
        -se d�barasser de centre (x fait)
        -coord tex dans tableau dans chaque face (x fait)
        -r�f�ren�age des face : passer de pointeurs � indice (useless)
        -se d�barasser de namespace std + bibliotheques redondantes (x fait)
        -passer en SDL (x fait)
    -faciliter la cr�ation manuelle des meshs (x fait)
            -> d�claration "� l'openGL" : d�claration de l'ordre des vertexs, le programme se d�mmerde pour cr�er des faces entre eux
            -> passage du stockage des coordonn�es textures des faces aux vertexs
            -> un peu moins bien optimis� niveau m�moire mais bien moins lourd � manipuler
    -bugs:
        -nb_beta = 12 -> affichage compl�tement cass� (origine : ???) (resolu????!!!)
        -d�bordement des valeurs pour l'interpolation des textures quand on regarde une face de c�t� -> affichage des textures hors de celle de la face (x r�solu)
        -crash total du programme (source : ???, conditions : ???)  (r�solu?)
        -glitch visuel quand on "sort" d'une face -> affichage �clair d'une texture sur tt l'�cran (mauvais clipping) (x r�solu)
**/

