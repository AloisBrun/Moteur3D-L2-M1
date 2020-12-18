/***********************************************
    CLASSE EXTERNE DE GENERATION DE LA SCENE

*************************************************/

#ifndef WORLD_H_INCLUDED
#define WORLD_H_INCLUDED

#include "types.h"
#include "player.h"
#include "mesh.h"
#include "myTexture.h"
#include "light.h"
#include <vector>

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
    world();

    void init();

    void draw();

    void update();

    bool quit();

    ~world();
};

//Mesh* init_terrain(myTexture *pic);

Mesh* init_cube(myTexture *pic, vertex centre, int L = DIMX/2);

void init_terrain_cube(std::vector<Mesh*> &list_m, myTexture *pic);



#endif // WORLD_H_INCLUDED
