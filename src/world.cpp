#include "world.h"

world::world(){
    init();
}

void world::init(){
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

void world::draw(){
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

void world::update(){
    ///update du joueur
    DUDE.update(events);
}

bool world::quit(){
    return events.type == SDL_QUIT;
}

world::~world(){
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





//Mesh* init_terrain(myTexture *pic){
//
//}

Mesh* init_cube(myTexture *pic, vertex centre, int L){
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
