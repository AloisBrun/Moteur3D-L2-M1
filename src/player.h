/***********************************************
                CLASSE JOUEUR

*************************************************/

#ifndef PLAYER_H_INCLUDED
#define PLAYER_H_INCLUDED

#include "types.h"
#include "camera.h"
#include "const.h"
#include <vector>
#include <ctime>
#include <SDL2/SDL.h>

class player{
private:
    vertex pos;
    float vMove, vAngulaire,
          time_dif = 0;
    std::vector<SDL_Scancode> KEY_PRESSED;  ///input buffer

    bool isKeyPressed(SDL_Scancode Scan);

    void popKey(SDL_Scancode Scan);

public:
    camera Cam;

    player();

    vertex getPos();

    void update(SDL_Event &event);
};

#endif // PLAYER_H_INCLUDED
