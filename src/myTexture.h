#ifndef MYTEXTURE_H_INCLUDED
#define MYTEXTURE_H_INCLUDED

#include <unistd.h>
#include <string>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iostream>

class myTexture{
private:
    uint32_t ***tex;
    int *w, *h,
        nb_mipmap = 1;

public:
    void init(std::string file_name, SDL_Window *screen);

    int getMipMap();

    int getH(int i);

    int getW(int i);

    uint32_t getTex(int mm, int w, int h);

    ~myTexture();
};

#endif // MYTEXTURE_H_INCLUDED
