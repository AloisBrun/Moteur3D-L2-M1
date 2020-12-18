/***********************************************
             CLASSE DE TEXTURE

*************************************************/

#include "myTexture.h"

void myTexture::init(std::string file_name, SDL_Window *screen){
    SDL_Surface *im = IMG_Load(file_name.c_str());

    if(im == NULL)
        std::cout << "erreur, impossible de charger l'image " << file_name << std::endl;

    SDL_LockSurface(im);

    int w2 = im->w,
        h2 = im->h;
    while(w2 != 1 && h2 != 1){
        w2 = w2/2; ///(int)((float)w/2 + 0.5f);
        h2 = h2/2; ///(int)((float)h/2 + 0.5f);
        nb_mipmap++;
    }
    tex = new uint32_t**[nb_mipmap];
    w = new int[nb_mipmap];
    h = new int[nb_mipmap];


    w2 = im->w;
    h2 = im->h;
    for(int i = 0; i < nb_mipmap; i++){
        w[i] = w2;
        h[i] = h2;

        w2 = w2/2; ///(int)((float)w/2 + 0.5f);
        h2 = h2/2; ///(int)((float)h/2 + 0.5f);
    }


    ///std::cout << (int)im->format->BitsPerPixel << std::endl;
    uint8_t *pixel = 0;
    uint8_t r = 0, g = 0, b = 0, a = 0;
    tex[0] = new uint32_t*[w[0]];
    for(int i = 0; i < w[0]; i++){
        tex[0][i] = new uint32_t[h[0]];
        for(int j = 0; j < h[0]; j++){
            pixel = (uint8_t*)im->pixels + ((h[0] - 1 - j) * w[0] + i) * 3;      ///hautement non-optimal!!!
            r = *pixel;
            g = *(pixel + 1);
            b = *(pixel + 2);
            tex[0][i][j] = (uint32_t)((((((a << 8) + r) << 8) + g) << 8) + b);
        }
    }


    for(int m = 1; m < nb_mipmap; m++){
        tex[m] = new uint32_t*[w[m]];
        for(int i = 0; i < w[m]; i++){
            tex[m][i] = new uint32_t[h[m]];
            for(int j = 0; j < h[m]; j++){
                uint32_t  Rm = 0,
                          Gm = 0,
                          Bm = 0;
                for(int k = 0; k < 2; k++){
                    for(int l = 0; l < 2; l++){
                        Rm += (unsigned char)(tex[m - 1][i*2 + k][j*2 + l] >> 16);
                        Gm += (unsigned char)(tex[m - 1][i*2 + k][j*2 + l] >> 8);
                        Bm += (unsigned char)tex[m - 1][i*2 + k][j*2 + l];
                    }
                }

                Rm /= 4;
                Gm /= 4;
                Bm /= 4;
                tex[m][i][j] = (((Rm << 8) + Gm) << 8) + Bm;
            }
        }
    }

    SDL_FreeSurface(im);
}

int myTexture::getMipMap(){
    return nb_mipmap;
}

int myTexture::getH(int i){
    return h[i];
}

int myTexture::getW(int i){
    return w[i];
}

uint32_t myTexture::getTex(int mm, int w, int h){
    return tex[mm][w][h];
}

myTexture::~myTexture(){
    for(int i = 0; i < nb_mipmap; i++){
        for(int j = 0; j < w[i]; j++){
            delete[] tex[i][j];
        }
        delete[] tex[i];
    }
    delete[] tex;

    delete[] h;
    delete[] w;
}
