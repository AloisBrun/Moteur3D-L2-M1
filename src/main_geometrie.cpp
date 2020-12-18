#include <random>
#include <iostream>
#include "Timer.h"
#include "world.h"

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

