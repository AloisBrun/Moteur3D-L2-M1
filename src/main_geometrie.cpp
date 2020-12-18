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

