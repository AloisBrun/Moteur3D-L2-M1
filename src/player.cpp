#include "player.h"

bool player::isKeyPressed(SDL_Scancode Scan){
    for(unsigned int i = 0; i < KEY_PRESSED.size(); i++)
        if(KEY_PRESSED[i] == Scan)
            return true;
    return false;
}

void player::popKey(SDL_Scancode Scan){
    bool pop = false;
    for(unsigned int i = 0; i < KEY_PRESSED.size() - 1; i++){
        if(KEY_PRESSED[i] == Scan)
            pop = true;
        if(pop)
            KEY_PRESSED[i] = KEY_PRESSED[i+1];
    }
    KEY_PRESSED.pop_back();
}

player::player(){
    pos.init(0,0,0);
    Cam.posCam = pos;
    Cam.angleCam.init(0,0,0);
    Cam.line_of_sight.init(0,0,1);
    Cam.sight_lim = Cam.posCam + Cam.line_of_sight * c_clip;
    Cam.OX.init(1,0,0);
    Cam.OY.init(0,1,0);

    vMove = vm;
    vAngulaire = va * Deg2Rad;

    time_dif = std::clock();
}

vertex player::getPos(){
    return pos;
}

void player::update(SDL_Event &event){
    ///gestion des inputs pour bouger le joueur
    float elipsedVmove = vMove * (std::clock() - time_dif);

    while(SDL_PollEvent(&event)){
        ///gestion des touches (pressé/relâché)
        if(event.type == SDL_KEYUP){
//                std::cout << "key up" << std::endl;
            if(isKeyPressed(event.key.keysym.scancode))
                popKey(event.key.keysym.scancode);
        }
        else if(event.type == SDL_KEYDOWN){
//                std::cout << "key down" << std::endl;
            if(!isKeyPressed(event.key.keysym.scancode))
                KEY_PRESSED.push_back(event.key.keysym.scancode);
        }
        ///gestions de la caméra
        else if(event.type == SDL_MOUSEMOTION){
//                std::cout << "mouse move!" << std::endl;
            ///inputs pour faire tourner la vision
            vertex r(0,0,0);

            int mx, my;
            SDL_GetRelativeMouseState(&mx, &my);

            r.y = vAngulaire * mx;
            r.x = vAngulaire * my;

            Cam.angleCam = Cam.angleCam + r;

            ///lock la vision quand on regard en l'air (évite de faire des 360)
            if(Cam.angleCam.x > M_PI / 2){
                Cam.angleCam.x = M_PI / 2;
            }
            else if(Cam.angleCam.x < -M_PI / 2){
                Cam.angleCam.x = -M_PI / 2;
            }
            ///ne change rien aux calculs mais préviens le débordement de valeurs (on ne sait jamais!)
            if(Cam.angleCam.y > M_PI * 2){
                Cam.angleCam.y -= M_PI * 2;
            }
            else if(Cam.angleCam.y < -M_PI * 2){
                Cam.angleCam.y += M_PI * 2;
            }

            Cam.update();
        }
    }

    vertex m(0,0,0);
    for(unsigned int i = 0; i < KEY_PRESSED.size(); i++){
        switch(KEY_PRESSED[i]){
            /*
            case SDL_SCANCODE_D:
                SDL_SetRelativeMouseMode(SDL_FALSE);
                SDL_ShowCursor(SDL_ENABLE);
                break;
            */
            case SDL_SCANCODE_LEFT:
                m = m - Cam.OX * elipsedVmove;
                break;
            case SDL_SCANCODE_RIGHT:
                m = m + Cam.OX * elipsedVmove;
                break;
            case SDL_SCANCODE_UP:
                m.x += -Cam.OX.z * elipsedVmove;
                m.z += Cam.OX.x * elipsedVmove;
                break;
            case SDL_SCANCODE_DOWN:
                m.x += Cam.OX.z * elipsedVmove;
                m.z += -Cam.OX.x * elipsedVmove;
                break;
            case SDL_SCANCODE_S:
                m.y = -elipsedVmove;
                break;
            case SDL_SCANCODE_Z:
                m.y = elipsedVmove;
                break;
            default:
                ;
        }
    }

    pos = pos + m;
    Cam.posCam = pos;
    Cam.sight_lim = Cam.posCam + Cam.line_of_sight * c_clip;

    ///on update time_dif
    time_dif = std::clock();
}
