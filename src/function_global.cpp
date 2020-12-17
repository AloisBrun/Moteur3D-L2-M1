#include "function_global.h"


float norme(vertex N){
    return sqrt(N.x*N.x + N.y*N.y + N.z*N.z);
}

float fast_inverse_square_root(float x){
    float xhalf = 0.5f * x;
    int i = *(int *)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x = x*(1.5f - xhalf*x*x);

    return x;
}

float angleEntreVec3(vertex AB, vertex AC){
        ///AC.AB = ||AC||.||AB||.cos(theta) <=> cos(theta) = AC.AB / (||AC||.||AB||)
        return acos((AC.x*AB.x + AC.y*AB.y + AC.z*AB.z)
                        * fast_inverse_square_root(AB.x*AB.x + AB.y*AB.y + AB.z*AB.z)       ///on utilise la fonction de calcul rapide pour la racine inversé
                            * fast_inverse_square_root(AC.x*AC.x + AC.y*AC.y + AC.z*AC.z));
}

vertex getNormal(vertex AB, vertex AC){
    vertex N;

    N.x = AB.y*AC.z - AB.z*AC.y;
    N.y = AB.z*AC.x - AB.x*AC.z;
    N.z = AB.x*AC.y - AB.y*AC.x;

    N = N * fast_inverse_square_root(N.x*N.x + N.y*N.y + N.z*N.z);

    return -N;
}




///berk berk berk
void pop_at(std::vector<int> &V, unsigned int H){
    for(unsigned int i = H; i < V.size() - 1; i++){
        V[i] = V[i+1];
    }
    V.pop_back();
}

void push_at(std::vector<int> &V, int I, unsigned int H){
    V.push_back(I);
    for(unsigned int i = V.size() - 1; i > H + 1; i--){
        V[i] = V[i-1];
    }
    V[H + 1] = I;
}

void pop_at(std::vector<Vec2> &V, unsigned int H){
    for(unsigned int i = H; i < V.size() - 1; i++){
        V[i] = V[i+1];
    }
    V.pop_back();
}

void push_at(std::vector<Vec2> &V, Vec2 I, unsigned int H){
    V.push_back(I);
    for(unsigned int i = V.size() - 1; i > H + 1; i--){
        V[i] = V[i-1];
    }
    V[H + 1] = I;
}
