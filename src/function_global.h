/***********************************************
      FICHIER DE FONCTIONS A USAGE GLOBAL
*************************************************/

#ifndef FUNC_GLOBAL_H_INCLUDED
#define FUNC_GLOBAL_H_INCLUDED

#include "types.h"
#include <math.h>
#include <vector>


float norme(vertex N);

float fast_inverse_square_root(float x);

float angleEntreVec3(vertex AB, vertex AC);

vertex getNormal(vertex AB, vertex AC);


///         A JETER DES QUE POSSIBLE
void pop_at(std::vector<int> &V, unsigned int H);

void push_at(std::vector<int> &V, int I, unsigned int H);

void pop_at(std::vector<Vec2> &V, unsigned int H);

void push_at(std::vector<Vec2> &V, Vec2 I, unsigned int H);


#endif // FUNC_GLOBAL_H_INCLUDED
