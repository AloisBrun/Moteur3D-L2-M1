/***********************************************
   FONCTIONS APRES RASTERIZATION DE LA SCENE

*************************************************/

#ifndef POST_PROCESSING_H_INCLUDED
#define POST_PROCESSING_H_INCLUDED

#include <unistd.h>

void copyDownscaleBuffer(uint32_t *buffer, const int DIMX1, const int DIMY1,
                            uint32_t *&buffer_copy, const int DIMX2, const int DIMY2);

#endif // POST_PROCESSING_H_INCLUDED
