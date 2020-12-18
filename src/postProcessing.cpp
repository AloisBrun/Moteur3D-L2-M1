#include "postProcessing.h"

void copyDownscaleBuffer(uint32_t *buffer, const int DIMX1, const int DIMY1,
                            uint32_t *&buffer_copy, const int DIMX2, const int DIMY2){
    int downscalingX = DIMX1/DIMX2,
        downscalingY = DIMY1/DIMY2,
        ox, oy;
    uint32_t tempCase,
             tempR,
             tempG,
             tempB,
             tempA,
             downscalingTotal = downscalingX * downscalingY;

    for(int x = 0; x < DIMX2; ++x){
      for(int y = 0; y < DIMY2; ++y){
        tempR = tempG = tempB = tempA = 0;
        ox = x * downscalingX;
        oy = y * downscalingY;

        for(int cx = 0; cx < downscalingX; cx++){
          for(int cy = 0; cy < downscalingY; cy++){
            tempCase = buffer[DIMX1*(oy + cy) + ox + cx];
            tempR += tempCase & 0x000000FF;
            tempG += (tempCase & 0x0000FF00) >> 8;
            tempB += (tempCase & 0x00FF0000) >> 16;
            tempA += (tempCase & 0xFF000000) >> 24;
          }
        }

        tempCase = ((tempR / downscalingTotal)) +
                   ((tempG / downscalingTotal) << 8) +
                   ((tempB / downscalingTotal) << 16) +
                   ((tempA / downscalingTotal) << 24);
        buffer_copy[y * DIMX2 + x] = (uint32_t)tempCase;
      }
    }
}
