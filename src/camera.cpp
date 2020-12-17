#include "camera.h"

void camera::update(){
        ///on update la valeur des angles pour accélérer la projection
        Cx = cos(angleCam.x);
        Cy = cos(angleCam.y);
        Cz = cos(angleCam.z);

        Sx = sin(angleCam.x);
        Sy = sin(angleCam.y);
        Sz = sin(angleCam.z);

        ///update des normales à la direction du regard
        OX.init(Cy, 0, -Sy);
        OY.init(Sy * Sx, Cx, Cy * Sx);

        ///update de la direction du regard
        line_of_sight = getNormal(OY, OX);
    }
