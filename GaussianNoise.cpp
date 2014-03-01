/*************************************************************
 * Copyright @ Dullerud's Autonomous Lab
 * University of Illinois Urbana Champaign
 * 
 * Author @ Bicheng Zhang <viczhang1990@gmail.com>
 * 
 * <---3D Simulation Framework for Autonomous Vehicles-->
 *
 *
 *            ********                  *****
 *             -A---               --A---
 *                
 *                      U U
 *                      ^
 *
 *
 *
 ************************************************************/

#include <math.h>
#include <stdlib.h>

static float spare;
static char spareready = 0;
 

float getGaussian(float center, float stdDev) {
        if (spareready) {
                spareready = 0;
                return spare * stdDev + center;
        } else {
                float u, v, s;
                do {
                        u = (float) (rand())/RAND_MAX * 2  - 1;
                        v = (float) (rand())/RAND_MAX * 2 - 1;
                        s = u * u + v * v;
                } while (s >= 1 || s == 0);
                spare = v *sqrtf(-2.0 *log(s) / s);
                spareready = 1;
                return center + stdDev * u * sqrtf(-2.0 * log(s) / s);
        }
}