#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "robot_defs.h"

// Initializes robot state and marker locations, 
// sets up OpenGL window management, then calls user-defined 
// myinit() function before starting main controller loop
void runMainLoop(int argc, char** argv);

// Convert from pixel to global coordinates
void pixel2global(int pixelX, int pixelY, double& globalX, double& globalY);

// Convert from global to pixel coordinates
void global2pixel(double globalX, double globalY, int& pixelX, int& pixelY);

#endif
