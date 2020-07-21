/*
 *  SOLUTION FOR ALGOLUX
 *  Created on: May 02, 2020
 *  Author: Munir Jojo-Verge
 */

#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "controller.h"
#include "particle_filter.h"
#include "robot_defs.h"

// Create particle filter instance
ParticleFilter pf;

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current
 * robot position estimate.
 */
void getRobotPositionEstimate(RobotState& estimatePosn) {
  // TODO: Write your procedures to set the current robot position estimate here

  // Calculate and output the average weighted error of the particle
  // filter over all time steps so far.

  Particle best_particle = pf.get_best_particle();
  std::cout << "Best Estimate- x: " << best_particle.x
            << ", y: " << best_particle.y << ", t: " << best_particle.theta
            << ". W: " << best_particle.weight << std::endl;

  estimatePosn.x = best_particle.x;
  estimatePosn.y = best_particle.y;
  estimatePosn.theta = best_particle.theta;
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the
 * robot in local robot coordinates (observed by odometry model), which
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta) {
  // TODO: Write your motion update procedures here
  pf.motionUpdate(delta);
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all
 * marker obervations (marker index and position of marker in robot
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations) {
  // TODO: Write your sensor update procedures here

  // update the weights then resample
  pf.updateWeights(observations);
  pf.resample();
}

/**
 * myinit()
 * Initialization function that takes as input the initial
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams,
            FieldLocation markerLocations[NUM_LANDMARKS]) {
  // TODO: Write your initialization procedures here
  std::cout << "Robot ini state: " << robotState.x << ", " << robotState.y
            << ", " << robotState.theta << std::endl;

  if (!pf.initialized()) {
    pf.init(125, robotState, robotParams, markerLocations);
  }
}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay() {
  // TODO: Write your drawing procedures here
  //       (e.g., robot position uncertainty representation)

  // Example drawing procedure
  int pixelX, pixelY;
  const int NUM_POINTS = pf.num_particles();

  // Particle best_particle = pf.get_best_particle();
  // const char* fmt = "%f";
  // // int sz = std::snprintf(nullptr, 0, fmt, best_particle.weight);
  // char* buf;  // note +1 for null terminator
  // std::snprintf(buf, 5, fmt, best_particle.weight);
  // drawString(best_particle.x, best_particle.y, 0, buf);

  // Draw cyan colored points at specified global locations on field
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.99f, 0.0f);  // make this vertex red

  for (auto particle : pf.particles_data()) {
    global2pixel(particle.x, particle.y, pixelX, pixelY);
    glPointSize(4);
    glVertex2i(pixelX, pixelY);
  }

  glEnd();
}

/**
 * Draw a character string.
 *
 * @param x        The x position
 * @param y        The y position
 * @param z        The z position
 * @param string   The character string
 */
void drawString(float x, float y, float z, char* string) {
  glRasterPos3f(x, y, z);

  for (char* c = string; *c != '\0'; c++) {
    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,
                        *c);  // Updates the position
  }
}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key) {
  // TODO: (Optional) Write your keyboard input handling procedures here

  return 0;
}

/**
 * Main entrypoint for the program.
 */
int main(int argc, char** argv) {
  // Initialize world, sets initial robot position
  // calls myinit() before returning
  runMainLoop(argc, argv);
  return 0;
}
