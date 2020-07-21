
#ifndef _ROBOT_DEFS_H_
#define _ROBOT_DEFS_H_

/* Window Dimensions */
const int WINDOW_WIDTH = 600;
const int WINDOW_HEIGHT = 420;

/* Pixel-to-Meter Ratios */
const double PIXELS_PER_METER = 100.0;
const double METERS_PER_PIXEL = 1.0 / PIXELS_PER_METER;

/* Field Constants (in pixels) */
const int FIELD_LENGTH = 540;
const int FIELD_WIDTH = 360;
const int FIELD_OFFSET_X = 30;
const int FIELD_OFFSET_Y = 30;
const int GOAL_LINE_WIDTH = 80;
const int GOAL_BOX_LENGTH = 65;
const int GOAL_BOX_WIDTH = 130;
const int MARKER_OFFSET_X = 135;
const int MARKER_OFFSET_Y = 15;

/* Robot Radius (in pixels) */
const int ROBOT_RADIUS = 7;

/* Number of Field Landmarks */
const int NUM_LANDMARKS = 4;

/* Robot state */
struct RobotState {
  double x;      // x-position (in meters)
  double y;      // y-position (in meters)
  double theta;  // orientation (in radians)
};

/* Field location structure */
struct FieldLocation {
  double x;  // x-position (in meters)
  double y;  // y-position (in meters)
};

/* Field location structure */
struct MarkerObservation {
  int markerIndex;     // Index of observed marker [0-3]
  double distance;     // Observed distance to landmark from robot position
  double orientation;  // Observed bearing to landmark in local robot coordinate
                       // frame
};

/* Field location structure */
struct MarkerObservation_World {
  int markerIndex;  // Index of observed marker [0-3]
  double x;         // x-position (in meters)
  double y;         // y-position (in meters)
};

/* Robot model parameters */
struct RobotParams {
  double angle_fov;  // Field of view of robot (in degrees)

  // Sensor noise model parameters
  double sensor_noise_distance;
  double sensor_noise_orientation;

  // Odometry noise model parameters
  double odom_noise_rotation_from_rotation;
  double odom_noise_rotation_from_translation;
  double odom_noise_translation_from_translation;
  double odom_noise_translation_from_rotation;
};

#endif
