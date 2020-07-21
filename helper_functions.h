/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 * Created on: May 02, 2020
 * Author: Munir Jojo-Verge
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <vector>

#include "robot_defs.h"

#define PI 3.14159265358979323846 /* pi */

/*
 * Splits a string into "words" deparated by a space (" ")
 * @param String
 * @output Container (template) filled with the "words"
 */
template <class Container>
inline void split1(const std::string& str, Container& cont) {
  std::istringstream iss(str);
  std::copy(std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>(), std::back_inserter(cont));
}

/*
 * Splits a string into "words" deparated by a delimiter character
 * @param String
 * @param delimiter character
 * @output Container (template) filled with the "words"
 */
template <class Container>
inline void split2(const std::string& str, Container& cont, char delim = ' ') {
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    cont.push_back(token);
  }
}

/*
 * Struct representing one position/control measurement.
 */
struct control_s {
  double velocity;  // Velocity [m/s]
  double yawrate;   // Yaw rate [rad/s]
};

/*
 * Struct representing one ground truth position.
 */
struct ground_truth {
  double x;      // Global vehicle x position [m]
  double y;      // Global vehicle y position
  double theta;  // Global vehicle yaw [rad]
};

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/*
 * Computes the Euclidean distance between two 2D points given in polar
 * coordinates.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(MarkerObservation p1, MarkerObservation p2) {
  double x1 = p1.distance * cos(p1.orientation);
  double y1 = p1.distance * sin(p1.orientation);
  double x2 = p2.distance * cos(p2.orientation);
  double y2 = p2.distance * sin(p2.orientation);
  return dist(x1, y1, x2, y2);
}

/*
 * Computes the errors (vector representing the errors on x, y, theta dims)
 * between ground truth point (x,y) and the position an a particle.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output error vector
 */
inline double* getError(double gt_x, double gt_y, double gt_theta, double pf_x,
                        double pf_y, double pf_theta) {
  static double error[3];
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  if (error[2] > M_PI) {
    error[2] = 2.0 * M_PI - error[2];
  }
  return error;
}

/* Reads configuration data from a file (sample_inputx.txt)
 * @param filename Name of file containing robot configuration data.
 * @output True if opening and reading file was successful.
 */
inline bool read_config_data(std::string filename, RobotParams& robotParams) {
  // Get file of map:
  std::ifstream in_file_config(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file.
  if (!in_file_config) {
    return false;
  }

  // Declare single line of the configuration file:
  std::string line_config;

  // words container
  std::vector<std::string> words;

  // Run over each single line:
  while (getline(in_file_config, line_config)) {
    std::istringstream iss_config(line_config);

    // Declare param name & value
    std::string name;
    double value;

    // Read data from current line to values
    iss_config >> name;
    iss_config >> value;

    if (name == "angle_fov:") {
      robotParams.angle_fov = value;
    } else if (name == "sensor_noise_distance:") {
      robotParams.sensor_noise_distance = value;
    } else if (name == "sensor_noise_orientation:") {
      robotParams.sensor_noise_distance = value;
    } else if (name == "odom_noise_rotation_from_rotation:") {
      robotParams.sensor_noise_distance = value;
    } else if (name == "odom_noise_rotation_from_translation:") {
      robotParams.sensor_noise_distance = value;
    } else if (name == "odom_noise_translation_from_translation:") {
      robotParams.sensor_noise_distance = value;
    } else if (name == "odom_noise_translation_from_rotation:") {
      robotParams.sensor_noise_distance = value;
    }
  }
  return true;
}

/* Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {
  // Get file of position measurements:
  std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file.
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file:
  std::string line_pos;

  // Run over each single line:
  while (getline(in_file_pos, line_pos)) {
    std::istringstream iss_pos(line_pos);

    // Declare position values:
    double x, y, azimuth;

    // Declare single ground truth:
    ground_truth single_gt;

    // read data from line to values:
    iss_pos >> x;
    iss_pos >> y;
    iss_pos >> azimuth;

    // Set values
    single_gt.x = x;
    single_gt.y = y;
    single_gt.theta = azimuth;

    // Add to list of control measurements and ground truth:
    gt.push_back(single_gt);
  }
  return true;
}

/**
*  Angle normalization to [-Pi, Pi]
        For optimization we create this procedure since it's repeated twice
*/
inline void angNorm(double& ang) {
  while (ang > PI) ang -= 2. * PI;
  while (ang < -PI) ang += 2. * PI;
}

/*
******************************************************************************
Here I will try to optimize the Closest Neighbor Algorithm
******************************************************************************
// A divide and conquer program in C++ to find the smallest distance from a
// given set of points.

// A structure to represent a Point in 2D plane
struct Point
{
        int x, y;
};


// Following two functions are needed for library function qsort().
// Refer: http://www.cplusplus.com/reference/clibrary/cstdlib/qsort/

// Needed to sort array of points according to X coordinate
int compareX(const void* a, const void* b)
{
        Point *p1 = (Point *)a, *p2 = (Point *)b;
        return (p1->x - p2->x);
}
// Needed to sort array of points according to Y coordinate
int compareY(const void* a, const void* b)
{
        Point *p1 = (Point *)a, *p2 = (Point *)b;
        return (p1->y - p2->y);
}

// A utility function to find the distance between two points
double dist(Point p1, Point p2)
{
        return sqrt((p1.x - p2.x)*(p1.x - p2.x) +
                (p1.y - p2.y)*(p1.y - p2.y)
        );
}

// A Brute Force method to return the smallest distance between two points
// in P[] of size n
float bruteForce(Point P[], int n)
{
        float min = FLT_MAX;
        for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                        if (dist(P[i], P[j]) < min)
                                min = dist(P[i], P[j]);
        return min;
}

// A utility function to find minimum of two float values
float min(float x, float y)
{
        return (x < y) ? x : y;
}


// A utility function to find the distance beween the closest points of
// strip of given size. All points in strip[] are sorted accordint to
// y coordinate. They all have an upper bound on minimum distance as d.
// Note that this method seems to be a O(n^2) method, but it's a O(n)
// method as the inner loop runs at most 6 times
float stripClosest(Point strip[], int size, float d)
{
        float min = d;  // Initialize the minimum distance as d

                                        // Pick all points one by one and try
the next points till the difference
                                        // between y coordinates is smaller than
d.
                                        // This is a proven fact that this loop
runs at most 6 times for (int i = 0; i < size; ++i) for (int j = i + 1; j < size
&& (strip[j].y - strip[i].y) < min; ++j) if (dist(strip[i], strip[j]) < min) min
= dist(strip[i], strip[j]);

        return min;
}
// A recursive function to find the smallest distance. The array Px contains
// all points sorted according to x coordinates and Py contains all points
// sorted according to y coordinates
float closestUtil(Point Px[], Point Py[], int n)
{
        // If there are 2 or 3 points, then use brute force
        if (n <= 3)
                return bruteForce(Px, n);

        // Find the middle point
        int mid = n / 2;
        Point midPoint = Px[mid];


        // Divide points in y sorted array around the vertical line.
        // Assumption: All x coordinates are distinct.
        Point Pyl[mid + 1];   // y sorted points on left of vertical line
        Point Pyr[n - mid - 1];  // y sorted points on right of vertical line
        int li = 0, ri = 0;  // indexes of left and right subarrays
        for (int i = 0; i < n; i++)
        {
                if (Py[i].x <= midPoint.x)
                        Pyl[li++] = Py[i];
                else
                        Pyr[ri++] = Py[i];
        }

        // Consider the vertical line passing through the middle point
        // calculate the smallest distance dl on left of middle point and
        // dr on right side
        float dl = closestUtil(Px, Pyl, mid);
        float dr = closestUtil(Px + mid, Pyr, n - mid);

        // Find the smaller of two distances
        float d = min(dl, dr);

        // Build an array strip[] that contains points close (closer than d)
        // to the line passing through the middle point
        Point strip[n];
        int j = 0;
        for (int i = 0; i < n; i++)
                if (abs(Py[i].x - midPoint.x) < d)
                        strip[j] = Py[i], j++;

        // Find the closest points in strip.  Return the minimum of d and
closest
        // distance is strip[]
        return min(d, stripClosest(strip, j, d));
}
// The main functin that finds the smallest distance
// This method mainly uses closestUtil()
float closest_landmark_id(Point P[], int n)
{
        Point Px[n];
        Point Py[n];
        for (int i = 0; i < n; i++)
        {
                Px[i] = P[i];
                Py[i] = P[i];
        }

        qsort(Px, n, sizeof(Point), compareX);
        qsort(Py, n, sizeof(Point), compareY);

        // Use recursive function closestUtil() to find the smallest distance
        return closestUtil(Px, Py, n);
}
/*
EXAMPLE OF HOW TO USE THIS "CLOSEST 2 POINTS":
// Driver program to test above functions
int main()
        {
        Point P[] = {{2, 3}, {12, 30}, {40, 50}, {5, 1}, {12, 10}, {3, 4}};
        int n = sizeof(P) / sizeof(P[0]);
        cout << "The smallest distance is " << closest(P, n);
        return 0;
        }
*/
#endif /* HELPER_FUNCTIONS_H_ */
