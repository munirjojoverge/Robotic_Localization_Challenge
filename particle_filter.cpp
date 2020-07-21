/*
 * particle_filter.cpp
 *
 *  2D particle filter class.
 *  Created on: May 02, 2020
 *  Author: Munir Jojo-Verge
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#define _USE_MATH_DEFINES
#define DEBUGGING false

#include <cmath>

using namespace std;

// declare a random engine to be used across multiple and various method calls
random_device rd;
static std::mt19937 randomGen(rd());
/*
 The state X of the robot is comprised of:
   x = position coordinate x (lower case)
   y = position coordinate y (lower case)
   th = heading angle

 The State Space Model is:
 X[k+1] = A X[k] + B u[k] + Wk
 Y[k] = C x[k] + Vk
 Where:
   A = state matrix
   X[k] =  State vector at sample/time k (analogous for k+1)
   B = input matrix
   u[k] =  measured inputs, For simplicity this is 0. We are not actuating
   Wk = unmeasured forces or faults. AKA System Noise
   Y[k] = measurment at sample/time k.
   C = measurment matrix.
   Vk = measurment noise.

*/

void ParticleFilter::init(int num_particles, const RobotState& robotState,
                          const RobotParams& robotParams,
                          const FieldLocation markerLocations[NUM_LANDMARKS]) {
  // 1) Set the number of particles.
  // 2) Initialize all particles to the inital Robot state and params
  // 3) Initialize all their weights to 1/n.
  // 4) Add random Gaussian noise to each particle based on the standard
  // deviation given in params.
  // 5) store the ground truth location of the landmarks.

  // NOTE: Please take a look at particle_filter.h for more information about
  // this method (and others in this file).

  num_particles_ = (num_particles < max_num_particles_)
                       ? num_particles
                       : max_num_particles_;  // Number of Particles
  // The optimal/correct initial weight (even though it will be normalized
  // later) is 1/num_particles_
  double weight_init = 1 / num_particles_;

  robotParams_ = robotParams;

  if (robotParams_.sensor_noise_distance <
          std::numeric_limits<double>::epsilon() ||
      robotParams_.sensor_noise_orientation <
          std::numeric_limits<double>::epsilon()) {
    cerr << "sensor_noise (distance and/or orientation is zero!" << endl;
  }

  robotParams_.angle_fov = robotParams_.angle_fov * M_PI / 180;
  angNorm(robotParams_.angle_fov);

  // Assuming that any adometry noise affecting Rotation is given in degrees
  robotParams_.odom_noise_rotation_from_rotation =
      robotParams_.odom_noise_rotation_from_rotation * M_PI / 180;
  angNorm(robotParams_.odom_noise_rotation_from_rotation);

  robotParams_.odom_noise_rotation_from_translation =
      robotParams_.odom_noise_rotation_from_translation * M_PI / 180;
  angNorm(robotParams_.odom_noise_rotation_from_translation);

  // Create the particles and add random Sys/Model noise
  // This might not be computationally efficient but its done this way for
  // conceptual readability.
  initialize_particles(robotState,
                       robotParams_.odom_noise_translation_from_translation,
                       robotParams_.odom_noise_translation_from_translation,
                       robotParams_.odom_noise_rotation_from_rotation);

  // copy(begin(markerLocations), end(markerLocations),
  // begin(landmarkLocations_));
  for (size_t i = 0; i < NUM_LANDMARKS; i++) {
    landmarkLocations_[i].markerIndex = i;
    landmarkLocations_[i].x = markerLocations[i].x;
    landmarkLocations_[i].y = markerLocations[i].y;
  };

  is_initialized_ = true;
}

void ParticleFilter::initialize_particles(const RobotState& robotState,
                                          const double& x_std,
                                          const double& y_std,
                                          const double& theta_std) {
  double weight_init = 1 / num_particles_;

  // If we ever call this function more than once to re-generate particles, we
  // want to make sure we have empty vectors.
  particles_.clear();
  weights_.clear();

  for (int i = 0; i < num_particles_; i++) {
    Particle particle;
    particle.id = i;
    particle.x = gen_noise(robotState.x, x_std);
    particle.y = gen_noise(robotState.y, y_std);
    particle.theta = gen_noise(robotState.theta, theta_std);
    angNorm(particle.theta);
    particle.weight = weight_init;

    keep_inside_field(particle);

    particles_.push_back(particle);
    weights_.push_back(particle.weight);
  }
}

void ParticleFilter::motionUpdate(const RobotState& delta) {
  std::cout << "Robot Delta: " << delta.x << ", " << delta.y << ", "
            << delta.theta << std::endl;

  for (auto& p : particles_) {
    p.x += gen_noise(delta.x,
                     robotParams_.odom_noise_translation_from_translation);
    p.y += gen_noise(delta.y,
                     robotParams_.odom_noise_translation_from_translation);
    p.theta +=
        gen_noise(delta.theta, robotParams_.odom_noise_rotation_from_rotation);
    angNorm(p.theta);

    // Make sure they all stay in the field. Could also use a cyclic world.
    keep_inside_field(p);
  }
}

// void ParticleFilter::motionUpdate(const RobotState& delta) {
//   std::cout << "Robot Delta: " << delta.x << ", " << delta.y << ", "
//             << delta.theta << std::endl;

//   for (auto& p : particles_) {
//     p.theta +=
//         gen_noise(delta.theta,
//         robotParams_.odom_noise_rotation_from_rotation);
//     angNorm(p.theta);
//     double range_noise = gen_noise(
//         delta.x, robotParams_.odom_noise_translation_from_translation);
//     p.x += range_noise * cos(p.theta);
//     p.y += range_noise * cos(p.theta);

//     // Make sure they all stay in the field. Could also use a cyclic world.
//     keep_inside_field(p);
//   }
// }

void ParticleFilter::keep_inside_field(Particle& p) {
  double half_field_len_m = METERS_PER_PIXEL * FIELD_LENGTH / 2;
  double half_field_wid_m = METERS_PER_PIXEL * FIELD_WIDTH / 2;

  if (p.x < -half_field_len_m)
    p.x = -half_field_len_m;
  else if (p.x > half_field_len_m)
    p.x = half_field_len_m;

  if (p.y < -half_field_wid_m)
    p.y = -half_field_wid_m;
  else if (p.y > half_field_wid_m)
    p.y = half_field_wid_m;
}

void ParticleFilter::updateWeights(
    std::vector<MarkerObservation>& observations) {
  // This is a Multivariate_normal_distribution constant that we will use below.
  const double K1 = 100 * 1.0 /
                    (2.0 * M_PI * robotParams_.sensor_noise_distance *
                     robotParams_.sensor_noise_orientation);
  const double sigma_distance =
      robotParams_.sensor_noise_distance * robotParams_.sensor_noise_distance;
  const double sigma_orientation = robotParams_.sensor_noise_orientation *
                                   robotParams_.sensor_noise_orientation;

  // Update the weights of each particle using a mult-variate Gaussian
  // distribution. You can read more about this distribution here:
  // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the ROBOT'S coordinate system. The
  // particles are located according to the World coordinate system. We will
  // need to transform between the two systems. Keep in mind that this
  // transformation requires both rotation AND translation (but no scaling). The
  // following is a good resource for the theory:
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // and the following is a good resource for the actual equation to implement
  // (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html

  for (auto& p : particles_) {
    std::vector<MarkerObservation_World> predicted_lm;
    // std::map<int, int> lm2idx;

    // 1) TRANSFORMATION: Particle frame to World frame

    // The following vector represent the transformation of the observations
    // vector w.r.t to the i - th particle. What this really means is that it
    // will hold the World coordenates of the observed/measured landmarks if the
    // particle was making the measurement and that's the reason I called it
    // particle_observations.
    std::vector<MarkerObservation_World> particle_observations;
    for (auto obs : observations) {
      MarkerObservation_World trans_obs;
      // from polar to cartesian still in robot frame
      double obs_x = obs.distance * cos(obs.orientation);
      double obs_y = obs.distance * sin(obs.orientation);

      // from robot coordiantes to world coordinates
      trans_obs.markerIndex = obs.markerIndex;
      trans_obs.x = p.x + (obs_x * cos(p.theta) - obs_y * sin(p.theta));
      trans_obs.y = p.y + (obs_x * sin(p.theta) + obs_y * cos(p.theta));

      // Again, these are the observations(measurements of landmark positions)
      // w.r.t the particle p in World coordinates
      particle_observations.push_back(trans_obs);
    }
    // 2) ASSOCIATION. Now we are trying to find which landmark does EACH
    // observation corresponds to. We will simply associate the closest one
    // (closest neighbor) This will actually give the mu(i) on the Weight Update
    // Equation based on The Multivariate-Gaussian probability Let's calculate
    // the CLOSEST LANDMARK TO EACH OBSERVATION

    // First and for efficiency I will go through all the landmarks and make a
    // list of ONLY the ones that are in sensor range (FOV). Then, after that, I
    // will do the more "computantionally heavy" Eucleadian distance calculation
    // using this list.

    // for (int k = 0; k < NUM_LANDMARKS; k++) {
    //   double x_lm = landmarkLocations_[k].x;
    //   double y_lm = landmarkLocations_[k].y;
    //   int id_lm = landmarkLocations_[k].markerIndex;

    //   double distance = dist(x_lm, y_lm, p.x, p.y);
    //   double orientation = atan2((y_lm - p.y), (x_lm - p.x));

    //   double fov_low_limit = p.theta - (robotParams_.angle_fov / 2);
    //   // angNorm(fov_low_limit);
    //   double fov_high_limit = p.theta + (robotParams_.angle_fov / 2);
    //   // angNorm(fov_high_limit);

    //   if (orientation >= fov_low_limit & orientation <= fov_high_limit) {
    //     MarkerObservation_World obs = {id_lm, x_lm, y_lm};
    //     predicted_lm.push_back(obs);
    //     // lm2idx[id_lm] = k; // This, ideally should be the same, i.e. landm
    //     at
    //     // index 0 would have an id = 0
    //   }
    // }

    for (int k = 0; k < NUM_LANDMARKS; k++) {
      double x_lm = landmarkLocations_[k].x;
      double y_lm = landmarkLocations_[k].y;
      int id_lm = landmarkLocations_[k].markerIndex;
      MarkerObservation_World obs = {id_lm, x_lm, y_lm};
      predicted_lm.push_back(obs);
    }

    landmark_association(predicted_lm, particle_observations);

    /* 3) Calculating the Particle's Final Weight
       Now we that we have done the measurement transformations and
       associations, we have all the pieces we need to calculate the particle's
       final weight. The particles final weight will be calculated as the
       product of each measurement's Multivariate-Gaussian probability. The
       Multivariate-Gaussian probability has two dimensions, x and y. The "Xi"
       of the Multivariate-Gaussian is the i-th measurement (map coordenates):
       particle_ observations. The mean (mu-i) of the Multivariate-Gaussian is
       the measurement's associated landmark position (landmark associated with
       the Xi measurment - map coordinates) and the Multivariate-Gaussian's
       standard deviation (sigma) is described by our initial uncertainty in the
       x and y ranges. The Multivariate-Gaussian is evaluated at the point of
       the transformed measurement's position.
    */

    /* compute bivariate-gaussian
    https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Bivariate_case
    */
    double weight_product = 1.0;

    for (auto obs : particle_observations) {
      int markerIndex = obs.markerIndex;

      // Error between where the particle "sees" the landmank and where the
      // landmark really is (World Coordinates)
      MarkerObservation_World lm;
      // lm = landmarkLocations_[lm2idx[markerIndex]];
      lm = landmarkLocations_[obs.markerIndex];
      double distance_err = dist(obs.x, obs.y, lm.x, lm.y);
      double orientation_err = atan2((lm.y - obs.y), (lm.x - obs.x));

      // Calculate the Probability density for this especific
      // measurment/observation and it's predicted_lm closest landmark
      double weight =
          K1 * exp(-0.5 * (((distance_err * distance_err) / (sigma_distance)) +
                           ((orientation_err * orientation_err) /
                            (sigma_orientation))));

      // multiply densities for all measurements to calculate the likelyhood
      // of this particle to be in the correct/real position.
      weight_product *= weight;
    }

    p.weight = weight_product;
  }

  // Normilize the weights
  normalizeWeights();
}

// void ParticleFilter::resample() {
//   std::vector<Particle> new_particles(num_particles_);
//   int index = gen_noise(0, 1.0) * num_particles_;
//   // cout << index << endl;
//   double beta = 0.0;
//   double mw = best_particle_.weight;
//   // cout << mw;
//   for (int i = 0; i < num_particles_; i++) {
//     beta += gen_noise(0, 1.0) * 2.0 * mw;
//     while (beta > particles_[index].weight) {
//       beta -= particles_[index].weight;
//       index = mod((index + 1), num_particles_);
//     }
//     new_particles[i] = particles_[index];
//   }
//   // for (int k = 0; k < num_particles_; k++) {
//   //   particles_[k] = new_particles[k];
//   //   // cout << p[k].show_pose() << endl;
//   // }
//   particles_ = new_particles;
// }

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to
  // their weight. NOTE: You may find helpful info on std::discrete_distribution
  // here: http//en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  /*
  What the following code specifically does is randomly, uniformaly,
  sample from the cumulative distribution of the probability distribution
  generated by the weights. When you sample randomly over this
  distribution, you will select values based upon there statistical
  probability, and thus, on average, pick values with the higher weights
  (i.e. high probability of being correct given the observation z). We
  will later see that we will select the particle with the highest
  probability (confidence) as our best location estimate.
  */
  std::vector<Particle> new_particles(num_particles_);
  discrete_distribution<> distribution_weights(weights_.begin(),
                                               weights_.end());
  for (int i = 0; i < num_particles_; i++) {
    // pick up a random particle based on their weight. Heavier particles get
    // picketup more often
    int selected_idx = distribution_weights(randomGen);
    particles_[selected_idx].id = i;
    new_particles[i] = (particles_[selected_idx]);
  }
  particles_ = new_particles;

  std::cout << "n-part: " << particles_.size() << std::endl;
}

void ParticleFilter::normalizeWeights() {
  std::cout << "==========NORMALIZING=========== " << std::endl;
  double WeightSum = 0.0;
  for (Particle p : particles_) {
    WeightSum += p.weight;
  }

  best_particle_.weight = -1.0;
  double New_WeightSum = 0.0;
  for (Particle& p : particles_) {
    p.weight /= WeightSum;
    weights_[p.id] = p.weight;
    if (p.weight > best_particle_.weight) best_particle_ = p;
    std::cout << "P " << p.id << ": " << p.x << ", " << p.y << ", " << p.theta
              << ", " << p.weight << std::endl;
    New_WeightSum += p.weight;
    if (isnan(New_WeightSum)) break;  // don't waste time
  }

  if (isnan(New_WeightSum)) {
    std::cout << "Particle filter failing." << std::endl;
    // RobotState center{best_particle.x, best_particle.y, best_particle.theta};
    // set_num_particles(num_particles_ * 1.2);
    // initialize_particles(center, METERS_PER_PIXEL * FIELD_LENGTH,
    //                      METERS_PER_PIXEL * FIELD_WIDTH, M_PI);
  }

  std::cout << "New Weight Sum = " << New_WeightSum << std::endl;
  std::cout << "Best confidence = " << best_particle_.weight << std::endl;
  std::cout << "================================ " << std::endl;
}

double ParticleFilter::gen_noise(double mean, double std) {
  // Generate the normal distributions for the noisy (either measurments or
  // Syste model noise).
  normal_distribution<double> dist(mean, std);
  return dist(randomGen);
}

vector<double> ParticleFilter::gen_odom_gauss_noise() {
  vector<double> result(4, 0);
  double std_tt = robotParams_.odom_noise_translation_from_translation;
  double std_tr = robotParams_.odom_noise_translation_from_rotation;
  double std_rt = robotParams_.odom_noise_rotation_from_translation;
  double std_rr = robotParams_.odom_noise_rotation_from_translation;

  result[0] = gen_noise(0, std_tt);
  result[1] = gen_noise(0, std_tr);
  result[2] = gen_noise(0, std_rt);
  result[3] = gen_noise(0, std_rr);
  return result;
}

void ParticleFilter::landmark_association(
    std::vector<MarkerObservation_World> predicted_lm,
    std::vector<MarkerObservation_World>& observations) {
  // Find the predicted_lm measurement that is closest to each observed
  // measurement and assign the observed measurement to this particular
  // landmark.

  for (auto& obs : observations) {
    int min = 1e6;
    int ld_id = -1;
    for (auto pred : predicted_lm) {
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      if (distance < min) {
        min = distance;
        ld_id = pred.markerIndex;
      }
    }
    obs.markerIndex = ld_id;
  }
}

Particle ParticleFilter::get_best_particle() {
  // // Sort the particles in ascending order.
  // std::sort(particles_.begin(), particles_.end());

  // // The last particle should have the greatest weight.
  // return particles_[particles_.size() - 1];
  return best_particle_;
  // Particle best_particle;
  // long double x = 0.0;
  // long double y = 0.0;
  // long double theta = 0.0;
  // long double Total_w = 0.0;
  // for (auto p : particles_) {
  //   x += (p.x * p.weight);
  //   y += (p.y * p.weight);
  //   theta += (p.theta * p.weight);
  //   Total_w += p.weight;
  // }

  // best_particle.x = x / Total_w;
  // best_particle.y = y / Total_w;
  // best_particle.theta = theta / Total_w;
  // best_particle.weight = 1.0;
  // std::cout << "Best Estimate- x: " << best_particle.x
  //           << ", y: " << best_particle.y << ", t: " << best_particle.theta
  //           << std::endl;
  // return best_particle;
}