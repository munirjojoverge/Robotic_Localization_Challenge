/*
 *  particle_filter.h
 *
 *  2D particle filter class.
 *  Created on: May 02, 2020
 *  Author: Munir Jojo-Verge
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

#include <array>

struct Particle {
  int id;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double weight = 0.0;
  std::vector<int> associations;

  friend bool operator<(const Particle& first, const Particle& second) {
    float w1 = first.weight;
    float w2 = second.weight;

    return (w1 < w2) ? true : false;
  }
};

class ParticleFilter {
 public:
  // Constructor
  // @param in M Number of particles
  ParticleFilter() : num_particles_(1), is_initialized_(false) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param in robotState
   * @param in robotParams
   * @param in markerLocations
   */
  void init(int num_particles, const RobotState& robotState,
            const RobotParams& robotParams,
            const FieldLocation markerLocations[NUM_LANDMARKS]);

  /**
   * motionUpdate Predicts the state for the next time step
   *   using the process model.
   * @param in delta The argument passed is the relative change in position of
   * the robot in local robot coordinates (observed by odometry model), which
   * may be subject to noise (according to motion model parameters).
   */
  void motionUpdate(const RobotState& delta);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   * of the observed measurements.
   * @param in observations Landmark observations
   */
  void updateWeights(std::vector<MarkerObservation>& observations);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const { return is_initialized_; }

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const int num_particles() const { return num_particles_; }

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const std::vector<Particle> particles_data() const { return particles_; }

  Particle get_best_particle();

  void set_num_particles(int num_particles) {
    num_particles_ = (num_particles < max_num_particles_) ? num_particles
                                                          : max_num_particles_;
  }

  void initialize_particles(const RobotState& robotState, const double& x_std,
                            const double& y_std, const double& theta_std);

  const double min_confidence_thr() const { return min_confidence_thr_; }

 private:
  // Number of particles to draw
  int num_particles_;

  // Max Number of particles allowed
  int max_num_particles_ = 3500;

  // Min confidence (weight or prob) threshold
  double min_confidence_thr_ = 0.1;

  // Flag, if filter is initialized
  bool is_initialized_ = false;

  /**
  I defined the Number of effective particles threshold for resampling as
  2/3 % of the number of particles.I will test / tune this value.
  Reference: Particle Filters for Positioning, Navigation and Tracking
  Fredrik Gustafsson, Fredrik Gunnarsson, Niclas Bergman, Urban Forssell,
  Jonas Jansson, Rickard Karlsson, Per - Johan Nordlund
  Final version for IEEE Transactions on Signal Processing.
  Special issue on Monte Carlo methods for statistical signal processing.
  */
  double Eff_Thr_Percent_ =
      0.66;  // Had NO time to perform this selective sampling

  // Set of current particles
  std::vector<Particle> particles_;

  std::vector<double> gen_odom_gauss_noise();
  double gen_noise(double mean, double std);

  void normalizeWeights();

  // Array where we will store the given landmark locations.
  MarkerObservation_World landmarkLocations_[NUM_LANDMARKS];

  void landmark_association(std::vector<MarkerObservation_World> predicted_lm,
                            std::vector<MarkerObservation_World>& observations);

  RobotParams robotParams_;

  void keep_inside_field(Particle& p);

  Particle best_particle_;

  // This is simply to save a full for loop over the particles when resampling
  // (while trying to put all the weights n one vector to sample from)
  std::vector<double> weights_;
};

#endif /* PARTICLE_FILTER_H_ */