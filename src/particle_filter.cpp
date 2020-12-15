/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 50;  // TODO: Set the number of particles
  
  // Set the sizes of the particles and weights vector
  particles.resize(num_particles);
  weights.resize(num_particles);

  // Initialize the default random number generator
  std::default_random_engine gen;

  // Initialize the normal (gaussian) distributions to draw x, y and theta from
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize each particle and add it to the filter's particles vector
  for (int i = 0; i < num_particles; ++i) {

    // Define new particle
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    
    particles[i] = particle;
  }
  
  // Set initialized flag to True
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Initialize the default random number generator
  std::default_random_engine gen;

  // Propagate each particle using the velocity and yaw rate at the current timestep
  for (auto& particle: particles) {
    double theta0 = particle.theta;
    if (fabs(yaw_rate) < 1e-5) {
      // prediction for a constant yaw angle in the current timestep (yaw rate = 0)
      particle.x += velocity * delta_t * cos(theta0);
      particle.y += velocity * delta_t * sin(theta0);
    } else {   
      // prediction for a non-zero yaw rate in the current timestep
      particle.theta += yaw_rate * delta_t;
      particle.x += velocity / yaw_rate * (sin(particle.theta) - sin(theta0));
      particle.y += velocity / yaw_rate * (cos(theta0) - cos(particle.theta));
    }

    // Initialize normal distributions to add gaussian noise to the new x, y and theta
    std::normal_distribution<double> dist_x(particle.x, std_pos[0]);
    std::normal_distribution<double> dist_y(particle.y, std_pos[1]);
    std::normal_distribution<double> dist_theta(particle.theta, std_pos[2]);

    // The propagated and noisy particles state
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // Loop over all observations (the measured landmark locations transformed 
  // into map coordinates)
  for (auto& observation: observations) {
    // Reset the minimal distance to a large value
    double min_dist = std::numeric_limits<float>::max();  
    // Loop over all landmarks on the map that are in the vehicle's sensor range
    for (const auto& landmark: predicted) {
      // Distance off the current observed landmark to the current landmark on the map
      double distance = dist(observation.x, observation.y, landmark.x, landmark.y);
      // If the distance is a new minimum, the current landmark is the best 
      // match to the current observation. Associate landmark and observation.
      if ( distance <= min_dist) {
        min_dist = distance;
        observation.id = landmark.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  // Clear the weights of the previous iterations;
  weights.clear();

  double car_x, car_y, car_theta;  // particle state (hypothetical state of the car)
  double obs_car_x, obs_car_y;  // observed landmark location in the car system
  double obs_map_x, obs_map_y;  // observed landmark location in the map system
  for (auto& particle: particles) {
    car_x = particle.x;
    car_y = particle.y;
    car_theta = particle.theta;
    // Transformation of observed landmark locations from car into map system
    vector<LandmarkObs> transformed_observations;
    for (const auto& observation: observations) {
      LandmarkObs obs_map;  // Landmark object for a transformed observation
      obs_car_x = observation.x;
      obs_car_y = observation.y;
      obs_map_x = car_x + cos(car_theta) * obs_car_x - sin(car_theta) * obs_car_y;
      obs_map_y = car_y + sin(car_theta) * obs_car_x + cos(car_theta) * obs_car_y;
      obs_map.id = -1;  // dummy value; will be set in ParticleFilter::dataAssociation
      obs_map.x = obs_map_x;
      obs_map.y = obs_map_y;
      transformed_observations.push_back(obs_map);
    }

    // Determination of the landmarks in sensor range of the current particle
    vector<LandmarkObs> landmarks_in_sensor_range;
    for (const auto& landmark: map_landmarks.landmark_list) {
      double distance = dist(car_x, car_y, landmark.x_f, landmark.y_f);
      if (distance < sensor_range) {
        LandmarkObs landmark_in_range;
        landmark_in_range.id = landmark.id_i;
        landmark_in_range.x = landmark.x_f;
        landmark_in_range.y = landmark.y_f;
        landmarks_in_sensor_range.push_back(landmark_in_range);
      }
    }

    // Association of each of the observed landmarks to one within sensor range
    dataAssociation(landmarks_in_sensor_range, transformed_observations);

    // Determination of the particles new weight
    // Although the weights are treated as probabilities, no normalization 
    // is performed in this step. That is, the sum of all particle weights 
    // will not result in 1. However, the normalization will implicitly be 
    // performed by using the std::discrite_distribution in the subsquent 
    // resample method.
    particle.weight = 1;  // Reset weight
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double mu_x, mu_y;  // coordinates of nearest landmark
    for (const auto& obs_map : transformed_observations) {  // observations in map coordinates
      for (const auto& landmark: landmarks_in_sensor_range) {  // landmarks in particles range
        if (obs_map.id == landmark.id) {  // associated observation and landmark
          mu_x = landmark.x;
          mu_y = landmark.y;
          double weight = multiv_prob(sig_x, sig_y, obs_map.x, obs_map.y, mu_x, mu_y);
          particle.weight *= weight;
        }
      }
    }
    weights.push_back(particle.weight);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  // Random sample generator
  std::default_random_engine gen;

  // Discrete distribution of particle weigths (normalization performed by the function)
  std::discrete_distribution<int> dist(weights.begin(), weights.end());

  // Use the discrete distribution to draw a new sample of particles
  vector<Particle> resampled_particles;
  for (size_t i = 0; i < particles.size(); ++i) {
    int index = dist(gen);
    resampled_particles.push_back(particles[index]);
  }

  // Resample
  particles = resampled_particles;

  // Reset weights
  weights.clear();
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}