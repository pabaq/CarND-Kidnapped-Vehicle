This project is part of Udacity's [Self-Driving-Car Nanodegree][Course]. The 
project resources and build instructions can be found [here][Project], the 
required simulator [here][Simulator].

## Vehicle localization using a particle filter

The goal of this project is to track the location and heading of a vehicle in 
real-time with a two-dimensional particle filter implemented in C++. The idea 
behind a particle filter is to continuously compare various uncertain 
observations (e.g. lidar measurements) with the positions of known landmarks on 
a map (e.g. street corners, traffic signs, buildings etc.), and to finally 
estimate one's location in respect to those landmarks.

## Introduction
A particle filter or Monte Carlo localization, is an algorithm that may be used 
to estimate the position and orientation (the pose) of a vehicle as it moves and 
senses the environment on a given map. Since the vehicle's movement may not 
always be perfectly predictable, many random guesses are generated about where 
it is going to be next. These guesses are known as the particles. 

The particles describe a distribution of likely vehicle states, with each 
particle representing a hypothesis of the vehicle's location. Whenever the 
vehicle moves, the particles are shifted in a similar way to predict its new 
state after the movement. Whenever the vehicle observes something, the particles 
are resampled, based on how well the actual sensed data correlate with the 
predicted state. Particles that are inconsistent with the observation are 
discarded and new particles are generated close to the ones that appear 
consistent. During this recursive process, the particles should converge towards 
the actual state of the vehicle.

The algorithm can be subdivided into the following steps:

1. Initialization of the particle filter
2. Prediction of the new particle states
3. Measurement update and resampling of the particles
4. Repeating steps 2. and 3. for every time step

## Initialization
The particle filter is intialized by generating the particles on the map. With 
absolutly no clue about the intial location of the vehicle, the filter could be 
initialized by scattering all particles with a uniform random distribution over 
the complete map. However, in our case GPS data is provided. Although, GPS is
not accurate enough to be used for the ongoing localization, it is precise 
enough to be used as an initial location estimate. The filter is intialized by 
randomly spawning particles around the initial GPS pose (x, y, theta) with some 
gaussian noise to consider the uncertainty in the GPS signal. Each particle 
weight is intialized to be 1 and will be adjusted in the update step.

```c++
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Number of particles to generate
  num_particles = 50;
  
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
```

## Prediction
A bicycle model is used to simulate the motion of the vehicle. At each time step 
the velocity and yaw rate are used to predict the new location of the car. Each 
particle is moved in the same manner. For example, if the car moves forward, all 
particles move forward too, based on their own pose and no matter in which way 
they point. Since no actuator is perfect, gaussian noise is added to the state 
prediction. 

Depending on the given yaw rate, the following system equations are implemented:

| Constant Yaw | Dynamic Yaw |
| :----------: | :-------------: |  	
| ![][constant] | ![][dynamic] |

```c++
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

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
```

## Update
When the vehicle observes its environment, it updates the filter's particles to 
more accuratly reflect where it is. For each particle, the vehicle computes the 
probability that, had it been at the state of the particle, it would perceive 
what its sensors have acutally sensed. A weight is assigned to each particle 
proportional to the said probability. 

In the case of our project, this can be described as follows. In the image below 
the red car represents the ground truth location of the vehicle on the map. The 
vehicle's lidar sensors continously measure the distances to the surrounding 
landmarks within the sensor range. The observations are illustrated by the red 
vectors OBS1, OBS2 and OBS3 which point to the landmarks L5, L1 and L2. These 
observations are expressed in the vehicle's coordinate system (the x-axis 
pointing in the cars heading direction, the y-axis pointing to the left). 

![][observation]

These observations are also determined for each particle in its own local 
vehicle frame. This is examplary shown for blue particle, which is somewhat 
shifted and rotated compared to the vehicles ground truth pose. The purpose of
this observation transformation is to be able to determine which landmarks each 
particle would detect if it "uses" the real vehicle's measurements in its own 
frame. For example, the real car's observation OBS1 points to landmark L5, 
whereas the same measurement in the particle's frame detects landmark L1. 

The determination of which of the landmarks is detected by which of the 
particle's observations is accomplished by a nearest neighbour association. For 
this association it is necessary to have the observations expressed in the 
global map system, since the landmark locations are only known in map 
coordinates. This coordinate transformation is performed by a translation and 
rotation as follows

![][transformation]

Having determined all the paricle observation/landmark associations, the 
particle weight can be computed. This is done by determining the 
Multivariate-Gaussian probability density in the dimensions x and y, for each 
observation/landmark association. 

![][weight]

The Multivariate-Gaussian is evaluated at the point of the transformed 
observation's position (the location to where the observation vector points). 
The mean in the x and y dimensions of the Multivariate-Gaussian are the 
coordinates of the landmark that was associated to the observation. The standard 
deviation is the uncertainty in the x and y coordinates of the particle's 
location, which was already used in the particle filter's intialization. 

The final particle weight is the product of all the Multivariate-Gaussians 
computed for each of the observation/landmark associations of this particle. 
The higher this weight is, the higher is the probability that the location of 
the particle matches the ground truth location of the vehicle. Besides, the 
weight also represents the probability of that particle being sampled in the 
upcoming resampling step.

```c++
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  
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
```

## Resampling
The resampling step can be interpreted as kind of surving of the fittest, since 
the particles with negligible weights are replaced by new particles in the 
proximity of the particles with higher weights. In other words, the particles 
that are consistent with the sensor readings are more likely to be chosen during 
resampling, probably more than once, wheras the particles inconsistent with the 
observations are likely to be ommited. This finally lets the particles converge 
towards the actual state of the vehicle.

```c++
void ParticleFilter::resample() {
   // Resampling of the particles. 
   
   // The probability of each particle to be drawn is proportional to its weight. 

  // Random sample generator
  std::default_random_engine gen;

  // Discrete distribution of particle weigths 
  // The normalization is implictily performed by the distribution function
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
```

## Project Result
![][solution]

[constant]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/bicycle_constant_yaw.png "Constant yaw"
[dynamic]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/bicycle_yaw_rate.png "Dynamic yaw"
[observation]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/observation.png "Observations"
[transformation]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/transformation.png "Transformation from vehicle to map coordinates"
[weight]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/weight.png "Multivariate-Gaussian"
[solution]: https://github.com/pabaq/CarND-Kidnapped-Vehicle/raw/master/images/solution.gif "Solution"

[Course]: https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
[Project]: https://github.com/udacity/CarND-Kidnapped-Vehicle-Project
[Simulator]: https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45
[1]: https://en.wikipedia.org/wiki/Monte_Carlo_localization
"https://en.wikipedia.org/wiki/Monte_Carlo_localization"
