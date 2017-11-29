/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 120;

	default_random_engine gen;
	double std_x = std[0], std_y = std[1], std_theta = std[2];

	//normal distributions for sensor noise
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0; i < num_particles; ++i) {
		Particle p;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;

		particles.push_back[p];
		weights.push_back[p.weight];
	}	

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	double std_x = std[0], std_y = std[1], std_theta = std[2];

	//normal distributions for sensor noise
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	for (int i = 0; i < num_particles; ++i) {
		double yaw = particles[i].theta;

		if (fabs(yaw_rate) > .001) {
			particles[i].x += velocity/yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
        	particles[i].y += velocity/yaw_rate * (-cos(yaw + yaw_rate * delta_t) + cos(yaw));
		}
		else {
			particles[i].x += velocity * cos(yaw) * delta_t;
        	particles[i].y += velocity * sin(yaw) * delta_t;
		}
		particles[i].theta += yaw_rate * delta_t;

		//add random Gaussian noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); ++i) {
		double min_dist = numeric_limits<double>::max();
		int best_id;

		for (int j = 0; j < predicted.size(); ++i) {
			double curr_dist = dist(predicted[j].x, predicted[j].y, observations[i].x, observtions[i].y);

			if (curr_dist < min_dist) {
				min_dist = curr_dist;
				best_id = predicted.id;
			}
		}

		observations[i].id = best_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	for (int i = 0; i < num_particles; ++i) {
		//find landmarks within the sensor range
		vector<LandmarkObs> predicted;
		for (int j = 0; j < map_landmarks.landmarks_list.size(); ++j) {
			float l_id = map_landmarks.landmarks_list[j].id_i;
			float l_x = map_landmarks.landmarks_list[j].x_f;
			float l_y = map_landmarks.landmarks_list[j].y_f;

			if (dist(particles[i].x, particles[i].y, l_x, l_y) <= sensor_range)
				predicted.push_back(LandmarkObs{ l_id, l_x, l_y });
		}

		//transform observations in the car coordinates into the map coordinates
		vector<LandmarkObs> observations_map;
		for (int k = 0; k < observations.size(); ++k) {
			double x_map = particles[i].x + (cos(particles[i].theta) * observations[k].x) 
							- (sin(particles[i].theta) * observations[k].y);
			double y_map = particles[i].y + (sin(particles[i].theta) * observations[k].x) 
							+ (cos(particles[i].theta) * observations[k].y);
			observations_map.push_back(LandmarkObs{ observations[k].id, x_map, y_map});
		}

		//data association: make the particle's predictions and obvervations both in the map coordincates
		dataAssociation(predicted, observations_map);

		//update the weight for the particle's predictions: 
		//multiplication of all probabilties of the particle's predictions to each observations
cout << "predicted_size: " << predicted.size() << endl;
cout << "observations_map_size:" << observations_map.size() << endl;

		double pp_x, pp_y;
		for (auto ob : observations_map) {
			for (auto pp : predicted {
				if (pp.id == ob.id) {
					pp_x = pp.x;
					pp_y = pp.y;
				}
			}
			double weight = 1/(2*M_PI*std_x*std_y) * exp(-((pp_x - ob.x)*(pp_x - ob.x)/(2*std_x*std_x) + (pp_y - ob.y)*(pp_y - ob.y)/(2*std_y*std_y)));
			particles[i].weight *= weight;
		}
		weights[i] = particle[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> dist_id(weights.begin(), weights.end());

	vector<Particle> new_particles;
	for (int i = 0; i < num_particles; ++i) {
		int id = dist_id(gen); //generate [0, ..., num_particles - 1] with the frequence according to its weight
		new_particles.push_back(particles[ind]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
