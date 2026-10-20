// Monte Carlo localization (a particle filter) for the robot's X/Y position.
// Heading comes from the IMU; the distance sensors correct X/Y against the field walls.
// Pure math with no VEX calls, so it can be tested on a computer.
// The robot side (thread, sensors, settings) is runMcl() in motor-control.cpp.
#include <math.h>
#include <stdint.h>
#include "utils.h"
#include "mcl.h"

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
const int mcl_capacity = 1000;
double particle_x[mcl_capacity], particle_y[mcl_capacity], particle_weight[mcl_capacity];
double resampled_x[mcl_capacity], resampled_y[mcl_capacity];
int particle_count = 0;

// Running averages of how well the particles explain the readings (slow and fast),
// used to notice when the filter has lost track and needs fresh particles
double likelihood_slow = 0, likelihood_fast = 0;

uint32_t rng_state = 2463534242u;

// ============================================================================
// RANDOM NUMBERS
// ============================================================================

void mclSeed(unsigned int seed) {
  rng_state = seed == 0 ? 2463534242u : seed;
}

// Uniform random number in [0, 1) (xorshift generator, fast and good enough here)
double randomUniform() {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 17;
  rng_state ^= rng_state << 5;
  return (rng_state >> 8) * (1.0 / 16777216.0);
}

// Random number shaped like a bell curve with mean 0. Adding three uniform numbers is close
// enough for particle noise and much faster on the Brain than the exact formula (no log/sin/cos)
double randomGaussian(double sigma) {
  return sigma * 2.0 * (randomUniform() + randomUniform() + randomUniform() - 1.5);
}

// ============================================================================
// FILTER STEPS
// ============================================================================

/*
 * mclInit
 * Starts (or restarts) the filter with particles spread around a known position.
 * - particle_count: Number of particles (more = smoother but slower, up to 1000).
 * - x, y: Starting position (in inches).
 * - spread_in: How unsure the starting position is (standard deviation, in inches).
 */
void mclInit(int count, double x, double y, double spread_in) {
  particle_count = count < 1 ? 1 : (count > mcl_capacity ? mcl_capacity : count);
  for (int i = 0; i < particle_count; i++) {
    particle_x[i] = x + randomGaussian(spread_in);
    particle_y[i] = y + randomGaussian(spread_in);
    particle_weight[i] = 1.0 / particle_count;
  }
  likelihood_slow = likelihood_fast = 0;
}

/*
 * mclPredict
 * Moves every particle by the odometry movement since the last call, plus a little random
 * noise so the cloud covers the error odometry can build up (wheel slip, bumps).
 * - delta_x, delta_y: Odometry movement in field coordinates (in inches).
 * - delta_heading_deg: Heading change over the same time (in degrees); turning adds slip.
 */
void mclPredict(double delta_x, double delta_y, double delta_heading_deg) {
  double moved = hypot(delta_x, delta_y);
  double sigma = 0.01 + 0.05 * moved + 0.004 * fabs(delta_heading_deg);
  for (int i = 0; i < particle_count; i++) {
    particle_x[i] += delta_x + randomGaussian(sigma);
    particle_y[i] += delta_y + randomGaussian(sigma);
  }
}

/*
 * readingLikelihood
 * How likely a distance reading is if the wall is really `expected` inches away.
 * Mostly a bell curve around the expected distance, plus a floor for readings cut
 * short by robots or game elements in front of the wall, plus a small floor for
 * anything else. A reading much LONGER than expected only gets the small floor,
 * since the wall can't be farther than it is.
 */
double readingLikelihood(double reading, double expected, double sigma) {
  double error = reading - expected;
  double hit = exp(-0.5 * error * error / (sigma * sigma)) / (sigma * 2.5066283);
  double blocked = (reading < expected && expected > 1) ? 0.15 / expected : 0;
  double random = 0.05 / 80.0;
  return 0.8 * hit + blocked + random;
}

/*
 * resampleParticles
 * Low-variance resampling: copies particles in proportion to their weights,
 * so likely positions get more particles and unlikely ones die out.
 */
void resampleParticles() {
  double step = 1.0 / particle_count;
  double pointer = randomUniform() * step;
  double cumulative = particle_weight[0];
  int source = 0;
  for (int i = 0; i < particle_count; i++) {
    double target = pointer + i * step;
    while (target > cumulative && source < particle_count - 1) {
      source++;
      cumulative += particle_weight[source];
    }
    resampled_x[i] = particle_x[source];
    resampled_y[i] = particle_y[source];
  }
  for (int i = 0; i < particle_count; i++) {
    particle_x[i] = resampled_x[i];
    particle_y[i] = resampled_y[i];
    particle_weight[i] = 1.0 / particle_count;
  }
}

/*
 * finishCorrection
 * Last step of a correction: normalize the weights, resample once the weight is concentrated
 * in few particles, and add fresh particles if the filter seems lost.
 * Returns true if the weights were updated.
 * - total_likelihood: Sum of every particle's likelihood for this update.
 * - total_weight: Sum of the particles' new weights.
 */
bool finishCorrection(double total_likelihood, double total_weight) {
  // Track how well the readings fit on average (slow and fast averages)
  double average_likelihood = total_likelihood / particle_count;
  if (likelihood_slow <= 0) {
    likelihood_slow = likelihood_fast = average_likelihood;
  } else {
    likelihood_slow += 0.02 * (average_likelihood - likelihood_slow);
    likelihood_fast += 0.3 * (average_likelihood - likelihood_fast);
  }

  // Normalize; if nothing fits at all, keep the particles as they were
  if (total_weight <= 1e-300) {
    for (int i = 0; i < particle_count; i++) particle_weight[i] = 1.0 / particle_count;
    return false;
  }
  double sum_squares = 0;
  for (int i = 0; i < particle_count; i++) {
    particle_weight[i] /= total_weight;
    sum_squares += particle_weight[i] * particle_weight[i];
  }

  // Resample once the weight is concentrated in few particles
  double effective_particles = 1.0 / sum_squares;
  if (effective_particles < particle_count / 2.0) {
    double est_x = 0, est_y = 0, spread_x = 0, spread_y = 0;
    mclEstimate(est_x, est_y, spread_x, spread_y);
    resampleParticles();

    // If readings suddenly fit much worse than usual (robot got pushed, odometry slipped),
    // replace some particles with fresh ones around the estimate so the filter can recover
    double lost_fraction = likelihood_slow > 0 ? 1.0 - likelihood_fast / likelihood_slow : 0;
    int fresh = (int)(particle_count * fmin(fmax(lost_fraction, 0.0), 0.1));
    for (int i = 0; i < fresh; i++) {
      int index = (int)(randomUniform() * particle_count);
      particle_x[index] = est_x + randomGaussian(5.0);
      particle_y[index] = est_y + randomGaussian(5.0);
    }
  }
  return true;
}

/*
 * mclCorrect
 * Weighs every particle by how well it explains the distance readings, then resamples.
 * Readings should already be checked (sensor saw something, in range, hits a wall
 * at a reasonable angle away from corners); see runMcl().
 * Returns true if the readings were used.
 * - heading_deg: Robot heading when the readings were taken (in degrees, 0 faces +Y, clockwise positive).
 * - readings, reading_count: The usable readings.
 * - field_half_size: Distance from the field center to each wall (in inches).
 * - extra_sigma_in: Extra reading uncertainty (in inches), e.g. for leftover sensor delay error.
 * - shift_x, shift_y: How far odometry says the robot moved since the readings were taken
 *   (in inches). Distance sensor readings lag behind, so each particle is checked against
 *   where it was back then.
 */
bool mclCorrect(double heading_deg, const MclReading* readings, int reading_count, double field_half_size,
                double extra_sigma_in, double shift_x, double shift_y) {
  if (reading_count <= 0 || particle_count <= 0) {
    return false;
  }
  // Each reading's sensor offset and beam direction are the same for every particle, so work them out once
  const int max_readings = 8;
  int count = reading_count > max_readings ? max_readings : reading_count;
  double offset_x[max_readings], offset_y[max_readings], beam_x[max_readings], beam_y[max_readings];
  for (int r = 0; r < count; r++) {
    localToGlobal(readings[r].s_x, readings[r].s_y, degToRad(heading_deg), offset_x[r], offset_y[r]);
    double beam_rad = degToRad(heading_deg + readings[r].angle_deg);
    beam_x[r] = sin(beam_rad);
    beam_y[r] = cos(beam_rad);
  }

  double total_weight = 0, total_likelihood = 0;
  for (int i = 0; i < particle_count; i++) {
    // A particle outside the field can't be right
    double likelihood = (fabs(particle_x[i]) < field_half_size && fabs(particle_y[i]) < field_half_size) ? 1 : 0;
    for (int r = 0; r < count && likelihood > 0; r++) {
      bool hits_x_wall = false;
      double other_wall_gap = 0;
      double expected = beamToWall(particle_x[i] - shift_x + offset_x[r], particle_y[i] - shift_y + offset_y[r],
                                   beam_x[r], beam_y[r], field_half_size, hits_x_wall, other_wall_gap);
      // The V5 distance sensor gets less precise farther away (this also covers small heading
      // errors, since readings steeper than distance_reset_max_angle are already skipped)
      double sigma = 0.75 + 0.04 * fmax(expected, 0.0) + extra_sigma_in;
      likelihood *= readingLikelihood(readings[r].reading_in, expected, sigma);
    }
    total_likelihood += likelihood;
    particle_weight[i] *= likelihood;
    total_weight += particle_weight[i];
  }

  return finishCorrection(total_likelihood, total_weight);
}

/*
 * mclEstimate
 * Weighted average position of the particles and how spread out they are along each axis.
 * - x, y: Set to the estimated position (in inches).
 * - spread_x, spread_y: Set to the standard deviation along X and Y (in inches); small = confident.
 *   A filter that only sees one wall is confident along one axis but not the other.
 */
void mclEstimate(double& x, double& y, double& spread_x, double& spread_y) {
  double sum_x = 0, sum_y = 0, sum_weight = 0;
  for (int i = 0; i < particle_count; i++) {
    sum_x += particle_weight[i] * particle_x[i];
    sum_y += particle_weight[i] * particle_y[i];
    sum_weight += particle_weight[i];
  }
  if (sum_weight <= 0) {
    spread_x = spread_y = 1e9;
    return;
  }
  x = sum_x / sum_weight;
  y = sum_y / sum_weight;
  double variance_x = 0, variance_y = 0;
  for (int i = 0; i < particle_count; i++) {
    variance_x += particle_weight[i] * (particle_x[i] - x) * (particle_x[i] - x);
    variance_y += particle_weight[i] * (particle_y[i] - y) * (particle_y[i] - y);
  }
  spread_x = sqrt(variance_x / sum_weight);
  spread_y = sqrt(variance_y / sum_weight);
}

/*
 * nudgeAxis
 * Moves one coordinate a little toward the MCL estimate, if MCL is confident along that axis.
 */
void nudgeAxis(double& value, double estimate, double spread) {
  double error = estimate - value;
  if (spread > 2.0 || fabs(error) < 0.02 || fabs(error) > 12) {
    return;
  }
  value += fmax(-0.15, fmin(0.15, error * 0.15));
}

/*
 * mclNudge
 * Moves the odometry position toward the MCL estimate a little at a time (at most 0.15 in
 * per axis per update), so motions never see a jump. Each axis only moves when MCL is
 * confident along it, and estimates that disagree by more than a foot are ignored
 * (the filter is probably lost then).
 * - x, y: Odometry position to adjust (in inches).
 * - est_x, est_y, spread_x, spread_y: Output of mclEstimate().
 */
void mclNudge(double& x, double& y, double est_x, double est_y, double spread_x, double spread_y) {
  nudgeAxis(x, est_x, spread_x);
  nudgeAxis(y, est_y, spread_y);
}
