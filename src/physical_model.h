/*
 * physical_model.h
 *
 * Created on: 2025. 8. 24.
 * Author: widen
 */

#ifndef PHYSICAL_MODEL_H_
#define PHYSICAL_MODEL_H_

// =================================================================
// ## Definition of Physical Model Parameters ##
// =================================================================
/*
#define SUBWAY_LENGTH 15.0       // Subway car length (m)
#define SUBWAY_WIDTH 3.0         // Subway car width (m)
#define MIN_DISTANCE 1.0         // Minimum separation distance between agents (m)
*/
// Based on a crowded scenario of 240 people: 10% -> 24 piconets, 20% -> 48 piconets
// Assuming 10 piconets, 10% density: 240/100 = 2.4 ratio, 20%: 240/50 = 4.8
#define SUBWAY_LENGTH (16.5 / 4.8) // Subway car length (m)
#define SUBWAY_WIDTH 3.1           // Subway car width (m)
#define MIN_DISTANCE 0.46          // Minimum separation distance between agents (m)

#define PATH_LOSS_EXPONENT 4.5   // Path loss exponent (crowded indoor)
#define REFERENCE_PATH_LOSS 40.0 // Path loss at 1m reference distance (dB)
#define THERMAL_NOISE_DBM -95.0  // Background noise (dBm)
#define SINR_THRESHOLD_DB 7.0    // SINR threshold (dB)

// 'Protagonist' (Agent 1) scenario
#define MY_DEVICE_DISTANCE 1.0   // Pocket-to-ear distance (m)
#define BODY_ATTENUATION_DB 15.0 // Body attenuation (dB)

// =================================================================
// ## Function Declarations (Prototypes) ##
// =================================================================

// Physical model helper functions
double calculate_distance(const Agent *agent1, const Agent *agent2);
double calculate_path_loss_db(double distance_m);
double generate_rayleigh_fading_db();
double dbm_to_mw(double dbm);
bool is_packet_successful(double sinr_linear);
void generate_valid_position(int, const Agent[], double, double, double, double *, double *);

// Main collision determination function
bool determine_packet_outcome(
    int receiver_id,
    int transmitter_id,
    const int potential_interferers[],
    int num_interferers,
    const Agent all_agents[]);

#endif /* PHYSICAL_MODEL_H_ */