#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "afh.h"
#include "marl.h"
#include "physical_model.h"
#include "marl_diffusion.h"

// Function to calculate the distance between two agents
double calculate_distance(const Agent *agent1, const Agent *agent2)
{
    double dx = agent1->pos_x - agent2->pos_x;
    double dy = agent1->pos_y - agent2->pos_y;
    return sqrt(dx * dx + dy * dy);
}

// Function to calculate path loss in dB
double calculate_path_loss_db(double distance_m)
{
    if (distance_m < 0.1)
        distance_m = 0.1;
    return REFERENCE_PATH_LOSS + 10.0 * PATH_LOSS_EXPONENT * log10(distance_m / 1.0);
}

// Function to generate power variation due to Rayleigh fading in dB
double generate_rayleigh_fading_db()
{
    double u = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double fading_power_linear = -log(u);
    return 10.0 * log10(fading_power_linear);
}

// Function to convert dBm to milliwatts (mW)
double dbm_to_mw(double dbm)
{
    return pow(10.0, dbm / 10.0);
}

// Convert SINR threshold (dB) to a linear scale
double sinr_db_to_linear(double db)
{
    return pow(10.0, db / 10.0);
}

// Function to determine packet success based on SINR
bool is_packet_successful(double sinr_linear)
{
    return sinr_linear >= sinr_db_to_linear(SINR_THRESHOLD_DB);
}

/**
 * @brief Generates a valid position for an agent, maintaining a minimum distance from existing agents.
 * @param current_agent_index The index (i) of the agent for which to generate a position.
 * @param agents              Array of existing agent information.
 * @param min_distance        The minimum separation distance to maintain (m).
 * @param max_x               The maximum value for the generated x-coordinate (length of the space).
 * @param max_y               The maximum value for the generated y-coordinate (width of the space).
 * @param new_pos_x           [out] Pointer to store the successfully generated x-coordinate.
 * @param new_pos_y           [out] Pointer to store the successfully generated y-coordinate.
 */
void generate_valid_position(
    int current_agent_index,
    const Agent agents[],
    double min_distance,
    double max_x,
    double max_y,
    double *new_pos_x,
    double *new_pos_y)
{
    bool position_ok;
    int max_retries = 1000;
    int retry_count = 0;
    double temp_pos_x, temp_pos_y;

    if (current_agent_index == 1)
    {
        temp_pos_x = ((double)rand() / RAND_MAX) * max_x;
        temp_pos_y = ((double)rand() / RAND_MAX) * max_y;
        *new_pos_x = temp_pos_x;
        *new_pos_y = temp_pos_y;
    }
    do
    {
        position_ok = true;

        // 1. Generate a new temporary position
        temp_pos_x = ((double)rand() / RAND_MAX) * max_x;
        temp_pos_y = ((double)rand() / RAND_MAX) * max_y;

        // 2. Check the distance against all previously placed agents (from index 1 to current_agent_index-1)
        for (int j = 1; j < current_agent_index; j++)
        {
            double dx = temp_pos_x - agents[j].pos_x;
            double dy = temp_pos_y - agents[j].pos_y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance < min_distance)
            {
                position_ok = false;
                break;
            }
        }

        retry_count++;
        if (retry_count > max_retries)
        {
            printf("Warning: Could not find a valid position for agent %d after %d retries. Placing it anyway.\n", current_agent_index, max_retries);
            // Prevents an infinite loop and uses the last attempted position
            break;
        }

    } while (!position_ok);

    // 3. Store the successfully found (or last attempted) position in the output pointers
    *new_pos_x = temp_pos_x;
    *new_pos_y = temp_pos_y;
}
/**
 * @brief Determines the packet success outcome for a specific receiver based on the SINR model.
 */
bool determine_packet_outcome(
    int receiver_id,
    int transmitter_id,
    const int potential_interferers[],
    int num_interferers,
    const Agent all_agents[])
{
    const Agent *receiver = &all_agents[receiver_id];
    const Agent *transmitter = &all_agents[transmitter_id];

    double signal_power_dbm;
    double fading = generate_rayleigh_fading_db();

    // 'Protagonist' scenario (self-reception)
    if (receiver_id == transmitter_id)
    {
        double path_loss = calculate_path_loss_db(MY_DEVICE_DISTANCE);
        signal_power_dbm = transmitter->tx_power_dbm - path_loss - BODY_ATTENUATION_DB + fading;
    }
    else
    { // Other agents
        double distance = calculate_distance(transmitter, receiver);
        double path_loss = calculate_path_loss_db(distance);
        signal_power_dbm = transmitter->tx_power_dbm - path_loss + fading;
    }
    double signal_power_mw = dbm_to_mw(signal_power_dbm);

    double total_interference_mw = 0.0;
    for (int i = 0; i < num_interferers; i++)
    {
        const Agent *interferer = &all_agents[potential_interferers[i]];
        double int_dist = calculate_distance(interferer, receiver);
        double int_pl = calculate_path_loss_db(int_dist);
        double int_fade = generate_rayleigh_fading_db();
        double int_power_dbm = interferer->tx_power_dbm - int_pl + int_fade;
        total_interference_mw += dbm_to_mw(int_power_dbm);
    }

    double noise_mw = dbm_to_mw(THERMAL_NOISE_DBM);
    double sinr = signal_power_mw / (total_interference_mw + noise_mw);

    return is_packet_successful(sinr);
}