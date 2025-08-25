/*
 * marl.h
 *
 * Created on: 2025. 8. 24.
 * Author: widen
 */

#ifndef MARL_H_
#define MARL_H_

#define NUM_AGENTS 10   // Maximum number of piconets
#define NUM_CHANNELS 79 // Maximum number of frequencies
#define MAX_EPISODES 100000
#define PERTURBATION 2000

#define ALPHA 0.1
#define GAMMA 0.9
// Note: With RL-AFH, the agent barely moves from its current channel...
#define EPSILON 0.1

// Note: Mostly performs diffusive moves, with a 0.1 probability of jumping to a channel with fewer collisions.
// In a way, the latter could be considered the exploration part.
#define EPSILON_DIFF 0.1
// The agent can hop a maximum of +- HMAX channels.
// #define HMAX 3

#define NUM_DIFF 10                           // Maximum number of piconets using diffusive hopping.
#define DFH_TIMEOUT (40 * 2)                  // 2 * poll interval (80 slots, 50ms)
#define DEFAULT_DFH_INSTANSTIME (40 * 6)      // 3 * poll interval (240 slots, 160ms) (120 slots, 80ms)
#define DEFAULT_DFH_UPDATE_TIMEOUT (1600 * 2) //(1600 *14) // 2 sec

/************************************** Main operation parameters *********/
// #define DIFFUSIVE
// #define ADAPTIVE
// #define WIFI
// #define SHUFFLE
// #define DEBUG
//  Use when heatmap output is needed.
// #define HEATMAP
//  Use to view selected channel info and collision status for each node.
// #define DEBUG_CH_STATE_1 1
//  Use to see the effect of Channel map size in AFH. Only applicable to AFH.
// #define CH_MAP_SIZE
#define NONE_MODEL (0)
#define RAYLEIGH_FADING_MODEL (1)

#define PHYSICAL_MODE (RAYLEIGH_FADING_MODEL)

typedef enum
{
    E_ACTION_TYPE_DEFAULT = 0,
    E_ACTION_TYPE_AFH = 0,
    E_ACTION_TYPE_DIFFUSIVE = 1,
    E_ACTION_TYPE_MAX = 2
} E_ACTION_TYPE;

typedef enum
{
    MODE_LEGACY = 0,
    MODE_LEGACY_RL = 1,
    MODE_AFH = 2,
    MODE_AFH_RL = 3,
    MODE_DFH_RL = 4,
    MODE_MAX,
} E_MODE;

typedef enum
{
    STATE_TIMER_INIT = 0,
    STATE_TIMER_WAIT_ARRIVAL,
    STATE_TIMER_WAIT_ACTIVATION,
    STATE_TIMER_RUN,
    STATE_TIMER_MAX
} E_STATE_TIMER;

typedef struct
{
    int id;
    int current_channel;
    int isCurChCollied;
    double cumulative_reward;
    double q_table[E_ACTION_TYPE_MAX][NUM_CHANNELS + 1]; // Channel (frequency) 0 is not used.
    // Variable to check for starvation. If a channel is not used for 10 seconds, its Q-table value is reset to 0.
    int lastUsedTime[NUM_CHANNELS + 1];
    // Current channel, remembered to select the next channel in the vicinity during a diffusive action.
    int last_channel;
    int isLastChCollied;
    int last_action;
    int hopping_mode; // Can be different for each piconet.

    // Parameter to enable switching between DFH <-> LFH modes.
    int cur_hopping_mode;
    // Used to check DFH_timeout. If (cur - last) > 80 slots, a timeout occurs.
    int last_succeed_time;
    // Time until settings are applied. The time when a new best action is determined and reflected.
    int instance_time;
    int default_rand;
    E_STATE_TIMER eStateTimer;
    // Stores the random number generated for the current clock.
    int random_no_by_fh;
    // The action to be updated at instance_time.
    int new_best_channel;
    bool new_available_channels[79];

    // The best action currently in use until new_best_channel is applied.
    int cur_best_channel;
    char logStr[32];

    // --- Added physical properties ---
    double pos_x;        // x coordinate (m)
    double pos_y;        // y coordinate (m)
    double tx_power_dbm; // Transmission power (dBm)
    int interferers[NUM_AGENTS];
    int interferer_count;
    uint64_t interferers_bitmap;

} Agent;

#endif /* MARL_H_ */
