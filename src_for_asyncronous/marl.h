#ifndef MARL_H_
#define MARL_H_

#include "marl_diffusion.h"

#define NUM_CHANNELS 79

typedef enum {
    E_ACTION_TYPE_DEFAULT = 0,
    E_ACTION_TYPE_AFH = 0,
    E_ACTION_TYPE_DIFFUSIVE = 1,
    E_ACTION_TYPE_MAX = 2
} E_ACTION_TYPE;

typedef enum {
    STATE_TIMER_INIT = 0,
    STATE_TIMER_WAIT_ARRIVAL,
    STATE_TIMER_WAIT_ACTIVATION,
    STATE_TIMER_RUN,
    STATE_TIMER_MAX
} E_STATE_TIMER;

typedef struct {
    int id;
    int current_channel;
    bool isCurChCollied;
    double cumulative_reward;
    double q_table[E_ACTION_TYPE_MAX][NUM_CHANNELS + 1];
    int last_channel;
    int hopping_mode;

    int cur_hopping_mode;
    int last_succeed_time; // changed to slot time unit
    int instance_time;     // changed to slot time unit
    int default_rand;
    E_STATE_TIMER eStateTimer;
    int random_no_by_fh;
    int new_best_channel;
    int best_channel_in_q;
    bool new_available_channels[79];
    int cur_best_channel;
    char logStr[64];
} Agent;

int marl_main(void);

#endif /* MARL_H_ */
