#include "marl.h"

//#include <crtdefs.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "afh.h"
#include "marl_diffusion.h"
#include "data_model.h"
// --- Original Code Defines and Global Variables ---
#define PERTURBATION 2000
#define ALPHA 0.1
#define GAMMA 0.9
#define EPSILON 0.1
#define EPSILON_DIFF 0.1

#define SIMUL_1SLOT (1)
#define SIMUL_AAC (2)
#define SIMUL_DOWNLOAD (3)
#define SIMUL_DATA_TYPE (SIMUL_DOWNLOAD)

#define DEFAULT_LINK_RATE	(3) //1,2,3 Mbps

#if(SIMUL_DATA_TYPE == SIMUL_AAC)
// --- Simulation Parameters for Music Streaming Profile (A2DP) ---
// For AAC @ 320kbps (High Quality Music Streaming)
// Bitrate: 320 kbps = 320,000 bits/sec
// Interval: Standard AAC frame interval is 23.2ms (23200 us)
// Data Size Calculation:
// (320,000 bits/sec) * 0.0232 sec = 7,424 bits
// 7,424 bits / 8 bits/byte = 928 bytes
#define DATA_ARRIVAL_INTERVAL_US 23200.0
#define DATA_SIZE 928
#elif(SIMUL_DATA_TYPE == SIMUL_1SLOT)
// --- Simulation parameters for 1Slot Data ---
#define DATA_ARRIVAL_INTERVAL_US 1250.0
#define DATA_SIZE 83
#endif

#define DEBUG_CH_STATE_1 (0)
//#define DEFAULT_DFH_INSTANCETIME (240)
#define DFH_TIMEOUT	(40 * 2) // 2 * poll interval (80 slot, 50ms)
#define DEFAULT_DFH_INSTANSTIME (40 * 6) // 3 * poll interval (240 slots, 160ms) (120 slots, 80ms)
#define DEFAULT_DFH_UPDATE_TIMEOUT  (1600 *2)//(1600 *14) // 2 sec

//#define WIFI
//#define HEATMAP

// --- Constant Definitions for Physical Layer (PHY) Time Calculation ---
#define BT_ACCESS_CODE_US 72.0
#define BT_HEADER_BDR_BITS 54.0
#define BT_GUARD_SYNC_TRAILER_US 18.0 //5, 11, 2
#define BT_BIT_RATE_BDR_MBPS 1.0
#define BT_BIT_RATE_EDR_MBPS 3.0
#define ACK_PACKET_TOTAL_BITS (BT_ACCESS_CODE_US + BT_HEADER_BDR_BITS)

int HMAX = 2;
int episode = 0;

typedef enum {
MODE_LEGACY = 0,
MODE_LEGACY_RL = 1,
MODE_AFH = 2,
MODE_AFH_RL = 3,
MODE_DFH_RL = 4,
MODE_MAX,
} E_MODE;

#ifdef WIFI
#define WIFI_START_EPISODE 0.3 * MAX_EPISODES
#define WIFI_END_EPISODE 0.7 * MAX_EPISODES
#define NO_OF_WIFI 3
#define WIFI_CHANNEL_START 25
#define WIFI_CHANNEL_END 47
#if NO_OF_WIFI > 1
#define WIFI_CHANNEL_11_START 50
#define WIFI_CHANNEL_11_END 72
#endif
#if NO_OF_WIFI > 2
#define WIFI_CHANNEL_1_START 1
#define WIFI_CHANNEL_1_END 21
#endif
#endif

extern int gNumOfPiconets;
extern int MAX_EPISODES;
extern const int packet_durations[];
int gNum_channels = NUM_CHANNELS;
int gNumOfAvailCh = 79;
int gModeDefault = MODE_AFH;
Agent gstAgents[MAX_PICONNETS + 1];

FILE* pcol;
FILE* pf_tput;
FILE* trajectory;
#ifdef HEATMAP
FILE* heatmapfile;
int heatmap[NUM_CHANNELS + 1];
#endif

typedef enum {
EVENT_DATA_ARRIVAL,
EVENT_TX_SLOT_START,
EVENT_TX_END,
EVENT_ACK_END,
EVENT_RL_WATCHDOG_TIMER,
EVENT_RL_APPLY_CHANNEL_TIMER,
EVENT_RL_PERIODIC_UPDATE, // Periodic RL update event
EVENT_AFH_MAP_UPDATE,
EVENT_HEATMAP_LOGGING
} EventType;

typedef struct {
double time_us;
EventType type;
int agent_id;
} Event;

#define MAX_EVENTS 100
Event event_queue[MAX_EVENTS];
int event_count = 0;

void pqueue_insert(Event event) {
	if (event_count >= MAX_EVENTS){
		return;
	}
	int i = event_count++;
	while (i > 0 && event.time_us < event_queue[(i - 1) / 2].time_us) {
		event_queue[i] = event_queue[(i - 1) / 2];
		i = (i - 1) / 2;
	}
	event_queue[i] = event;
}

Event pqueue_extract_min() {
	Event min_event = event_queue[0];
	Event last_event = event_queue[--event_count];
	int i = 0, child;
	while ((child = 2 * i + 1) < event_count) {
		if (child + 1 < event_count && event_queue[child + 1].time_us < event_queue[child].time_us)
		{
			child++;
		}
		if (last_event.time_us > event_queue[child].time_us) {
			event_queue[i] = event_queue[child];
			i = child;
		} else break;
	}
	event_queue[i] = last_event;
	return min_event;
}

bool check_us_collision(int agent_id, double start_us, double end_us, int channel) {
	Queue* other_pQ;
	Queue* pQ = &piconet_queues[agent_id];
	for (int i = 1; i <= gNumOfPiconets; i++) {
		if (i == agent_id) continue;

		other_pQ = &piconet_queues[i];
		if (other_pQ->is_transmitting && other_pQ->current_tx_packet.channel == channel) {
			if (start_us < other_pQ->current_tx_packet.tx_end_us && end_us > other_pQ->current_tx_packet.tx_start_us) {
				pQ->current_tx_packet.data_collided = true;
				other_pQ->current_tx_packet.data_collided = true;
			}
			else if (start_us < other_pQ->current_tx_packet.ack_end_us && end_us > other_pQ->current_tx_packet.ack_start_us){
				pQ->current_tx_packet.data_collided = true;
				other_pQ->current_tx_packet.data_collided = true;
			}
		}
	}

	if(pQ->current_tx_packet.data_collided == true)
	{
		return true;
	}

	#ifdef WIFI
	int current_episode = (int)(start_us / (2 * SLOT_TIME_US));
	if (current_episode >= WIFI_START_EPISODE && current_episode <= WIFI_END_EPISODE) {
		if ((channel >= WIFI_CHANNEL_START && channel <= WIFI_CHANNEL_END)
		#if NO_OF_WIFI > 1
			|| (channel >= WIFI_CHANNEL_11_START && channel <= WIFI_CHANNEL_11_END)
		#endif
		#if NO_OF_WIFI > 2
			|| (channel >= WIFI_CHANNEL_1_START && channel <= WIFI_CHANNEL_1_END)
		#endif
		) {
			return true;
		}
	}
	#endif
	return false;
}

#if(SIMUL_DATA_TYPE==SIMUL_DOWNLOAD)
void handle_data_arrival(int agent_id, double current_time_us) {
	Queue* pQ = &piconet_queues[agent_id];
	DataPacket* pNewData;

	if(pQ->data_queue_size == 0)
	{
		pNewData = &(pQ->data_queue[pQ->data_queue_rear]);
		pNewData->size = greedy_generate_master_data(&(pQ->stGreedyDlData));
		pNewData->sdu_generation_time_us = current_time_us;
		pNewData->slotType = pQ->stGreedyDlData.stMseg.slots;

		pQ->data_queue_rear = (pQ->data_queue_rear + 1) % MAX_DATA_Q;
		pQ->data_queue_size++;
	}

	// event will be scheduled when data acked
	//Event next_arrival = { current_time_us + DATA_ARRIVAL_INTERVAL_US, EVENT_DATA_ARRIVAL, agent_id };

	//pqueue_insert(next_arrival);
}
#endif

#if(SIMUL_DATA_TYPE==SIMUL_1SLOT || SIMUL_DATA_TYPE==SIMUL_AAC)
void handle_data_arrival(int agent_id, double current_time_us) {
	Queue* pQ = &piconet_queues[agent_id];
	DataPacket new_data = { .size = DATA_SIZE, .sdu_generation_time_us = current_time_us };

	new_data.slotType = 1;

	if (pQ->data_queue_size < MAX_DATA_Q) {
		pQ->data_queue[pQ->data_queue_rear] = new_data;
		pQ->data_queue_rear = (pQ->data_queue_rear + 1) % MAX_DATA_Q;
		pQ->data_queue_size++;
	} else {
		pQ->numOfLossDueToQdelay++;
	}
	Event next_arrival = { current_time_us + DATA_ARRIVAL_INTERVAL_US, EVENT_DATA_ARRIVAL, agent_id };
	pqueue_insert(next_arrival);
}
#endif


int get_best_channel_based_on_qtable(Agent* agent) {
/*
	int best_action = 1;
		for (int i = 2; i <= gNum_channels; i++) {
			if (agent->q_table[E_ACTION_TYPE_DEFAULT][i] > agent->q_table[E_ACTION_TYPE_DEFAULT][best_action]) {
				best_action = i;
			}
		}

		return best_action;
*/
	return agent->best_channel_in_q;
}

double setChMapBasedOnQtable(bool* pChMap , double* pQtable, int noOfUsedCh) {
int i, j, temp;
int index[NUM_CHANNELS + 1];
double avgQvalue = 0;

for(i = 1; i <= gNum_channels; i++) index[i] = i;

for (i = 1; i < gNum_channels ; i++) {
	for (j = 1; j < gNum_channels; j++) {
		if (pQtable[index[j]] > pQtable[index[j + 1]]) {
			temp = index[j];
			index[j] = index[j + 1];
			index[j + 1] = temp;
		}
	}
}
memset(pChMap, 0, sizeof(bool) * 79);
for(i = gNum_channels; i > gNum_channels - noOfUsedCh; i--) {
	if (i > 0 && i <= NUM_CHANNELS) {
		pChMap[index[i] - 1] = true;
		avgQvalue += pQtable[index[i]];
	}
}
return (noOfUsedCh > 0) ? (avgQvalue / noOfUsedCh) : 0.0;
}

void update_q_table_pdr(Agent* agent, int action, double reward) {
// Ignore invalid channel numbers.
	if (action <= 0 || action > NUM_CHANNELS) {
		return;
	}

	// Get the current Q-value (previous PDR estimate) for the channel.
	double current_q_value = agent->q_table[E_ACTION_TYPE_DEFAULT][action];

	// Convert the current transmission result into a PDR sample (Success: 1.0, Failure: 0.0).
	double new_sample = (reward==0) ? 1.0 : 0.0;

	// Calculate the new Q-value (new PDR estimate) using the exponential moving average formula.
	// New Q-value = (1 - Learning Rate) * Old Q-value + Learning Rate * New Sample
	double new_q_value = (1.0 - ALPHA) * current_q_value + ALPHA * new_sample;

	// Update the Q-table with the calculated new Q-value.
	agent->q_table[E_ACTION_TYPE_DEFAULT][action] = new_q_value;

	if(agent->q_table[E_ACTION_TYPE_DEFAULT][agent->best_channel_in_q] <= agent->q_table[E_ACTION_TYPE_DEFAULT][action])
	{
		agent->best_channel_in_q = action;
	}
}

int select_diffusive_best_action(Agent* agent, int last_channel)
{
    int best_action = 0;

    for (int i = 1; i <= gNum_channels; i++) {
        if (agent->q_table[E_ACTION_TYPE_DEFAULT][i] > agent->q_table[E_ACTION_TYPE_DEFAULT][best_action]) {
            best_action = i;
        }
    }

    return best_action;
}

void update_q_table(Agent* agent, int action, double reward, int current_time) {
	int best_next_action;

	if (agent->hopping_mode == MODE_LEGACY) {
		best_next_action = select_classical_action(agent, current_time+1); // Here, the Q-table is not considered at all
	}
	else if (agent->hopping_mode == MODE_DFH_RL || agent->hopping_mode == MODE_LEGACY_RL || agent->hopping_mode == MODE_AFH_RL)
	{
		best_next_action = get_best_channel_based_on_qtable(agent);
	}
	else if (agent->hopping_mode == MODE_AFH)
		best_next_action = select_classical_action(agent, current_time+1);
	else {
		printf("Unknown hopping mode. Exiting.\n");
		exit(777);
	}


agent->q_table[E_ACTION_TYPE_DEFAULT][action] += ALPHA * (reward + GAMMA * agent->q_table[E_ACTION_TYPE_DEFAULT][best_next_action] - agent->q_table[E_ACTION_TYPE_DEFAULT][action]);
//agent->q_table[E_ACTION_TYPE_DEFAULT][action] += ALPHA * (reward - agent->q_table[E_ACTION_TYPE_DEFAULT][action]);

agent->cumulative_reward = reward + GAMMA * agent->cumulative_reward;

if(agent->q_table[E_ACTION_TYPE_DEFAULT][action] >= agent->q_table[E_ACTION_TYPE_DEFAULT][agent->best_channel_in_q])
{
	agent->best_channel_in_q = action;
}
//if (agent->id == 1) fprintf(creward, "%d %lf\n", episode, agent->cumulative_reward);

}


/*
int select_classical_action(Agent* agent, int current_time) {
return (select_channel(agent->id, current_time, 2)+1);
}
*/
/*
int select_channel(uint8_t picoId, int current_time, int duration) {
S_HOPPING_INFO *pHopInfo = &(piconet_queues[picoId].stHoppingInfo);
uint8_t nextFreq;

nextFreq = calculate_next_frequency(pHopInfo->bdAddr, ((uint32_t)current_time + pHopInfo->base_clk) << 1, pHopInfo->available_channels, pHopInfo->noOfCh);


return nextFreq;
//	uint8_t calculate_next_frequency(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels) {

	//return rand() % gChannels; // Select channel randomly
}
*/
int select_channel(uint8_t picoId, int current_time, int duration) {
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[picoId].stHoppingInfo);
	S_SELECTED_CH_INFO *pChInfo;
	uint8_t nextFreq;
	pHopInfo->chInfoIndex = INC_CH_INDEX(pHopInfo->chInfoIndex);
	pChInfo = &(pHopInfo->stChInfo[pHopInfo->chInfoIndex]);

	if(((uint32_t)current_time + pHopInfo->base_clk)%2)
	{
		printf("select_channel error invalid clk");
		exit(1);
	}
	nextFreq = calculate_next_frequency(pHopInfo->bdAddr, ((uint32_t)current_time + pHopInfo->base_clk) << 1, pHopInfo->available_channels, pHopInfo->noOfCh);

	pChInfo->chId = nextFreq;
	pChInfo->startClk = current_time;
	pChInfo->endClk = current_time + duration;
	pChInfo->duration = duration;

	return nextFreq;

}
int select_classical_action(Agent* agent, int current_time) {
	return (select_channel(agent->id, current_time, 2)+1);
}
/*
int select_classical_action(Agent* agent, int current_slot_equiv) {
return calculate_next_frequency(piconet_queues[agent->id].stHoppingInfo.bdAddr, (piconet_queues[agent->id].stHoppingInfo.base_clk + current_slot_equiv), piconet_queues[agent->id].stHoppingInfo.available_channels, piconet_queues[agent->id].stHoppingInfo.noOfCh);
}
*/
int classical_action_for_rl(Agent* agent, int last_channel, double current_time_us) {
return select_classical_action(agent, (int)(current_time_us / SLOT_TIME_US));
}

int select_classical_afh_action(Agent* agent, int current_time) {
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[agent->id].stHoppingInfo);
	int numOfAvailCh = gNumOfAvailCh;
	int i;
	double avgQvalue;
	int ch_afh = select_channel(agent->id, current_time, 2)+1;

	#ifdef DIFFUSIVE // In a mixed case, AFH is set to the minimum.
	if(agent->hopping_mode == MODE_AFH)
	{
		numOfAvailCh = 20;
	}
	#endif

	if(gNumOfAvailCh > gNum_channels)
		numOfAvailCh = gNum_channels;

	// Update channel map every 2 sec based on base clock
	if(current_time == 0)
	{
		setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
		pHopInfo->noOfCh = gNum_channels;
		//printf("init=%02d==%d,%02f,%d====\n",agent->id,current_time,current_time/1600,gNum_channels);

	}
	if(((uint32_t)current_time + pHopInfo->base_clk) % (1600*2) == 0){
		//printf("=%02d==%d,%02f====\n",agent->id,current_time,current_time%1600);
	#ifdef WIFI
		for(i=0;i<79;i++)
		{
			if(pHopInfo->available_channels[i] == false && agent->q_table[E_ACTION_TYPE_DEFAULT][i+1] < 0)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i+1] *= 0.98;
			}
		}
	#endif
	#ifdef WIFI
		// If Wi-Fi turns on, ban the channel within 1 second; if it turns off, unban it after 5 seconds.
		if(pHopInfo->bWifiStart == false && episode >= (WIFI_START + (1600)))
		{
			pHopInfo->bWifiStart = true;

			for(i=WIFI_CHANNEL_START;i<=WIFI_CHANNEL_END;i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}

	#if NO_OF_WIFI>1
			for(i=WIFI_CHANNEL_11_START;i<=WIFI_CHANNEL_11_END;i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
	#endif

	#if NO_OF_WIFI>2
			for(i=WIFI_CHANNEL_1_START;i<=WIFI_CHANNEL_1_END;i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
	#endif

		}
	#endif

		avgQvalue = setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), numOfAvailCh);

	#ifdef WIFI
		// If Wi-Fi turns on, ban the channel within 1 second; if it turns off, unban it after 15 seconds.
		if(pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == false && episode >= (WIFI_END + (1600*15)))
		{
			pHopInfo->bWifiStop = true;
			numOfAvailCh = 40;

			for(i=WIFI_CHANNEL_START;i<=WIFI_CHANNEL_END;i++)
			{
				pHopInfo->available_channels[i-1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand()%100) * 0.00001);
			}
	#if NO_OF_WIFI>1
			for(i=WIFI_CHANNEL_11_START;i<=WIFI_CHANNEL_11_END;i++)
			{
				pHopInfo->available_channels[i-1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand()%100) * 0.00001);
			}
	#endif
	#if NO_OF_WIFI>2
			for(i=WIFI_CHANNEL_1_START;i<=WIFI_CHANNEL_1_END;i++)
			{
				pHopInfo->available_channels[i-1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand()%100) * 0.00001);
			}
		}
	#endif
		else
		{
			if(pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == true && episode <= (WIFI_END + (1600*15)))
			{
				numOfAvailCh = 79;
			}
		}

	#endif
		pHopInfo->noOfCh = numOfAvailCh;

		/*
		if(agent->id == 1)
		{
			for(int i=1;i<=79;i++)
				fprintf(pfQValueFile,"%02f ", agent->q_table[E_ACTION_TYPE_DEFAULT][i]);
			fprintf(pfQValueFile,"\n");
		}
		*/
	}
	#if DEBUG_CH_STATE_1
	sprintf(agent->logStr,"%02d  A ",agent->id);
	#endif

	return (ch_afh);
}


int best_channel_action(Agent* agent, int last_channel, double current_time_us) {
return agent->cur_best_channel;
}

int diffusive_action(Agent* agent, int last_channel, double current_time_us) {

int explored_channel;
int magnitude;
int sign = rand() % 2;
if (sign == 0) sign = -1; // else sign = 1;

if (HMAX == 1)
	magnitude = 1; // This is a degenerate case
else
	magnitude = (rand() % HMAX) + 1;

explored_channel = last_channel + (sign * magnitude);

if (explored_channel <= 0) { // Hyogon: The last channel is a bit strange here
	explored_channel = explored_channel + gNum_channels;
}
//else if (explored_channel > NUM_CHANNELS)
	//explored_channel %= NUM_CHANNELS;
else if (explored_channel > gNum_channels) {
	explored_channel %= gNum_channels;
}

return explored_channel;
}

int select_diffusive_rl_action(Agent* agent, int last_channel, int current_time) {
	int sign, magnitude; // Direction and magnitude of diffusive movement
	int explored_channel;
	//int random;
	int best_channel;
	bool bExpiry = false;
	bool bTriggerUpdate = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	best_channel = get_best_channel_based_on_qtable(agent);

	if(agent->random_no_by_fh > gNum_channels)
	{
		printf("invalid FH : error");
		exit(1);
	}
	//random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;
	// Check for consecutive collisions
	if(agent->isCurChCollied == true)
	{
		if(agent->hopping_mode == MODE_DFH_RL)
		{
			if(agent->cur_hopping_mode == MODE_DFH_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
				sprintf(agent->logStr,"%02d OL ", agent->id);
	#endif //DEBUG
				agent->cur_hopping_mode = MODE_LEGACY;
				agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
				agent->new_best_channel = get_best_channel_based_on_qtable(agent);
				agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
				return agent->random_no_by_fh;
			}
		}
	}
	else
	{
		agent->last_succeed_time = current_time;
	}



	if(agent->instance_time <= current_time)
		bExpiry = true;

	// Actions based on timer expiry (instance)
	if(bExpiry)
	{

		// apply new Q value info
		if(agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			//agent->cur_hopping_mode = MODE_DFH_RL;
			agent->cur_best_channel = agent->new_best_channel;
			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if(agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_DFH_RL;

		}
		// If update message transmission is not complete, wait
		else if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if(agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;

			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// generate Q update msg.
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			//agent->cur_hopping_mode = MODE_DFH_RL;
			agent->new_best_channel =  get_best_channel_based_on_qtable(agent);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
	else
	{
		if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if(agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;

			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
	}
	#if DEBUG_CH_STATE_1
	//    if(agent->id == 1)
	//    {
		switch(eTimerState)
		{
		case STATE_TIMER_RUN:
			sprintf(agent->logStr,"%02d  U.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		case STATE_TIMER_WAIT_ARRIVAL:
			sprintf(agent->logStr,"%02d  S.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		case STATE_TIMER_WAIT_ACTIVATION:
			sprintf(agent->logStr,"%02d  A.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		default:
			sprintf(agent->logStr,"%02d   .%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
		}
	//	}

	#endif //DEBUG



	if(agent->cur_hopping_mode == MODE_LEGACY)
	{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"L ");
	#endif //DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF) {  //****************************** EXPLORATION by DIFFUSION?
		// return rand() % NUM_CHANNELS; // Exploration
		sign = rand() % 2;
		if (sign == 0) sign = -1; // else sign = 1;

		if (HMAX == 1)
			magnitude = 1; // This is a degenerate case
		else
			magnitude = (rand() % HMAX) + 1;

		explored_channel = last_channel + (sign * magnitude);


		if (explored_channel <= 0) {
			explored_channel = explored_channel + gNum_channels;
		}
		//else if (explored_channel > NUM_CHANNELS)
			//explored_channel %= NUM_CHANNELS;
		else if (explored_channel > gNum_channels) {
			explored_channel %= gNum_channels;
		}
	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"E ");
	#endif //DEBUG
		return (explored_channel);
	}
	else {
	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"B ");
	#endif //DEBUG
	#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
	#endif

		return agent->cur_best_channel; // Exploitation
	}

}

int backup_select_diffusive_rl_action(Agent* agent, int last_channel, int current_time) {
	int sign, magnitude; // Direction and magnitude of diffusive movement
	int explored_channel;
	//int random;
	int best_channel;
	bool bExpiry = false;
	bool bTriggerUpdate = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	best_channel = get_best_channel_based_on_qtable(agent);

	if(agent->random_no_by_fh > gNum_channels)
	{
		printf("invalid FH : error");
		exit(1);
	}
	//random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;
	// Check for consecutive collisions
	if(agent->isCurChCollied == true)
	{
		if(agent->hopping_mode == MODE_DFH_RL)
		{
			if(agent->cur_hopping_mode == MODE_DFH_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
				sprintf(agent->logStr,"%02d OL ", agent->id);
	#endif //DEBUG
				agent->cur_hopping_mode = MODE_LEGACY;
				agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
				agent->new_best_channel = get_best_channel_based_on_qtable(agent);
				agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
				return agent->random_no_by_fh;
			}
		}
	}
	else
	{
		agent->last_succeed_time = current_time;
	}



	if(agent->instance_time <= current_time)
		bExpiry = true;

	// Actions based on timer expiry (instance)
	if(bExpiry)
	{

		// apply new Q value info
		if(agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			//agent->cur_hopping_mode = MODE_DFH_RL;
			agent->cur_best_channel = agent->new_best_channel;
			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if(agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_DFH_RL;

		}
		// If update message transmission is not complete, wait
		else if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if(agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;

			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// generate Q update msg.
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			//agent->cur_hopping_mode = MODE_DFH_RL;
			agent->new_best_channel =  get_best_channel_based_on_qtable(agent);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
	else
	{
		if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if(agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;

			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
	}
	#if DEBUG_CH_STATE_1
	//    if(agent->id == 1)
	//    {
		switch(eTimerState)
		{
		case STATE_TIMER_RUN:
			sprintf(agent->logStr,"%02d  U.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		case STATE_TIMER_WAIT_ARRIVAL:
			sprintf(agent->logStr,"%02d  S.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		case STATE_TIMER_WAIT_ACTIVATION:
			sprintf(agent->logStr,"%02d  A.%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
			break;
		default:
			sprintf(agent->logStr,"%02d   .%d (%d,%d,%d) ",agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
		}
	//	}

	#endif //DEBUG



	if(agent->cur_hopping_mode == MODE_LEGACY)
	{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"L ");
	#endif //DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF) {  //****************************** EXPLORATION by DIFFUSION?
		// return rand() % NUM_CHANNELS; // Exploration
		sign = rand() % 2;
		if (sign == 0) sign = -1; // else sign = 1;

		if (HMAX == 1)
			magnitude = 1; // This is a degenerate case
		else
			magnitude = (rand() % HMAX) + 1;

		explored_channel = last_channel + (sign * magnitude);


		if (explored_channel <= 0) {
			explored_channel = explored_channel + gNum_channels;
		}
		//else if (explored_channel > NUM_CHANNELS)
			//explored_channel %= NUM_CHANNELS;
		else if (explored_channel > gNum_channels) {
			explored_channel %= gNum_channels;
		}
	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"E ");
	#endif //DEBUG
		return (explored_channel);
	}
	else {
	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+strlen(agent->logStr),"B ");
	#endif //DEBUG
	#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
	#endif

		return agent->cur_best_channel; // Exploitation
	}

}

int select_legacy_rl_action(Agent* agent, int last_channel, int current_time) {
//int random;
bool bExpiry = false;
E_STATE_TIMER eTimerState = STATE_TIMER_INIT;

agent->random_no_by_fh = select_classical_action(agent, current_time);
//random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;

// Check for consecutive collisions
if(agent->isCurChCollied == true)
{
	if(agent->hopping_mode == MODE_LEGACY_RL)
	{
		if(agent->cur_hopping_mode == MODE_LEGACY_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
		{
#if DEBUG_CH_STATE_1
//		if(agent->id == 1)
			sprintf(agent->logStr,"%02d OL ", agent->id);
#endif //DEBUG
			agent->cur_hopping_mode = MODE_LEGACY;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			agent->new_best_channel = get_best_channel_based_on_qtable(agent);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			return agent->random_no_by_fh;
		}

	}

}
else
{
	agent->last_succeed_time = current_time;
}

if(agent->instance_time <= current_time)
	bExpiry = true;

// Actions based on timer expiry (instance)
if(bExpiry)
{
/*
	if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL && agent->isCurChCollied == false)
	{
		agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
	}
*/
	// apply new Q value info
	if(agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
	{
		//agent->cur_hopping_mode = MODE_DFH_RL;
		agent->cur_best_channel = agent->new_best_channel;
		agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
		agent->eStateTimer = STATE_TIMER_RUN;
		eTimerState = STATE_TIMER_RUN;
		if(agent->cur_hopping_mode == MODE_LEGACY)
			agent->cur_hopping_mode = MODE_LEGACY_RL;

	}
	// If update message transmission is not complete, wait
	else if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
	{
		if(agent->isCurChCollied == false)
		{
			agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
			eTimerState = STATE_TIMER_WAIT_ACTIVATION;

		}
		else
		{
			eTimerState = STATE_TIMER_INIT;
		}
	}
	// generate Q update msg.
	else // agent->eStateTimer == STATE_TIMER_RUN
	{
		//agent->cur_hopping_mode = MODE_DFH_RL;
		agent->new_best_channel =  get_best_channel_based_on_qtable(agent);
		agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
		agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
		eTimerState = STATE_TIMER_WAIT_ARRIVAL;
	}
}
#if DEBUG_CH_STATE_1
//    if(agent->id == 1)
//    {
	switch(eTimerState)
	{
	case STATE_TIMER_RUN:
		sprintf(agent->logStr,"%02d  U ",agent->id);
		break;
	case STATE_TIMER_WAIT_ARRIVAL:
		sprintf(agent->logStr,"%02d  S ",agent->id);
		break;
	case STATE_TIMER_WAIT_ACTIVATION:
		sprintf(agent->logStr,"%02d  A ",agent->id);
		break;
	default:
		sprintf(agent->logStr,"%02d    ",agent->id);
	}
//	}

#endif //DEBUG



if(agent->cur_hopping_mode == MODE_LEGACY)
{
#if DEBUG_CH_STATE_1
//		if(agent->id == 1)
	sprintf(agent->logStr+6,"L ");
#endif //DEBUG
	return agent->random_no_by_fh;
}

if ((double)rand() / RAND_MAX < EPSILON_DIFF) {  //****************************** EXPLORATION by DIFFUSION?
	// Exploration

#if DEBUG_CH_STATE_1
//				if(agent->id == 1)
	sprintf(agent->logStr+6,"E ");
#endif //DEBUG
	return (agent->random_no_by_fh);
}
else {
#if DEBUG_CH_STATE_1
//				if(agent->id == 1)
	sprintf(agent->logStr+6,"B ");
#endif //DEBUG
#ifdef DEBUG
	printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
#endif

	return agent->cur_best_channel; // Exploitation
}

}


int select_afh_rl_action(Agent* agent, int last_channel, int current_time) {
	//int random;
	bool bExpiry = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[agent->id].stHoppingInfo);
	int i;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	//random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;

	// Check for consecutive collisions
	if(agent->isCurChCollied == true)
	{
		if(agent->hopping_mode == MODE_AFH_RL)
		{
			if(agent->cur_hopping_mode == MODE_AFH_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
				sprintf(agent->logStr,"%02d OL ", agent->id);
	#endif //DEBUG
				agent->cur_hopping_mode = MODE_LEGACY;
				agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
				agent->new_best_channel = get_best_channel_based_on_qtable(agent);
	//store new used channel
				setChMapBasedOnQtable(agent->new_available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);

				agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
				return agent->random_no_by_fh;
			}

		}

	}
	else
	{
		agent->last_succeed_time = current_time;
	}

	if(agent->instance_time <= current_time)
		bExpiry = true;

	// Actions based on timer expiry (instance)
	if(bExpiry)
	{
	/*
		if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL && agent->isCurChCollied == false)
		{
			agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
		}
	*/
		// apply new Q value info
		if(agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			agent->cur_best_channel = agent->new_best_channel;
			//pHopInfo->available_channels = agent->new_available_channels;
			memcpy(pHopInfo->available_channels,agent->new_available_channels, sizeof(bool)*79);

			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if(agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_AFH_RL;

		}
		// If update message transmission is not complete, wait
		else if(agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if(agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;

			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// generate Q update msg.
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			for(i=0;i<79;i++)
			{
				if(pHopInfo->available_channels[i] == false && agent->q_table[E_ACTION_TYPE_DEFAULT][i+1] < 0)
				{
					agent->q_table[E_ACTION_TYPE_DEFAULT][i+1] *= 0.98;
				}
			}

			agent->new_best_channel =  get_best_channel_based_on_qtable(agent);
			setChMapBasedOnQtable(agent->new_available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
	#if DEBUG_CH_STATE_1
	//    if(agent->id == 1)
	//    {
		switch(eTimerState)
		{
		case STATE_TIMER_RUN:
			sprintf(agent->logStr,"%02d  U ",agent->id);
			break;
		case STATE_TIMER_WAIT_ARRIVAL:
			sprintf(agent->logStr,"%02d  S ",agent->id);
			break;
		case STATE_TIMER_WAIT_ACTIVATION:
			sprintf(agent->logStr,"%02d  A ",agent->id);
			break;
		default:
			sprintf(agent->logStr,"%02d    ",agent->id);
		}
	//	}

	#endif //DEBUG



	if(agent->cur_hopping_mode == MODE_LEGACY)
	{
	#if DEBUG_CH_STATE_1
	//		if(agent->id == 1)
		sprintf(agent->logStr+6,"L ");
	#endif //DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF) {  //****************************** EXPLORATION by DIFFUSION?
		// Exploration

	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+6,"E ");
	#endif //DEBUG
		return (agent->random_no_by_fh);
	}
	else {
	#if DEBUG_CH_STATE_1
	//				if(agent->id == 1)
		sprintf(agent->logStr+6,"B ");
	#endif //DEBUG
	#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
	#endif

		return agent->cur_best_channel; // Exploitation
	}

}


void initialize_agents(Agent agents[], int num_agents) {
    int default_rand;
    uint32_t base_clk;
    uint32_t clk_offset;

	for (int i = 1; i <= num_agents; i++) {
		default_rand = agents[i].default_rand;
		memset(&agents[i], 0, sizeof(Agent));

		agents[i].default_rand = default_rand;
		agents[i].id = i;
		agents[i].hopping_mode = gModeDefault;
		agents[i].last_channel = -1;

		if (gModeDefault == MODE_LEGACY_RL || gModeDefault == MODE_AFH_RL || gModeDefault == MODE_DFH_RL) {
			agents[i].cur_hopping_mode = MODE_LEGACY;
			agents[i].eStateTimer = STATE_TIMER_RUN;
			agents[i].instance_time = 1600 + (agents[i].default_rand % (DEFAULT_DFH_UPDATE_TIMEOUT/2));

		} else {
			agents[i].cur_hopping_mode = gModeDefault;
			agents[i].eStateTimer = STATE_TIMER_RUN;

		}

		agents[i].cur_best_channel = (agents[i].default_rand % gNum_channels) + 1;
		agents[i].best_channel_in_q = agents[i].cur_best_channel;

		for (int k = 0; k < E_ACTION_TYPE_MAX; k++) {
			for (int j = 0; j <= NUM_CHANNELS; j++) {
				agents[i].q_table[k][j] = 0.0;
			}
		}

		Queue* pQ = &piconet_queues[i];
		base_clk = pQ->stHoppingInfo.base_clk;
		clk_offset = pQ->stHoppingInfo.clk_offset;
        memset(pQ, 0, sizeof(Queue));
        greedy_init(&(pQ->stGreedyDlData), DEFAULT_LINK_RATE, 0);
    	pQ->stHoppingInfo.base_clk = base_clk;
    	pQ->stHoppingInfo.clk_offset = clk_offset;

        pQ->stHoppingInfo.bdAddr = bdAddr[i - 1];

		//memset(pQ, 0, sizeof(Queue));
		memset(pQ->stHoppingInfo.available_channels, 0, sizeof(bool) * 79);
		memset(agents[i].new_available_channels, 0,79*sizeof(bool));
		memset(pQ->stHoppingInfo.available_channels, 1, sizeof(bool) * gNum_channels);
		pQ->stHoppingInfo.noOfCh = gNum_channels;
		pQ->stHoppingInfo.bWifiStart = false;
		pQ->stHoppingInfo.bWifiStop = false;
		pQ->current_state = STATE_IDLE;

		memset(agents[i].logStr, '\0', 16);


	}

#ifdef HEATMAP
if (heatmapfile) {
	memset(heatmap, 0, sizeof(heatmap));
}
#endif
}

void run_simulation(Agent agents[], int num_agents) {
	double currentTime_us = 0.0;
	double simulation_end_time_us = (double)MAX_EPISODES * 2.0 * SLOT_TIME_US;
	event_count = 0;
	double tempStartTime_us;
	bool isWarmupComplete = false;
	int agent_id;
	Agent* pAgent;

	Queue* pQ;
	int current_slot_equiv;
	Event current_event;
	Event next_tx_slot;

	Event initial_arrival;
	Event initial_tx_slot;
	DataPacket* pPacket;

	for (int i = 1; i <= num_agents; i++) {
		S_HOPPING_INFO *pHopInfo = &(piconet_queues[i].stHoppingInfo);
		tempStartTime_us = (pHopInfo->base_clk % 2) * SLOT_TIME_US + pHopInfo->clk_offset;
		//piconet_queues[i].start_time_us = (double)(agents[i].default_rand % 1600) * SLOT_TIME_US + pHopInfo->clk_offset;
		piconet_queues[i].start_time_us = tempStartTime_us + (pHopInfo->clk_offset/2) ;
		//piconet_queues[i].start_time_us = (double)tempStartTime_us;

		initial_arrival.time_us = piconet_queues[i].start_time_us+(pHopInfo->base_clk%625);
		initial_arrival.type = EVENT_DATA_ARRIVAL;
		initial_arrival.agent_id = i;

		initial_tx_slot.time_us = tempStartTime_us;
		initial_tx_slot.type = EVENT_TX_SLOT_START;
		initial_tx_slot.agent_id = i;
		pqueue_insert(initial_arrival);
		pqueue_insert(initial_tx_slot);
		//agents[i].last_succeed_time = piconet_queues[i].start_time_us / SLOT_TIME_US;
		agents[i].last_succeed_time = 0;
		//agents[i].instance_time = piconet_queues[i].start_time_us + DEFAULT_DFH_UPDATE_TIMEOUT;

		if (agents[i].hopping_mode == MODE_AFH) {// || agents[i].hopping_mode == MODE_AFH_RL) {
			setChMapBasedOnQtable(pHopInfo->available_channels, &(agents[i].q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
			pHopInfo->noOfCh = gNum_channels;
		}

	}

	#ifdef HEATMAP
	for (int i = 1; i * 100 < MAX_EPISODES; i++) {
		Event heatmap_event = { (double)i * 100.0 * 2.0 * SLOT_TIME_US, EVENT_HEATMAP_LOGGING, -1 };
		pqueue_insert(heatmap_event);
	}
	#endif

	while (event_count > 0) {
		current_event = pqueue_extract_min();
		currentTime_us = current_event.time_us;

		if(isWarmupComplete == false && currentTime_us >= PERTURBATION * 1000)
		{
			isWarmupComplete = true;
			for (int i = 1; i <= gNumOfPiconets; i++) {
				piconet_queues[i].successful_transmissions = 0;
				piconet_queues[i].collisions = 0;
				piconet_queues[i].numOfSduGenerated = 0;
				piconet_queues[i].numOfLossDueToQdelay = 0;
				piconet_queues[i].total_delay_us = 0;
				piconet_queues[i].stGreedyDlData.dl_seg_total_bytes = 0;
				piconet_queues[i].stGreedyDlData.num_of_test_slots = (simulation_end_time_us/625.0) - (double)((PERTURBATION * 1000.0)/625.0);
				piconet_queues[i].stGreedyDlData.dl_segments = 0;

			}
		}

		if (currentTime_us > simulation_end_time_us) {
			break;
		}

		if (current_event.type == EVENT_HEATMAP_LOGGING) {
			#ifdef HEATMAP
			if(heatmapfile) {
				for (int i = 1; i <= NUM_CHANNELS; i++) fprintf(heatmapfile, "%d ", heatmap[i]);
				fprintf(heatmapfile, "\n");
				fflush(heatmapfile);
				memset(heatmap, 0, sizeof(heatmap));
			}
			#endif
			continue;
		}

		agent_id = current_event.agent_id;
		pAgent = &agents[agent_id];
		pQ = &piconet_queues[agent_id];
		current_slot_equiv = (int)(currentTime_us / SLOT_TIME_US);

		switch (current_event.type) {
			case EVENT_TX_SLOT_START: {
				if (pQ->current_state == STATE_IDLE ) {
					int next_channel;

					if (pAgent->hopping_mode == MODE_LEGACY) {
						next_channel = select_classical_action(pAgent, current_slot_equiv);
					} else if (pAgent->hopping_mode == MODE_DFH_RL) {
						next_channel = select_diffusive_rl_action(pAgent, pAgent->cur_best_channel, current_slot_equiv);
					} else if (pAgent->hopping_mode == MODE_LEGACY_RL) {
						next_channel = select_legacy_rl_action(pAgent, pAgent->current_channel, current_slot_equiv);
					} else if (pAgent->hopping_mode == MODE_AFH) {
						next_channel = select_classical_afh_action(pAgent, current_slot_equiv);
					} else if (pAgent->hopping_mode == MODE_AFH_RL) {
						next_channel = select_afh_rl_action(pAgent, pAgent->current_channel, current_slot_equiv);
					} else {
						printf("Unknown hopping mode. Exiting.\n");
						exit(777);
					}

					if(next_channel > 79)
					{
						printf("invalid channel %d %d. Exiting.\n",agent_id,next_channel);

						exit(777);
					}

					if(next_channel>gNum_channels || next_channel < 1)
					{
						printf("invalid channel %d %d. Exiting.\n",agent_id,next_channel);

						exit(777);
					}

					pAgent->last_channel = pAgent->current_channel;
					pAgent->current_channel = next_channel;

					if(pAgent->current_channel == 0)
					{
						exit(1);
					}

					#ifdef HEATMAP
					if (next_channel > 0 && next_channel <= NUM_CHANNELS) heatmap[next_channel]++;
					#endif

					pPacket = &(pQ->current_tx_packet);

					if(pQ->current_tx_packet.data_collided == false)
					{
						if(pQ->data_queue_size > 0)
						{
							memcpy(pPacket, &(pQ->data_queue[pQ->data_queue_front]), sizeof(DataPacket));


							pQ->data_queue_front = (pQ->data_queue_front + 1) % MAX_DATA_Q;
							pQ->data_queue_size--;
#if (SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
							greedy_generate_slave_response(&(pQ->stGreedyDlData));
#endif
						}
						else
						{
							next_tx_slot.time_us = currentTime_us + (2.0 * SLOT_TIME_US);
							next_tx_slot.type = EVENT_TX_SLOT_START;
							next_tx_slot.agent_id = agent_id;
							pqueue_insert(next_tx_slot);
							break;
						}
					}
					else //reTx case
					{

						pPacket->retransmission_count++;
						pPacket->data_collided = false;
					}

					pQ->current_state = STATE_TRANSMITTING;
					pQ->is_transmitting = true;

					double payload_bits = (double)(pPacket->size + 4)* 8.0; // add payload size
					double overhead_duration_us = BT_ACCESS_CODE_US + (BT_HEADER_BDR_BITS / BT_BIT_RATE_BDR_MBPS);
					double payload_duration_us = (payload_bits / BT_BIT_RATE_EDR_MBPS);
					double total_tx_duration_us = overhead_duration_us + payload_duration_us + BT_GUARD_SYNC_TRAILER_US;

					double ack_duration_us = ACK_PACKET_TOTAL_BITS / BT_BIT_RATE_BDR_MBPS;

					pPacket->channel = next_channel;
					pPacket->tx_start_us = currentTime_us;
					pPacket->tx_end_us = currentTime_us + total_tx_duration_us;


#if (SIMUL_DATA_TYPE == SIMUL_AAC)
					pPacket->ack_start_us = pPacket->tx_start_us + (SLOT_TIME_US*5.0);
#endif

#if(SIMUL_DATA_TYPE == SIMUL_1SLOT || SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
					pPacket->ack_start_us = pPacket->tx_start_us + (SLOT_TIME_US*pPacket->slotType);
#endif

#if (SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
					if(pPacket->size != pQ->stGreedyDlData.stMseg.bytes || pPacket->slotType != pQ->stGreedyDlData.stMseg.slots || \
							pQ->stGreedyDlData.stMseg.airtime_us != total_tx_duration_us)
					{
						printf("EVENT_TX_SLOT_START data miss match error");
						exit(1);
					}

					if(pQ->stGreedyDlData.stUlResp.is_ack)
					{
						ack_duration_us = airtime_us_edr(pQ->stGreedyDlData.stUlResp.user_bytes, 3);
					}

#endif

					pPacket->ack_end_us = pPacket->ack_start_us + ack_duration_us;
					//pQ->current_tx_packet = packet;

					Event tx_end = {pPacket->tx_end_us, EVENT_TX_END, agent_id};
					pqueue_insert(tx_end);
				}
				else
				{
					next_tx_slot.time_us = currentTime_us + (2.0 * SLOT_TIME_US);
					next_tx_slot.type = EVENT_TX_SLOT_START;
					next_tx_slot.agent_id = agent_id;
					pqueue_insert(next_tx_slot);
				}
				break;
			}
			case EVENT_DATA_ARRIVAL:
				handle_data_arrival(agent_id, currentTime_us);
				break;
			case EVENT_TX_END: {
				pQ->current_state = STATE_WAITING_FOR_ACK;
				bool collision = check_us_collision(agent_id, pQ->current_tx_packet.tx_start_us, pQ->current_tx_packet.tx_end_us, pQ->current_tx_packet.channel);
				Event ack_end = {pQ->current_tx_packet.ack_end_us, EVENT_ACK_END, agent_id};
				pqueue_insert(ack_end);
				break;
			}
			case EVENT_ACK_END: {
				pQ->current_state = STATE_IDLE;
				bool has_collision = check_us_collision(agent_id, pQ->current_tx_packet.ack_start_us, pQ->current_tx_packet.ack_end_us, pQ->current_tx_packet.channel);
				double reward = has_collision ? -1.0 : 0.0;
				pQ->is_transmitting = false;

				/*
				if (agent->hopping_mode == MODE_AFH)
					update_q_table_pdr(agent,pQ->current_tx_packet.channel, reward);
				else
				*/
				current_slot_equiv = (int)(pQ->current_tx_packet.ack_start_us / SLOT_TIME_US);
				update_q_table(pAgent, pQ->current_tx_packet.channel, reward, current_slot_equiv);

				if (has_collision) {
					pQ->collisions++;
					// reTx action
					pAgent->isCurChCollied = true;

				} else {
					pQ->successful_transmissions++;
					pQ->total_delay_us += (currentTime_us - pQ->current_tx_packet.sdu_generation_time_us);
					pAgent->last_succeed_time = current_slot_equiv;
					pAgent->isCurChCollied = false;
					// new Tx action
#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
					// trigger Data_arrival
					Event next_arrival = {currentTime_us + 1.0 , EVENT_DATA_ARRIVAL, agent_id };
					pqueue_insert(next_arrival);
#endif

				}

				next_tx_slot.time_us = pQ->current_tx_packet.ack_start_us + SLOT_TIME_US;
				next_tx_slot.type = EVENT_TX_SLOT_START;
				next_tx_slot.agent_id = agent_id;
				pqueue_insert(next_tx_slot);
#if DEBUG_CH_STATE_1
				// if(i == 1)
				{
					fprintf(trajectory, "%s Channel %d, Col %d\n", agents[agent_id].logStr, pQ->current_tx_packet.channel, has_collision);
				}
#endif // DEBUG
				break;
			}
			default:
				printf("Unknown event. Exiting.\n");
				exit(777);
		}
	}
}

int marl_main(void) {
	char pcol_filename[256];
	char tput_filename[256];
	char col_per_agent[256];
	char tput_per_agent[256];
	int temp_index=0;
	int temp_tput_index=0;
	char postfix[256];
	char trajectory_str[256];
	time_t now = time(NULL);

	long long total_successes = 0;
	long long total_collisions = 0;

	long long total_attempts;
	double avg_collision_rate;
	double tput;
	double total_tput;
	strftime(postfix, sizeof(postfix), "_%Y%m%d_%H%M%S.txt", localtime(&now));

	sprintf(pcol_filename,"pcol_final%s", postfix);
	sprintf(tput_filename,"tput_final%s", postfix);
	sprintf(trajectory_str,"trajectory%s", postfix);


	pcol = fopen(pcol_filename, "w");
	if (!pcol) {
		perror("Failed to open pcol file");
		return 1;
	}

#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
	pf_tput = fopen(tput_filename, "w");
	if (!pf_tput) {
		perror("Failed to open pcol file");
		return 1;
	}
#endif
	const int max_agents_to_run = 10;//(gNumOfPiconets > MAX_PICONNETS) ? MAX_PICONNETS : gNumOfPiconets;

	for(int i = 1;i<=MAX_PICONNETS;i++)
	{
		memset(&gstAgents[i], 0, sizeof(Agent));

		gstAgents[i].default_rand = rand();
	}

	//for (int na = 10; na <= max_agents_to_run; na++) {
	for (int na = 10; na <= 10; na++) {
		gNumOfPiconets = na;
		for (int nc = 20; nc <= 79; nc+=10)
		{
			gNum_channels = nc;
			for (gModeDefault = MODE_AFH; gModeDefault <= MODE_DFH_RL; gModeDefault=gModeDefault+2)
			{

				if (gModeDefault == MODE_AFH || gModeDefault == MODE_AFH_RL) {
					gNumOfAvailCh = 20;
				} else {
					gNumOfAvailCh = 79;
				}
				if (nc < gNumOfAvailCh) {
					gNumOfAvailCh = nc;
				}

				//printf("\nRunning sim: Piconets=%d, Channels=%d, Mode=%d, AvailCh=%d\n", gNumOfPiconets, gNum_channels, gModeDefault, gNumOfAvailCh);

				#ifdef HEATMAP
				char heatmap_filename[256];
				sprintf(heatmap_filename, "heatmap_p%02d_c%02d_m%d.txt", gNumOfPiconets, gNum_channels, gModeDefault);
				heatmapfile = fopen(heatmap_filename, "w");
				if (heatmapfile) {
					memset(heatmap, 0, sizeof(heatmap));
				}
				#endif
				#if DEBUG_CH_STATE_1

				trajectory = fopen(trajectory_str, "w");

				#endif


				initialize_agents(gstAgents, gNumOfPiconets);
				run_simulation(gstAgents, gNumOfPiconets);

				total_successes = 0;
				total_collisions = 0;
				total_tput = 0;
				temp_index = 0;
				temp_tput_index = 0;
				for (int i = 1; i <= gNumOfPiconets; i++) {
					total_successes += piconet_queues[i].successful_transmissions;
					total_collisions += piconet_queues[i].collisions;
					sprintf(&(col_per_agent[temp_index]),"%f ",((double)piconet_queues[i].collisions/(double)(piconet_queues[i].successful_transmissions+piconet_queues[i].collisions)));
					temp_index = strlen(col_per_agent);

#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
					tput = ((piconet_queues[i].stGreedyDlData.dl_seg_total_bytes * 8.0) / (piconet_queues[i].stGreedyDlData.num_of_test_slots/1600.0))/1000 ;
					total_tput += tput;
					sprintf(&(tput_per_agent[temp_tput_index]),"%f ", (double)tput);
					temp_tput_index = strlen(tput_per_agent);
#endif
				}
				total_attempts = total_successes + total_collisions;
				avg_collision_rate = (total_attempts > 0) ? (double)total_collisions / (double)total_attempts * 100.0 : 0.0;

				//fprintf(pcol, "Piconets: %2d, Channels: %2d, Mode: %d, Avg_Collision_Rate: %.2f%%\n", gNumOfPiconets, gNum_channels, gModeDefault, avg_collision_rate);
				//printf("Piconets: %2d, Channels: %2d, Mode: %d, Avg_Collision_Rate: %.2f%%\n", gNumOfPiconets, gNum_channels, gModeDefault, avg_collision_rate);

				printf("\nM%d %d %d %d col:%f tput:%f attempt:%d hmax%d %s", gModeDefault, nc, na, gNumOfAvailCh, (double)((double)total_collisions / (double)total_attempts) , total_tput/10.0,(int)total_attempts, HMAX, col_per_agent);
				fprintf(pcol,"\nM%d %d %d %d %f hmax%d %s", gModeDefault, nc, na, gNumOfAvailCh, (double)((double)total_collisions / (double)total_attempts), HMAX, col_per_agent);
				fflush(pcol);
#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
				fprintf(pf_tput,"\nM%d %d %d %d %f hmax%d %s", gModeDefault, nc, na, gNumOfAvailCh, total_tput/10.0,HMAX, tput_per_agent);
				fflush(pf_tput);
#endif


				#ifdef HEATMAP
				if (heatmapfile) fclose(heatmapfile);
				#endif
				#if DEBUG_CH_STATE_1
					fclose(trajectory);
				#endif
				/*
				double total_avg_collision_rate = 0;
				for (int i = 1; i <= gNumOfPiconets; i++) {
					Queue *pQ = &piconet_queues[i];
					long long total_attempts = pQ->successful_transmissions + pQ->collisions;
					double success_rate = (total_attempts > 0) ? (double)pQ->successful_transmissions / total_attempts * 100.0 : 0.0;
					double avg_delay_ms = (pQ->successful_transmissions > 0) ? pQ->total_delay_us / pQ->successful_transmissions / 1000.0 : 0.0;

					total_avg_collision_rate += (100.0 - success_rate);

					printf("Piconet %2d: Success Rate: %6.2f%%, Avg Delay: %7.3f ms, Collisions: %lld\n",
						i, success_rate, avg_delay_ms, pQ->collisions);
				}
				printf("----------------------------------------------------------------\n");
				printf("Overall Average Collision Rate: %.2f%%\n", total_avg_collision_rate / gNumOfPiconets);
				 */
				/*
				switch(gModeDefault)
				{
				case MODE_DFH_RL:
					switch(HMAX)
					{
					case 5:
						HMAX = 3;
						gModeDefault--;
						break;
					case 3:
						HMAX = 2;
						gModeDefault--;
						break;
					case 2:
						HMAX = 5;
						break;
					default:
						break;
					}
					break;
	#ifdef CH_MAP_SIZE
				case MODE_AFH:
					if(gNumOfAvailCh < 79)
					{
						gNumOfAvailCh++;
						gModeDefault--;
					}
					break;
	#endif
				default:
					break;
				}
				*/
			}
			if(nc == 70)
				nc = 69;
		}
		printf("num agent : %d\n",na);
		fprintf(pcol, "\n");
#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
		fprintf(pf_tput, "\n");
#endif
	}

	fclose(pcol);
#if(SIMUL_DATA_TYPE == SIMUL_DOWNLOAD)
	fclose(pf_tput);
#endif
	return 0;
}
