/* * A multi-agent reinforcement learning (MARL) algorithm for
 * adaptive frequency hopping in Bluetooth piconets.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "afh.h"
#include "marl.h"
#include "marl_diffusion.h"
#include "physical_model.h"

int HMAX = 2;
int episode = 0;

#ifdef DIFFUSIVE
int gTargetCoexit;
#endif

#define WIFI_START 0.3 * MAX_EPISODES
#define WIFI_END 0.7 * MAX_EPISODES
// Note: Channels 11-50 are for simulating severe interference from two WLANs. Otherwise, set to 31.
// #define WIFI_CHANNEL_START 31
// #define WIFI_CHANNEL_END 50
#define NO_OF_WIFI 3

// Note: Channels 25-47 are for simulating severe interference from two WLANs. Otherwise, set to 31.
#define WIFI_CHANNEL_START 25
#define WIFI_CHANNEL_END 47

#if NO_OF_WIFI > 1
// WiFi channel bands: WiFi 1 (1-21), WiFi 6 (25-47), WiFi 11 (50-72)
#define WIFI_CHANNEL_11_START 50
#define WIFI_CHANNEL_11_END 72
#elif

#define WIFI_CHANNEL_11_START 80
#define WIFI_CHANNEL_11_END 80

#endif

#if NO_OF_WIFI > 2
// WiFi channel bands: WiFi 1 (1-21), WiFi 6 (25-47), WiFi 11 (50-72)
#define WIFI_CHANNEL_1_START 1
#define WIFI_CHANNEL_1_END 21
#elif
#define WIFI_CHANNEL_1_START 80
#define WIFI_CHANNEL_1_END 80
#endif

#ifdef SHUFFLE

// A map used for channel shuffling with the Fisher-Yates algorithm.
int CHANNEL_SHUFFLE[NUM_CHANNELS];

// Function to perform Fisher-Yates shuffle, which creates a maximum entropy shuffle.
void fisherYatesShuffle(int CHANNEL_SHUFFLE[], int n)
{
	// Seed the random number generator
	srand(time(NULL));

	for (int i = 0; i < NUM_CHANNELS; i++)
		CHANNEL_SHUFFLE[i] = i;

	// Start from the last element and swap one by one
	for (int i = n - 1; i > 0; i--)
	{
		// Pick a random index from 0 to i
		int j = rand() % (i + 1);

		// Swap CHANNEL_SHUFFLE[i] with the element at the random index
		int temp = CHANNEL_SHUFFLE[i];
		CHANNEL_SHUFFLE[i] = CHANNEL_SHUFFLE[j];
		CHANNEL_SHUFFLE[j] = temp;
	}
}

#endif

extern int select_channel(uint8_t picoId, int current_time, int duration);

/******************************************************************************************/
int hopping_mode = 0; // Default is legacy. 1=adaptive, 2=diffusive
int gNum_channels = NUM_CHANNELS;
int num_diff = 0;
FILE *pcol;
FILE *col_graph;
FILE *chan;
FILE *heatmapfile;
// FILE* creward;
FILE *trajectory;
FILE *pfQValueFile;

// Statistics for the number of collisions per episode.
int collision_map[NUM_AGENTS + 1];
// Statistics for the final total number of collisions.
int total_collisions[NUM_AGENTS + 1];
// Statistics for the final total number of collisions with WiFi.
int total_wifi_collisions[NUM_AGENTS + 1];
int final_collision_tally = 0;
double final_wifi_collision_tally = 0;
int prev_cols[NUM_AGENTS + 1] = {0};
int gModeDefault = MODE_AFH;
int gNumOfAvailCh = 79;

#ifdef HEATMAP
// Used to visualize the frequency hopping pattern. Analyzes one channel, hence the uppercase name.
int heatmap[NUM_CHANNELS + 1];
#endif

void initialize_agents(Agent agents[], int num_agents)
{
	// Agents also start from index 1; index 0 is unused.
	for (int i = 1; i <= num_agents; i++)
	{

		agents[i].id = i;
		// Experiment: Start all agents at the same channel to observe weaknesses of the diffusive mode.
		// Result: Performance improved even as the number of channels increased.
		// agents[i].current_channel = 0;

		// Use channel 1 instead of 0. Added "+1". Bug fix: num_channels is variable.
		agents[i].current_channel = agents[i].default_rand % gNum_channels + 1;
		collision_map[i] = 0;
		total_collisions[i] = 0;
		total_wifi_collisions[i] = 0;
		final_collision_tally = 0;

#ifdef DEBUG
		printf("agent %d uses %d\n", i, agents[i].current_channel);
#endif

		agents[i].hopping_mode = gModeDefault;
		if (gModeDefault == MODE_DFH_RL || gModeDefault == MODE_LEGACY_RL || gModeDefault == MODE_AFH_RL)
		{
			agents[i].cur_hopping_mode = MODE_LEGACY;
			agents[i].eStateTimer = STATE_TIMER_RUN;
			agents[i].instance_time = 1600 + (agents[i].default_rand % DEFAULT_DFH_UPDATE_TIMEOUT);

#ifdef DIFFUSIVE
			if (i > num_diff)
			{
				agents[i].hopping_mode = gTargetCoexit;
			}
#endif
		}
		memset(agents[i].logStr, '\0', 16);

		// Remember the last channel for the diffusive action. Initialize with a value different from current_channel.
		agents[i].last_channel = -1;
		for (int k = 0; k < E_ACTION_TYPE_MAX; k++)
		{
			// Bug fix: num_channels is variable, modified to gNum_channels + 1.
			for (int j = 1; j < gNum_channels + 1; j++)
			{
				agents[i].q_table[k][j] = 0.0;
			}
		}
		memset(piconet_queues[i].stHoppingInfo.available_channels, 0, 79 * sizeof(bool));
		memset(agents[i].new_available_channels, 0, 79 * sizeof(bool));
		memset(piconet_queues[i].stHoppingInfo.available_channels, 1, gNum_channels * sizeof(bool));
		piconet_queues[i].stHoppingInfo.noOfCh = gNum_channels;
		piconet_queues[i].stHoppingInfo.bWifiStart = false;
		piconet_queues[i].stHoppingInfo.bWifiStop = false;

		agents[i].cumulative_reward = 0.0;
	}

#ifdef HEATMAP
	// Used to visualize the frequency hopping pattern. Uppercase for a single channel analysis.
	for (int k = 1; k <= NUM_CHANNELS; k++)
		heatmap[k] = 0;
#endif
}

int select_classical_action(Agent *agent, int current_time)
{
	return (select_channel(agent->id, current_time, 2) + 1);
}
void printChMap(int id)
{
	int j;
	printf("P_ID %02d : ", id);
	for (j = 0; j < NUM_CHANNELS; j++)
		printf("%d", piconet_queues[id].stHoppingInfo.available_channels[j]);
	printf("\n");
}

double setChMapBasedOnQtable(bool *pChMap, double *pQtable, int noOfUsedCh)
{
	int i, j, temp;
	int index[gNum_channels + 1];
	double avgQvalue = 0;

	for (i = 1; i <= gNum_channels; i++)
		index[i] = i;

	// Sort indices based on Q-table values
	for (i = 1; i < gNum_channels; i++)
	{
		for (j = 1; j < gNum_channels; j++)
		{
			if (pQtable[index[j]] > pQtable[index[j + 1]])
			{
				// Swap indices
				temp = index[j];
				index[j] = index[j + 1];
				index[j + 1] = temp;
			}
		}
	}

	/*
	for(i = 1; i <= NUM_CHANNELS;i++)
	{
		printf("%d, %f\n", index[i], pQtable[index[i]]);
	}
	 */

	memset(pChMap, 0, sizeof(bool) * gNum_channels);

	for (i = gNum_channels; i > gNum_channels - noOfUsedCh; i--)
	{
		pChMap[index[i] - 1] = true;
		avgQvalue += pQtable[index[i]];
	}

#ifdef DEBUG
	for (i = 0; i < gNum_channels; i++)
	{
		printf("%d", pChMap[i]);
	}
	printf("\n");

	for (i = 1; i < NUM_AGENTS + 1; i++)
	{
		for (j = 0; j < gNum_channels; j++)
			printf("%d", piconet_queues[i].stHoppingInfo.available_channels[j]);
		printf("\n");
	}

	printf("\n");
#endif // DEBUG
	return (avgQvalue / noOfUsedCh);
}

int select_afh_rl_action(Agent *agent, int last_channel, int current_time)
{
	// int random;
	bool bExpiry = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[agent->id].stHoppingInfo);
	int i;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	// random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;

	// Check for consecutive collisions
	if (agent->isCurChCollied == true)
	{
		if (agent->hopping_mode == MODE_AFH_RL)
		{
			if (agent->cur_hopping_mode == MODE_AFH_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
#if DEBUG_CH_STATE_1
				//		if(agent->id == 1)
				sprintf(agent->logStr, "%02d OL ", agent->id);
#endif // DEBUG
				agent->cur_hopping_mode = MODE_LEGACY;
				agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
				agent->new_best_channel = get_best_channel_based_on_qtable(agent);
				// Store new used channel
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

	if (agent->instance_time <= current_time)
		bExpiry = true;

	// Action based on timer expiration (instance)
	if (bExpiry)
	{
		// Apply new Q-value information
		if (agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			agent->cur_best_channel = agent->new_best_channel;
			// pHopInfo->available_channels = agent->new_available_channels;
			memcpy(pHopInfo->available_channels, agent->new_available_channels, sizeof(bool) * 79);

			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if (agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_AFH_RL;
		}
		// If update message delivery is not complete, wait
		else if (agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if (agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;
			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// Generate Q-update message
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			for (i = 0; i < 79; i++)
			{
				if (pHopInfo->available_channels[i] == false && agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] < 0)
				{
					agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] *= 0.98;
				}
			}

			agent->new_best_channel = get_best_channel_based_on_qtable(agent);
			setChMapBasedOnQtable(agent->new_available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
#if DEBUG_CH_STATE_1
	//    if(agent->id == 1)
	//    {
	switch (eTimerState)
	{
	case STATE_TIMER_RUN:
		sprintf(agent->logStr, "%02d  U ", agent->id);
		break;
	case STATE_TIMER_WAIT_ARRIVAL:
		sprintf(agent->logStr, "%02d  S ", agent->id);
		break;
	case STATE_TIMER_WAIT_ACTIVATION:
		sprintf(agent->logStr, "%02d  A ", agent->id);
		break;
	default:
		sprintf(agent->logStr, "%02d    ", agent->id);
	}
//	}
#endif // DEBUG

	if (agent->cur_hopping_mode == MODE_LEGACY)
	{
#if DEBUG_CH_STATE_1
		//		if(agent->id == 1)
		sprintf(agent->logStr + 6, "L ");
#endif // DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF)
	{	//****************************** EXPLORATION by DIFFUSION?
		// Exploration
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + 6, "E ");
#endif // DEBUG
		return (agent->random_no_by_fh);
	}
	else
	{
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + 6, "B ");
#endif // DEBUG
#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
#endif
		return agent->cur_best_channel; // Exploitation
	}
}

int select_classical_afh_rl_action(Agent *agent, int current_time)
{
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[agent->id].stHoppingInfo);
	int numOfAvailCh = gNumOfAvailCh;
	int i;
	double avgQvalue;

#ifdef DIFFUSIVE // For mixed cases, set AFH to use the minimum number of channels.
	if (agent->hopping_mode == MODE_AFH)
	{
		numOfAvailCh = 20;
	}
#endif

	if (gNumOfAvailCh > gNum_channels)
		numOfAvailCh = gNum_channels;

	// Update channel map every 2 seconds based on the base clock.
	if (current_time == 0)
	{
		setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
		pHopInfo->noOfCh = gNum_channels;
		if (agent->hopping_mode == MODE_AFH_RL)
			agent->cur_best_channel = -1;
	}
	if (((uint32_t)current_time + pHopInfo->base_clk) % (1600 * 2) == 0)
	{
		// printf("===%d====\n",current_time);

		for (i = 0; i < 79; i++)
		{
			if (pHopInfo->available_channels[i] == false && agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] < 0)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] *= 0.98;
			}
		}

#ifdef WIFI
		// When WiFi turns on, ban its channels within 1 sec. Unban them 5 secs after WiFi turns off.
		if (pHopInfo->bWifiStart == false && episode >= (WIFI_START + (1600)))
		{
			pHopInfo->bWifiStart = true;

			for (i = WIFI_CHANNEL_START; i <= WIFI_CHANNEL_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}

#if NO_OF_WIFI > 1
			for (i = WIFI_CHANNEL_11_START; i <= WIFI_CHANNEL_11_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
#endif

#if NO_OF_WIFI > 2
			for (i = WIFI_CHANNEL_1_START; i <= WIFI_CHANNEL_1_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
#endif
		}
#endif

		avgQvalue = setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), numOfAvailCh);

		if (agent->hopping_mode == MODE_AFH_RL)
			agent->cur_best_channel = get_best_channel_based_on_qtable(agent);
#ifdef WIFI
		// When WiFi turns on, ban its channels within 1 sec. Unban them 15 secs after WiFi turns off.
		if (pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == false && episode >= (WIFI_END + (1600 * 15)))
		{
			pHopInfo->bWifiStop = true;
			numOfAvailCh = 40;

			for (i = WIFI_CHANNEL_START; i <= WIFI_CHANNEL_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
#if NO_OF_WIFI > 1
			for (i = WIFI_CHANNEL_11_START; i <= WIFI_CHANNEL_11_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
#endif
#if NO_OF_WIFI > 2
			for (i = WIFI_CHANNEL_1_START; i <= WIFI_CHANNEL_1_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
		}
#endif
		else
		{
			if (pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == true && episode <= (WIFI_END + (1600 * 15)))
			{
				numOfAvailCh = 79;
			}
		}

#endif
		pHopInfo->noOfCh = numOfAvailCh;
	}

	if (agent->hopping_mode == MODE_AFH_RL)
	{
		if ((double)rand() / RAND_MAX < EPSILON_DIFF)
		{ //****************************** EXPLORATION by AFH
			return (select_channel(agent->id, current_time, 2) + 1);
		}
		else
		{ // explicit
			if (agent->cur_best_channel > -1)
				return agent->cur_best_channel;
			else
				return (select_channel(agent->id, current_time, 2) + 1);
		}
	}

	return (select_channel(agent->id, current_time, 2) + 1);
}

int select_classical_afh_action(Agent *agent, int current_time)
{
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[agent->id].stHoppingInfo);
	int numOfAvailCh = gNumOfAvailCh;
	int i;
	double avgQvalue;
	int ch_afh = select_channel(agent->id, current_time, 2) + 1;

#ifdef DIFFUSIVE // For mixed cases, set AFH to use the minimum number of channels.
	if (agent->hopping_mode == MODE_AFH)
	{
		numOfAvailCh = 20;
	}
#endif

	if (gNumOfAvailCh > gNum_channels)
		numOfAvailCh = gNum_channels;

	// Update channel map every 2 seconds based on the base clock.
	if (current_time == 0)
	{
		setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), gNum_channels);
		pHopInfo->noOfCh = gNum_channels;
	}
	if (((uint32_t)current_time + pHopInfo->base_clk) % (1600 * 2) == 0)
	{
		// printf("===%d====\n",current_time);
#ifdef WIFI
		for (i = 0; i < 79; i++)
		{
			if (pHopInfo->available_channels[i] == false && agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] < 0)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i + 1] *= 0.98;
			}
		}
#endif
#ifdef WIFI
		// When WiFi turns on, ban its channels within 1 sec. Unban them 5 secs after WiFi turns off.
		if (pHopInfo->bWifiStart == false && episode >= (WIFI_START + (1600)))
		{
			pHopInfo->bWifiStart = true;

			for (i = WIFI_CHANNEL_START; i <= WIFI_CHANNEL_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}

#if NO_OF_WIFI > 1
			for (i = WIFI_CHANNEL_11_START; i <= WIFI_CHANNEL_11_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
#endif

#if NO_OF_WIFI > 2
			for (i = WIFI_CHANNEL_1_START; i <= WIFI_CHANNEL_1_END; i++)
			{
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = -RAND_MAX;
			}
#endif
		}
#endif

		avgQvalue = setChMapBasedOnQtable(pHopInfo->available_channels, &(agent->q_table[E_ACTION_TYPE_DEFAULT][0]), numOfAvailCh);

#ifdef WIFI
		// When WiFi turns on, ban its channels within 1 sec. Unban them 15 secs after WiFi turns off.
		if (pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == false && episode >= (WIFI_END + (1600 * 15)))
		{
			pHopInfo->bWifiStop = true;
			numOfAvailCh = 40;

			for (i = WIFI_CHANNEL_START; i <= WIFI_CHANNEL_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
#if NO_OF_WIFI > 1
			for (i = WIFI_CHANNEL_11_START; i <= WIFI_CHANNEL_11_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
#endif
#if NO_OF_WIFI > 2
			for (i = WIFI_CHANNEL_1_START; i <= WIFI_CHANNEL_1_END; i++)
			{
				pHopInfo->available_channels[i - 1] = true;
				agent->q_table[E_ACTION_TYPE_DEFAULT][i] = avgQvalue + ((rand() % 100) * 0.00001);
			}
		}
#endif
		else
		{
			if (pHopInfo->bWifiStart == true && pHopInfo->bWifiStop == true && episode <= (WIFI_END + (1600 * 15)))
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
	sprintf(agent->logStr, "%02d  A ", agent->id);
#endif

	return (ch_afh);
}

/* Experimental alternative: diffusive channel hopping */
int select_diffusive_best_action(Agent *agent, int last_channel)
{
	int best_action = 0;

	for (int i = 1; i <= gNum_channels; i++)
	{
		if (agent->q_table[E_ACTION_TYPE_DEFAULT][i] > agent->q_table[E_ACTION_TYPE_DEFAULT][best_action])
		{
			best_action = i;
		}
	}

	return best_action;
}
// DIFFUSIVE FREQUENCY HOPPING
int get_best_channel_based_on_qtable(Agent *agent)
{
	int best_action = 1;
	int i;

	for (i = 1; i <= gNum_channels; i++)
	{
		if (agent->q_table[E_ACTION_TYPE_DEFAULT][i] > agent->q_table[E_ACTION_TYPE_DEFAULT][best_action])
		{
			best_action = i;
		}
	}

	return best_action;
}

int select_diffusive_rl_action(Agent *agent, int last_channel, int current_time)
{
	// Direction and magnitude of the diffusive movement
	int sign, magnitude;
	int explored_channel;
	// int random;
	int best_channel;
	bool bExpiry = false;
	bool bTriggerUpdate = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	best_channel = get_best_channel_based_on_qtable(agent);
	// random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;
	//  Check for consecutive collisions
	if (agent->isCurChCollied == true)
	{
		if (agent->hopping_mode == MODE_DFH_RL)
		{
			if (agent->cur_hopping_mode == MODE_DFH_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
#if DEBUG_CH_STATE_1
				//		if(agent->id == 1)
				sprintf(agent->logStr, "%02d OL ", agent->id);
#endif // DEBUG
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

	if (agent->instance_time <= current_time)
		bExpiry = true;

	// Action based on timer expiration (instance)
	if (bExpiry)
	{

		// Apply new Q-value information
		if (agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			// agent->cur_hopping_mode = MODE_DFH_RL;
			agent->cur_best_channel = agent->new_best_channel;
			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if (agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_DFH_RL;
		}
		// If update message delivery is not complete, wait
		else if (agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if (agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;
			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// Generate Q-update message
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			// agent->cur_hopping_mode = MODE_DFH_RL;
			agent->new_best_channel = get_best_channel_based_on_qtable(agent);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
	else
	{
		if (agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if (agent->isCurChCollied == false)
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
	switch (eTimerState)
	{
	case STATE_TIMER_RUN:
		sprintf(agent->logStr, "%02d  U.%d (%d,%d,%d) ", agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
		break;
	case STATE_TIMER_WAIT_ARRIVAL:
		sprintf(agent->logStr, "%02d  S.%d (%d,%d,%d) ", agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
		break;
	case STATE_TIMER_WAIT_ACTIVATION:
		sprintf(agent->logStr, "%02d  A.%d (%d,%d,%d) ", agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
		break;
	default:
		sprintf(agent->logStr, "%02d   .%d (%d,%d,%d) ", agent->id, (int)bTriggerUpdate, best_channel, agent->cur_best_channel, agent->new_best_channel);
	}
//	}
#endif // DEBUG

	if (agent->cur_hopping_mode == MODE_LEGACY)
	{
#if DEBUG_CH_STATE_1
		//		if(agent->id == 1)
		sprintf(agent->logStr + strlen(agent->logStr), "L ");
#endif // DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF)
	{ //****************************** EXPLORATION by DIFFUSION?
		// return rand() % NUM_CHANNELS; // Exploration
		sign = rand() % 2;
		if (sign == 0)
			sign = -1; // else sign = 1;

		if (HMAX == 1)
			magnitude = 1; // This is a degenerate case.
		else
			magnitude = (rand() % HMAX) + 1;

		explored_channel = last_channel + (sign * magnitude);

		// Dev note: The 'last_channel' variable seems problematic here.
		if (explored_channel <= 0)
		{
			explored_channel = explored_channel + gNum_channels;
		}
		// else if (explored_channel > NUM_CHANNELS)
		// explored_channel %= NUM_CHANNELS;
		else if (explored_channel > gNum_channels)
		{
			explored_channel %= gNum_channels;
		}
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + strlen(agent->logStr), "E ");
#endif // DEBUG
		return (explored_channel);
	}
	else
	{
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + strlen(agent->logStr), "B ");
#endif // DEBUG
#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
#endif

		return agent->cur_best_channel; // Exploitation
	}
}

int select_legacy_rl_action(Agent *agent, int last_channel, int current_time)
{
	// int random;
	bool bExpiry = false;
	E_STATE_TIMER eTimerState = STATE_TIMER_INIT;

	agent->random_no_by_fh = select_classical_action(agent, current_time);
	// random = select_channel_wo_remapping(agent->id, current_time, 2) + 1;

	// Check for consecutive collisions
	if (agent->isCurChCollied == true)
	{
		if (agent->hopping_mode == MODE_LEGACY_RL)
		{
			if (agent->cur_hopping_mode == MODE_LEGACY_RL && DFH_TIMEOUT < (current_time - agent->last_succeed_time))
			{
#if DEBUG_CH_STATE_1
				//		if(agent->id == 1)
				sprintf(agent->logStr, "%02d OL ", agent->id);
#endif // DEBUG
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

	if (agent->instance_time <= current_time)
		bExpiry = true;

	// Action based on timer expiration (instance)
	if (bExpiry)
	{
		// Apply new Q-value information
		if (agent->eStateTimer == STATE_TIMER_WAIT_ACTIVATION)
		{
			// agent->cur_hopping_mode = MODE_DFH_RL;
			agent->cur_best_channel = agent->new_best_channel;
			agent->instance_time = current_time + DEFAULT_DFH_UPDATE_TIMEOUT;
			agent->eStateTimer = STATE_TIMER_RUN;
			eTimerState = STATE_TIMER_RUN;
			if (agent->cur_hopping_mode == MODE_LEGACY)
				agent->cur_hopping_mode = MODE_LEGACY_RL;
		}
		// If update message delivery is not complete, wait
		else if (agent->eStateTimer == STATE_TIMER_WAIT_ARRIVAL)
		{
			if (agent->isCurChCollied == false)
			{
				agent->eStateTimer = STATE_TIMER_WAIT_ACTIVATION;
				eTimerState = STATE_TIMER_WAIT_ACTIVATION;
			}
			else
			{
				eTimerState = STATE_TIMER_INIT;
			}
		}
		// Generate Q-update message
		else // agent->eStateTimer == STATE_TIMER_RUN
		{
			// agent->cur_hopping_mode = MODE_DFH_RL;
			agent->new_best_channel = get_best_channel_based_on_qtable(agent);
			agent->instance_time = current_time + DEFAULT_DFH_INSTANSTIME;
			agent->eStateTimer = STATE_TIMER_WAIT_ARRIVAL;
			eTimerState = STATE_TIMER_WAIT_ARRIVAL;
		}
	}
#if DEBUG_CH_STATE_1
	//    if(agent->id == 1)
	//    {
	switch (eTimerState)
	{
	case STATE_TIMER_RUN:
		sprintf(agent->logStr, "%02d  U ", agent->id);
		break;
	case STATE_TIMER_WAIT_ARRIVAL:
		sprintf(agent->logStr, "%02d  S ", agent->id);
		break;
	case STATE_TIMER_WAIT_ACTIVATION:
		sprintf(agent->logStr, "%02d  A ", agent->id);
		break;
	default:
		sprintf(agent->logStr, "%02d    ", agent->id);
	}
	//	}

#endif // DEBUG

	if (agent->cur_hopping_mode == MODE_LEGACY)
	{
#if DEBUG_CH_STATE_1
		//		if(agent->id == 1)
		sprintf(agent->logStr + 6, "L ");
#endif // DEBUG
		return agent->random_no_by_fh;
	}

	if ((double)rand() / RAND_MAX < EPSILON_DIFF)
	{	//****************************** EXPLORATION by DIFFUSION?
		// Exploration
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + 6, "E ");
#endif // DEBUG
		return (agent->random_no_by_fh);
	}
	else
	{
#if DEBUG_CH_STATE_1
		//				if(agent->id == 1)
		sprintf(agent->logStr + 6, "B ");
#endif // DEBUG
#ifdef DEBUG
		printf("Agent %d chooses action %d\n", agent->id, agent->cur_best_channel);
#endif
		return agent->cur_best_channel; // Exploitation
	}
}

int choose_best_action(Agent *agent, int channel)
{
	// Choose the best action
	if (agent->q_table[E_ACTION_TYPE_AFH][channel] == agent->q_table[E_ACTION_TYPE_DIFFUSIVE][channel])
	{
		return (rand() % 2);
		// return E_ACTION_TYPE_DIFFUSIVE;
	}
	else if (agent->q_table[E_ACTION_TYPE_AFH][channel] > agent->q_table[E_ACTION_TYPE_DIFFUSIVE][channel])
	{
		return E_ACTION_TYPE_AFH;
	}
	else
	{
		return E_ACTION_TYPE_DIFFUSIVE;
	}
}

int select_diffusive_new_action(Agent *agent, int current_channel, int current_time, int *pNext_channel)
{
	// Direction and magnitude of the diffusive movement
	int sign, magnitude;
	int explored_channel;
	int afh_chnnel = select_channel(agent->id, current_time, 2) + 1;
	bool bDiffusive;

	if ((double)afh_chnnel / 79 < EPSILON_DIFF)
	{ //****************************** EXPLORATION : random or diffusion
		// Choose action (random)
		bDiffusive = afh_chnnel % 2;
#if DEBUG_CH_STATE_1
		//                if(agent->id == 1)
		printf(" E ");
#endif // DEBUG
	}
	else // Exploitation
	{
		// Choose the best action
		bDiffusive = choose_best_action(agent, current_channel);
#if DEBUG_CH_STATE_1
		//                if(agent->id == 1)
		printf(" B ");
#endif // DEBUG
	}

	if (bDiffusive)
	{
		// return rand() % NUM_CHANNELS; // Exploration
		sign = afh_chnnel % 2;
		if (sign == 0)
			sign = -1; // else sign = 1;
		if (HMAX == 1)
			magnitude = 1; // This is a degenerate case.
		else
			while ((magnitude = rand() % HMAX) == 0)
			{
				// Hopping to the same channel is forbidden -- re-select.
				;
			}

		explored_channel = current_channel + sign * magnitude;

		// Dev note: The 'last_channel' variable seems problematic here.
		if (explored_channel <= 0)
		{
			explored_channel = explored_channel + gNum_channels;
		}
		else if (explored_channel > gNum_channels)
		{
			explored_channel %= gNum_channels;
		}
	}
	else
	{
		// explored_channel = select_channel(agent->id, current_time, 2)+1;
		explored_channel = afh_chnnel;
	}

	*pNext_channel = explored_channel;

	return (bDiffusive);
}

double calculate_reward(Agent agents[], int num_agents, int agent_id, int next_channel)
{
	int collisions = 0;
	agents[agent_id].isCurChCollied = false;

	agents[agent_id].interferer_count = 0;
	agents[agent_id].interferers_bitmap = 0;

#ifdef WIFI
	// Collision is guaranteed due to WiFi interference.
	if (episode >= WIFI_START && episode <= WIFI_END && ((next_channel >= WIFI_CHANNEL_START && next_channel <= WIFI_CHANNEL_END) || (next_channel >= WIFI_CHANNEL_11_START && next_channel <= WIFI_CHANNEL_11_END) || (next_channel >= WIFI_CHANNEL_1_START && next_channel <= WIFI_CHANNEL_1_END)))
	{
		// Other piconets are not involved, so only update the agent's own collision map.
		if (agents[agent_id].isCurChCollied == false)
		{
			collision_map[agent_id]++;
			agents[agent_id].isCurChCollied = true;
		}
	}
	else
	{ // If there is no WiFi, check for collisions with other piconets.
#endif
		for (int i = 1; i <= num_agents; i++)
		{
			if (i != agent_id && agents[i].current_channel == next_channel)
			{
				// collisions++; // Should we account for multiple collisions? In reality, this is impossible.
				// The break statement ensures we only register that a collision with one or more nodes occurred.
				collisions = 1;
#ifdef DEBUG
				{
					printf("Agent %d hits at channel %d\n", agent_id, next_channel);
					printf("Agent %d is get hit at channel %d\n", i, next_channel);
				}
#endif // DEBUG
				if (agents[agent_id].isCurChCollied == false)
				{
#if (PHYSICAL_MODE == NONE_MODEL)
					collision_map[agent_id]++;
#endif
					agents[agent_id].isCurChCollied = true;
				}

				if (agents[i].isCurChCollied == false)
				{
#if (PHYSICAL_MODE == NONE_MODEL)
					collision_map[i]++;
#endif
					agents[i].isCurChCollied = true;
				}

#if (PHYSICAL_MODE == RAYLEIGH_FADING_MODEL)

				agents[agent_id].interferers[agents[agent_id].interferer_count] = i;
				agents[agent_id].interferer_count++;

				if ((agents[i].interferers_bitmap & (1 << agent_id)) == 0)
				{
					agents[i].interferers[agents[i].interferer_count] = agent_id;
					agents[i].interferer_count++;
					agents[i].interferers_bitmap |= (1 << agent_id);
				}
#endif
			}
		}
#ifdef WIFI
	}
#endif

	// Negative reward for collisions. From an individual node's perspective, it only knows that a collision occurred,
	// not how many other nodes it collided with, so the reward is always -1.
	return -collisions;
}

void update_q_table(Agent *agent, int action, double reward, int current_time)
{
	int best_next_action;

	// Replaced by the three action types below.
	// int best_next_action = select_action(agent);
	if (agent->hopping_mode == MODE_LEGACY)
	{
		// Here, the Q-table is not consulted, but it is still being updated.
		best_next_action = select_classical_action(agent, current_time);
	}
	else if (agent->hopping_mode == MODE_DFH_RL || agent->hopping_mode == MODE_LEGACY_RL || agent->hopping_mode == MODE_AFH_RL)
	{
		// Uses diffusive action (selecting a nearby channel) as the main form of EXPLORATION.
		best_next_action = select_diffusive_best_action(agent, agent->last_channel);
	}
	else if (agent->hopping_mode == MODE_AFH)
		best_next_action = select_classical_action(agent, current_time);
	else
	{
		printf("Unknown hopping mode. Exiting.\n");
		exit(777);
	}

	agent->q_table[E_ACTION_TYPE_DEFAULT][action] += ALPHA * (reward + GAMMA * agent->q_table[E_ACTION_TYPE_DEFAULT][best_next_action] - agent->q_table[E_ACTION_TYPE_DEFAULT][action]);
	// agent->q_table[E_ACTION_TYPE_DEFAULT][action] += ALPHA * (reward - agent->q_table[E_ACTION_TYPE_DEFAULT][action]);

	agent->cumulative_reward = reward + GAMMA * agent->cumulative_reward;
	// if (agent->id == 1) fprintf(creward, "%d %lf\n", episode, agent->cumulative_reward);
}

void run_simulation(Agent agents[], int num_agents)
{
	int current_time;
	int next_channel;
	double reward;
	int action;
	// printf("Number of agents = %d\n", num_agents);

	for (episode = 1; episode <= MAX_EPISODES; episode++)
	{

		current_time = (episode - 1) * 2; // every
#ifdef DEBUG
		printf("Episode %d\n", episode);
#endif
		if (episode <= PERTURBATION)
		{
			for (int i = 1; i <= num_agents; i++)
			{
				// The initial phase is for perturbation.
				total_collisions[i] = 0;
				total_wifi_collisions[i] = 0;
				prev_cols[i] = 0;
			}
		}

		for (int i = 1; i <= num_agents; i++)
		{
			action = E_ACTION_TYPE_DEFAULT;

#ifdef DEBUG
			fprintf(chan, "Agent %d\n", i);
#endif

			// Channel collision status can only be known after all other piconets have changed channels.
			// Therefore, the Q-table must be updated before selecting a new channel.
			if (episode > 1)
			{
#if (PHYSICAL_MODE == RAYLEIGH_FADING_MODEL)

				if (agents[i].isCurChCollied == true)
				{
					if (determine_packet_outcome(i, i, agents[i].interferers, agents[i].interferer_count, agents) == false)
					{
						collision_map[i]++;
					}
					else
					{
						agents[i].isCurChCollied = false;
					}
				}
#endif
				reward = (agents[i].isCurChCollied * -1);

#if DEBUG_CH_STATE_1
				// if(i == 1)
				{
					fprintf(trajectory, "%s Channel %d, Col %d\n", agents[i].logStr, agents[i].current_channel, agents[i].isCurChCollied);
				}
#endif // DEBUG
				update_q_table(&agents[i], agents[i].current_channel, reward, current_time);
			}

			// Added classical and diffusive modes.
			if (agents[i].hopping_mode == MODE_LEGACY)
			{
				// Here, the Q-table is not consulted, but it is still being updated.
				next_channel = select_classical_action(&agents[i], current_time);
			}
			else if (agents[i].hopping_mode == MODE_DFH_RL)
			{
				// Primarily uses diffusive action (selecting a nearby channel). If a collision occurs, it consults the Q-table (below).
				next_channel = select_diffusive_rl_action(&(agents[i]), agents[i].current_channel, current_time);
			}
			else if (agents[i].hopping_mode == MODE_LEGACY_RL)
			{
				next_channel = select_legacy_rl_action(&(agents[i]), agents[i].current_channel, current_time);
			}
			else if (agents[i].hopping_mode == MODE_AFH)
			{
				next_channel = select_classical_afh_action(&agents[i], current_time);
			}
			else if (agents[i].hopping_mode == MODE_AFH_RL)
			{
				next_channel = select_afh_rl_action(&(agents[i]), agents[i].current_channel, current_time);
			}
			else
			{
				printf("Unknown hopping mode. Exiting.\n");
				exit(777);
			}

			if (next_channel > 79)
			{
				printf("invalid channel %d %d. Exiting.\n", i, next_channel);
				exit(777);
			}

			if (next_channel > gNum_channels || next_channel < 1)
			{
				printf("invalid channel %d %d. Exiting.\n", i, next_channel);
				exit(777);
			}

			// Checks for collisions on the newly selected channel.
			calculate_reward(agents, num_agents, agents[i].id, next_channel);

			// The current channel becomes the last channel, used as a reference for +-dx calculations.
			agents[i].last_channel = agents[i].current_channel;
			// The currently selected channel is recorded to check for collisions in the next step.
			agents[i].current_channel = next_channel;

			if (agents[i].current_channel == 0)
			{
				exit(1);
			}
			agents[i].last_action = action;
#ifdef HEATMAP
			// To see which channels are being used.
			heatmap[agents[i].current_channel]++;
#endif

#ifdef SHUFFLE
			fprintf(chan, "agent %d at %d %f hops to %d namely %d \n", i, episode, episode / 1000, agents[i].current_channel - 1, CHANNEL_SHUFFLE[agents[i].current_channel - 1]);
#endif
		}
#ifdef SHUFFLE
		// if (i == 1) fprintf(chan, "\n");
#endif
		// Collect collision statistics for the current episode.
		for (int i = 1; i <= num_agents; i++)
		{
			total_collisions[i] += (collision_map[i] ? 1 : 0);

#ifdef WIFI
			if (episode >= WIFI_START && episode <= WIFI_END)
				total_wifi_collisions[i] += (collision_map[i] ? 1 : 0);
#endif
		}
		// Reset collision statistics for the next episode.
		for (int i = 1; i <= num_agents; i++)
		{
			collision_map[i] = 0;
		}

#ifdef HEATMAP
		if (episode % 100 == 0)
		{
			// Note: This graph is for a single channel count, hence the uppercase NUM_CHANNELS.
			for (int i = 1; i <= NUM_CHANNELS; i++)
				fprintf(heatmapfile, "%.2f ", (float)heatmap[i]);
			fprintf(heatmapfile, "\n");
			fflush(heatmapfile);

			// Note: This graph is for a single channel count, hence the uppercase NUM_CHANNELS.
			for (int i = 1; i <= NUM_CHANNELS; i++)
				heatmap[i] = 0;

			// Let's track the increase in the number of collisions for a single agent.
			for (int i = 1; i <= NUM_AGENTS; i++)
			{
				fprintf(col_graph, "%d ", total_collisions[i] - prev_cols[i]);
				prev_cols[i] = total_collisions[i];
			}
			fprintf(col_graph, "\n");
			fflush(col_graph);
		}
#endif
	}
}

Agent gstAgents[NUM_AGENTS + 1];

int marl_main(void)
{
	char col_graph_str[128];
	char heatmapfile_str[128];
	char postfix_str[64];
	char trajectory_str[128];
	char col_per_agent[256];
	char *pPostStr;
	int temp_index = 0;

	float result_pcol[3];
	time_t now;
	struct tm *t;
	char filename[256];

	time(&now);
	t = localtime(&now);

	// Create a string in YYYYMMDD_HHMMSS format.
#if (PHYSICAL_MODE == RAYLEIGH_FADING_MODEL)
	strftime(filename, sizeof(filename), "pcol_%Y%m%d_%H%M%S_RAYLEIGH.txt", t);
#else
	strftime(filename, sizeof(filename), "pcol_%Y%m%d_%H%M%S.txt", t);
#endif
	srand(time(NULL));

	pcol = fopen(filename, "w");
	//    pfQValueFile = fopen("qvalue.txt","w");

	// Store default values to ensure identical frequency hopping regardless of hopping mode.
	for (int i = 1; i <= NUM_AGENTS; i++)
	{
		gstAgents[i].default_rand = rand();
		// --- Initialize physical properties for a subway environment ---
#if (PHYSICAL_MODE == RAYLEIGH_FADING_MODEL)
		generate_valid_position(i, gstAgents, MIN_DISTANCE, SUBWAY_LENGTH, SUBWAY_WIDTH, &(gstAgents[i].pos_x), &(gstAgents[i].pos_y));
		gstAgents[i].tx_power_dbm = 4.0; // BT Class 2
#endif
	}
#ifdef SHUFFLE
	fisherYatesShuffle(CHANNEL_SHUFFLE, NUM_CHANNELS);
#endif

	// To be used for the overall simulation (normal conditions).
	for (int na = 10; na <= NUM_AGENTS; na++)
	// For partial simulation (to observe behavior with a specific number of agents).
	// for (int na = NUM_AGENTS; na <= NUM_AGENTS; na++)
	{
		gNumOfAvailCh = 20;
#ifdef DIFFUSIVE
		for (gTargetCoexit = MODE_AFH; gTargetCoexit < MODE_LEGACY_RL + 1; gTargetCoexit++)
		{

			// To observe the effect of the number of channels.
			for (int nc = 40; nc <= NUM_CHANNELS; nc += 39)
			{
#else
		// To observe the effect of the number of channels.
		for (int nc = 20; nc <= 79; nc = nc + 10)
		{
#endif
				// For partial simulation (to observe behavior with a specific number of channels).
				// for (int nc = NUM_CHANNELS; nc <= NUM_CHANNELS; nc++) {
				gNum_channels = nc;

				// To iterate through an increasing number of diffusive piconets (0 is for baseline).
				// for (int nd = 0; nd <= NUM_AGENTS; nd++) {
				// To observe a specific number of diffusive piconets (default: all are diffusive).
				for (int nd = NUM_AGENTS; nd <= NUM_AGENTS; nd++)
				{
					for (gModeDefault = MODE_LEGACY; gModeDefault < MODE_DFH_RL + 1; gModeDefault++)
					{

#ifndef CH_MAP_SIZE // To observe the effect of the channel map.
						if (gModeDefault == MODE_AFH || gModeDefault == MODE_AFH_RL || gModeDefault == MODE_AFH_RL)
							gNumOfAvailCh = 20;
						else
						{
							gNumOfAvailCh = 79;

							if (nc < gNumOfAvailCh)
								gNumOfAvailCh = nc;
						}
#endif

#ifdef DIFFUSIVE
						num_diff = nd; // When all use diffusive.
#endif
#ifndef DIFFUSIVE
						num_diff = 0; // When all use adaptive or legacy.
#endif
						pPostStr = postfix_str;

						switch (gModeDefault)
						{
						case MODE_LEGACY:
							sprintf(pPostStr, "_LFH");
							break;
						case MODE_AFH:
							sprintf(pPostStr, "_AFH");
							break;
						case MODE_AFH_RL:
							sprintf(pPostStr, "_AFH-RL");
							break;
						case MODE_LEGACY_RL:
							sprintf(pPostStr, "_LFH-RL");
							break;
						case MODE_DFH_RL:
							sprintf(pPostStr, "_DFH-RL_HMAX-%d", HMAX);
							break;
						default:
							break;
						}

						pPostStr += strlen(postfix_str);
#ifdef DIFFUSIVE
						sprintf(pPostStr, "-coexist-%02d-dfh", num_diff);
						pPostStr = postfix_str + strlen(postfix_str);
#endif
#ifdef WIFI
						sprintf(pPostStr, "-wifi_%d_picos", na);
						pPostStr = postfix_str + strlen(postfix_str);
#endif
						if (nc < NUM_AGENTS)
							sprintf(pPostStr, "-n%d", nc);

						// creward=fopen("creward.txt", "w");
#ifdef HEATMAP
						sprintf(col_graph_str, "col_graph%s.txt", postfix_str);
						sprintf(heatmapfile_str, "heatmap%s.txt", postfix_str);
						sprintf(trajectory_str, "trajectory%s.txt", postfix_str);

						col_graph = fopen(col_graph_str, "w");
						chan = fopen("chan.txt", "w");
						heatmapfile = fopen(heatmapfile_str, "w");
						trajectory = fopen(trajectory_str, "w");
						// fopen_s(&heatmapfile, heatmapfile_str, "w");
						// fopen_s(&trajectory,trajectory_str,"w");
#endif
						// initialize_agents(agents, NUM_AGENTS);
						// run_simulation(agents, NUM_AGENTS);

						// The total_collisions array is initialized to all zeros in initialize_agents.
						initialize_agents(gstAgents, na);
						run_simulation(gstAgents, na);

						// fprintf(pcol, "pico1 = %f, pico10 = %f (nd = %d)\n", total_collisions[1] * 1.0 / (MAX_EPISODES - PERTURBATION), total_collisions[10] * 1.0 / (MAX_EPISODES - PERTURBATION), nd);
#ifdef DIFFUSIVE
						final_collision_tally = 0;
						for (int i = 1; i <= num_diff; i++)
						{
							final_collision_tally += total_collisions[i];
							// printf("%f ", total_collisions[i] * 1.0 / (MAX_EPISODES - PERTURBATION));
						}

						result_pcol[0] = 0;
						if (num_diff > 0)
							result_pcol[0] = final_collision_tally * 1.0 / num_diff / (MAX_EPISODES - PERTURBATION);
						// printf("\nno_diff%d M%d %f MODE_DFH", num_diff, gstAgents[1].hopping_mode, );

						final_collision_tally = 0;
						for (int i = num_diff + 1; i <= NUM_AGENTS; i++)
						{
							final_collision_tally += total_collisions[i];
						}

						result_pcol[1] = 0;
						if (num_diff < NUM_AGENTS)
							result_pcol[1] = final_collision_tally * 1.0 / (NUM_AGENTS - num_diff) / (MAX_EPISODES - PERTURBATION);
						// printf("\nno_diff%d M%d %f MODE_DFH", num_diff, gstAgents[10].hopping_mode, final_collision_tally * 1.0 / (NUM_AGENTS-num_diff) / (MAX_EPISODES - PERTURBATION));

#endif
						final_collision_tally = 0;
						temp_index = 0;
						for (int i = 1; i <= NUM_AGENTS; i++)
						{
							sprintf(&(col_per_agent[temp_index]), "%f ", (total_collisions[i] * 1.0 / (MAX_EPISODES - PERTURBATION)));
							temp_index = strlen(col_per_agent);
							final_collision_tally += total_collisions[i];
							// printf("%f ", total_collisions[i] * 1.0 / (MAX_EPISODES - PERTURBATION));
							final_wifi_collision_tally += (double)(total_wifi_collisions[i]);
						}

						final_wifi_collision_tally = final_wifi_collision_tally / ((WIFI_END - WIFI_START) * na);

						// fprintf(pcol, "%d %d %f\n", na, nc, final_collision_tally * 1.0 / na / (MAX_EPISODES - PERTURBATION));
						// if(nc == 79)
						//	fprintf(pcol, "\n");
#ifdef DIFFUSIVE
						result_pcol[2] = final_collision_tally * 1.0 / na / (MAX_EPISODES - PERTURBATION);
						printf("With %d NoDiff %02d ch %d hmax%d %f %f %f\n", gTargetCoexit, num_diff, gNum_channels, HMAX, result_pcol[0], result_pcol[1], result_pcol[2]);
#else

					printf("\nM%d %d %d %d %f %f hmax%d %s", gModeDefault, nc, na, gNumOfAvailCh, final_collision_tally * 1.0 / na / (MAX_EPISODES - PERTURBATION), final_wifi_collision_tally, HMAX, col_per_agent);
					fprintf(pcol, "\nM%d %d %d %d %f %f hmax%d %s", gModeDefault, nc, na, gNumOfAvailCh, final_collision_tally * 1.0 / na / (MAX_EPISODES - PERTURBATION), final_wifi_collision_tally, HMAX, col_per_agent);
#endif

						switch (gModeDefault)
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
							if (gNumOfAvailCh < 79)
							{
								gNumOfAvailCh++;
								gModeDefault--;
							}
							break;
#endif
						default:
							break;
						}
						/*
						if(gModeDefault == MODE_AFH && gNumOfAvailCh < 79)
						{
							gNumOfAvailCh = gNumOfAvailCh + 1;

	//						if(gNumOfAvailCh == 79)
	//							gNumOfAvailCh = 60;
	//						else
	//							gNumOfAvailCh -= 10;

							gModeDefault = MODE_AFH -1;
						}

						*/
						// fclose(creword);
#ifdef HEATMAP
						fclose(heatmapfile);
						// fflush(trajectory);
						fclose(trajectory);
						fclose(col_graph);
						fclose(chan);
#endif
					}
				}

			}
			fprintf(pcol, "\n", na);
			// fflush(pcol);
#ifdef DIFFUSIVE
		} // for (gTargetCoexit = MODE_AFH; gTargetCoexit != MODE_LEGACY_RL ; gTargetCoexit = MODE_LEGACY_RL){
#endif
	}
	fclose(pcol);
	//    fclose(pfQValueFile);

	return 0;
}
