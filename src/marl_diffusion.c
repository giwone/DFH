#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "afh.h"
#include "marl.h"
#include "marl_diffusion.h"
#include "physical_model.h"
#define PACKET_TYPES 3

#define PICONETS 40 // Number of piconets
#define SLOT_TYPE 3 // 1: 1-slot, 2: 3-slot, 3: 5-slot, 4: variable
#define SLOT_TYPE_VARIABLE 4
#define CHANNELS 40	  // 79 // Number of channels
#define SLOT_TIME 625 // Slot time (µs)
// SBC : 14.5ms, 595bytes
// LDAC : 7.5ms, 928 bytes (using 930)
// VBR : 23.219ms, 639 bytes
// constant : 1.25ms, 83bytes
#define ARRIVAL_INTERVAL 1.25 // Data arrival interval (ms)
#define DATA_SIZE 83		  // Arriving data size (bytes)
#define ACK 1
#define NACK 0
#define STATE_COUNT 10 // Number of states (based on collision rate)
// X_FACTOR for delay. A value of 0.03 results in -1000 for a 200ms delay. Larger values increase the gap. A value of 0.01 results in -345.
#define X_FACTOR_DELAY 0.03
#define X_FACTOR_COLLISION 0
#define MAX_DATA_Q 100
#define DEF_CH_USED_TIME 10

#define MAX_CH_INFO 8

#define INC_CH_INDEX(index) ((index + 1) % MAX_CH_INFO)

extern int marl_main(void);

typedef int (*FP_CAL_STATE)(Queue *pQ);
// Initialize the queue for each piconet
Queue piconet_queues[MAX_PICONNETS + 1];

const int packet_sizes[MAX_PICONNETS] = {83, 552, 1021}; // 3-DH1, 3-DH3, 3-DH5
const int packet_durations[PACKET_TYPES] = {1, 3, 5};	 // Number of slots for 3-DH1, 3-DH3, 3-DH5
const int gPredefTime[MAX_PICONNETS] = {
	0,
};
// const int gPredefTime[MAX_PICONNETS] = {37, 208, 695, 443, 242, 1142, 1316, 1565, 1556, 184, };
//  Initialize Q-table for each piconet
double Q[MAX_PICONNETS][STATE_COUNT][PACKET_TYPES] = {
	{
		{
			0.0,
		},
	},
};
// Channel usage state (0: not in use, 1: in use)
uint64_t bdAddr[56] = {
	0x28022e9d20d4, 0xbc9307225d70, 0xf0d7932fc094, 0xbc9307182750,
	0xe0bda04b3b44, 0xbc9307430cbc, 0x6cacc2f35447, 0xbc93072f3d22,
	0x6cacc2e95d6d, 0xbc9307356bb0, 0xbc93072e3dca, 0xbc9307356eb8,
	0x20022e9262aa, 0xbc9307345996, 0xdc1a40c8fd81, 0xcfce92910705,
	0x4d2c4780b90b, 0x440290a03c8f, 0xc09e27de10f, 0xd77ef1c66e12,
	0xc38df82ce718, 0xc709518f9ea1, 0xb7a3147a1d22, 0xc7c3139cb723,
	0x4995f5649223, 0x87fff04c85a3, 0x1800ef32ba26, 0xb12129e681ab,
	0xf4fdff703cae, 0xfbe1fc960d2f, 0x9309ff90a333, 0x40711c56aeb4,
	0x27b3ed8e5335, 0xb7c962e94a34, 0x2fd5436fa937, 0x101de013793b,
	0xbda2da2a5d3d, 0x3fa7f5f0a5bf, 0x9a05c36458bf, 0x15527b931945,
	0xa0e6c15ab04b, 0xeee2129159cd, 0xd56b206f4cd0, 0xa45c8b1d7f50,
	0x4ce49878e0d1, 0x4f1612d86557, 0xbecc1aa68ade, 0xd74a3843de2,
	0x3d68594de6e5, 0x49b827634566, 0x6b9cb5d98a67, 0xd920873a6dea,
	0xaec5fcf520ee, 0x2b4c25523770, 0x4f90b5742f2, 0x893039c73474};

int gNumOfPiconets = PICONETS;
int gSlotType = SLOT_TYPE;
int gChannels = CHANNELS;
int gPredefStartTime = 0;
int gDefChUsedTime = 0;

bool gAvailable_channels[79];

int select_channel(uint8_t picoId, int current_time, int duration)
{
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[picoId].stHoppingInfo);
	S_SELECTED_CH_INFO *pChInfo;
	uint8_t nextFreq;
	pHopInfo->chInfoIndex = INC_CH_INDEX(pHopInfo->chInfoIndex);
	pChInfo = &(pHopInfo->stChInfo[pHopInfo->chInfoIndex]);

	nextFreq = calculate_next_frequency(pHopInfo->bdAddr, ((uint32_t)current_time + pHopInfo->base_clk) << 1, pHopInfo->available_channels, pHopInfo->noOfCh);

	pChInfo->chId = nextFreq;
	pChInfo->startClk = current_time;
	pChInfo->endClk = current_time + duration;
	pChInfo->duration = duration;

	return nextFreq;
}

int select_channel_wo_remapping(uint8_t picoId, int current_time, int duration)
{
	S_HOPPING_INFO *pHopInfo = &(piconet_queues[picoId].stHoppingInfo);
	S_SELECTED_CH_INFO *pChInfo;
	uint8_t nextFreq;
	pHopInfo->chInfoIndex = INC_CH_INDEX(pHopInfo->chInfoIndex);
	pChInfo = &(pHopInfo->stChInfo[pHopInfo->chInfoIndex]);

	nextFreq = calculate_next_frequency(pHopInfo->bdAddr + (1 << 27), ((uint32_t)current_time + pHopInfo->base_clk) << 1, gAvailable_channels, 79);

	pChInfo->chId = nextFreq;
	pChInfo->startClk = current_time;
	pChInfo->endClk = current_time + duration;
	pChInfo->duration = duration;

	return nextFreq;
}

void generateRandomStartClock(int n, int *array)
{

	if (gPredefStartTime)
	{
		for (int i = 0; i < n; i++)
		{
			array[i] = gPredefTime[i];
		}
		return;
	}

	for (int i = 0; i < n; i++)
	{
		array[i] = (rand() % 1600);
	}
}

void printQ(void)
{
	for (int piconet = 0; piconet < gNumOfPiconets; piconet++)
	{
		printf("Piconet %d Q-values:\n", piconet + 1);
		for (int state = 0; state < STATE_COUNT; state++)
		{
			for (int i = 0; i < PACKET_TYPES; i++)
			{
				printf("  Q-value for State %d, Packet Type 3-DH%d: %.2f\n", state, (i == 0 ? 1 : (i == 1 ? 3 : 5)), Q[piconet][state][i]);
			}
		}
	}
}

int main(int argc, char *argv[])
{
	// Initialize current time
	int current_time = 0;

	FILE *fp;
	Queue *pQ;
	int tempStartTime[gNumOfPiconets];
	// Initialize queue
	memset(piconet_queues, 0, sizeof(piconet_queues));

	if (argc == 1)
	{
		printf("default value used : ");
		gNumOfPiconets = 40;
		gSlotType = 1;
		gChannels = 79;
		gPredefStartTime = 0;
	}
	else
	{
		if (argc < 4)
		{
			printf("parameter error: piconets, slotType, channel needed\n");
			exit(1);
		}

		gNumOfPiconets = atoi(argv[1]);
		gSlotType = atoi(argv[2]);
		gChannels = atoi(argv[3]);

		if (argc >= 4)
		{
			gPredefStartTime = atoi(argv[4]);

			if (argc >= 5)
				gDefChUsedTime = atoi(argv[5]);
		}

		if (gNumOfPiconets > MAX_PICONNETS)
		{
			printf("error exceed the number of piconets\n");
			exit(1);
		}

		printf("Set value : ");
	}

	printf("%d piconets, %d slotType, %d channels\n", gNumOfPiconets, gSlotType, gChannels);
	// Initialize random seed
	srand(time(NULL));

	generateRandomStartClock(gNumOfPiconets, tempStartTime);

	for (int piconet = 1; piconet < gNumOfPiconets + 1; piconet++)
	{
		pQ = &(piconet_queues[piconet]);
		pQ->startClock = tempStartTime[piconet];
		pQ->next_arrival_time = (current_time + pQ->startClock) * SLOT_TIME / 1000.0;
		pQ->stHoppingInfo.bdAddr = bdAddr[piconet];
		memset(pQ->stHoppingInfo.available_channels, 1, 79);
		pQ->stHoppingInfo.noOfCh = 79;
		// pQ->stHoppingInfo.base_clk = ((rand() % (1600*100))&(0xffffffff-1)) | ((pQ->startClock % 2));
		pQ->stHoppingInfo.base_clk = ((rand() % (1600 * 100)) & (0xffffffff - 1));
	}

	memset(gAvailable_channels, 1, 79);

	marl_main();
}