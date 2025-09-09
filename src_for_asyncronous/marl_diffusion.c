#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "marl_diffusion.h"
#include "marl.h"

#define ASYNC
Queue piconet_queues[MAX_PICONNETS + 1];
const int packet_durations[] = {1, 3, 5};
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
    0xaec5fcf520ee, 0x2b4c25523770, 0x4f90b5742f2,  0x893039c73474
};

int gNumOfPiconets = 40;
int MAX_EPISODES = 100000;

int main(int argc, char* argv[]) {
    if (argc > 1) {
        gNumOfPiconets = atoi(argv[1]);
        if (gNumOfPiconets > MAX_PICONNETS) {
            printf("Error: Number of piconets cannot exceed %d\n", MAX_PICONNETS);
            return 1;
        }
    }
    if (argc > 2) MAX_EPISODES = atoi(argv[2]);
    printf("Starting us-based simulation with %d piconets for %d episodes.\n", gNumOfPiconets, MAX_EPISODES);

    srand(time(NULL));

    for (int piconet = 1; piconet <= gNumOfPiconets; piconet++) {
        Queue *pQ = &piconet_queues[piconet];
        memset(pQ, 0, sizeof(Queue));

        pQ->stHoppingInfo.bdAddr = bdAddr[piconet - 1];
        memset(pQ->stHoppingInfo.available_channels, 1, 79);
        pQ->stHoppingInfo.noOfCh = 79;
        //pQ->stHoppingInfo.base_clk = (rand() % (1600 * 100));

#ifdef ASYNC
    	pQ->stHoppingInfo.base_clk = ((rand() % (1600))&(0xffffffff));
    	pQ->stHoppingInfo.clk_offset = (rand() % 625);
#endif
#ifndef ASYNC
    	pQ->stHoppingInfo.base_clk = ((rand() % (1600))&(0xffffffff-1));
    	pQ->stHoppingInfo.clk_offset = 0;
#endif
        pQ->current_state = STATE_IDLE;
    }

    marl_main();
/*
    printf("\n--- Simulation Finished ---\n");
    double total_avg_collision_rate = 0;
    for (int i = 1; i <= gNumOfPiconets; i++) {
        Queue *pQ = &piconet_queues[i];
        long long total_attempts = pQ->successful_transmissions + pQ->collisions;
        double success_rate = (total_attempts > 0) ? (double)pQ->successful_transmissions / total_attempts * 100.0 : 0.0;
        double avg_delay_ms = (pQ->successful_transmissions > 0) ? ((pQ->total_delay_us / pQ->successful_transmissions) / 1000.0) : 0.0;

        total_avg_collision_rate += (100.0 - success_rate);

        printf("Piconet %2d: Success Rate: %6.2f%%, Avg Delay: %7.3f ms, Collisions: %lld\n", i, success_rate, avg_delay_ms, pQ->collisions);
    }
    printf("----------------------------------------------------------------\n");
    printf("Overall Average Collision Rate: %.2f%%\n", total_avg_collision_rate / gNumOfPiconets);
*/
    return 0;
}
