#ifndef MARL_DIFFUSION_H_
#define MARL_DIFFUSION_H_

#include <stdbool.h>
#include <stdint.h>
#include "data_model.h"
#define MAX_DATA_Q 100
#define MAX_CH_INFO 8
#define MAX_PICONNETS 40
#define SLOT_TIME_US 625.0


#define INC_CH_INDEX(index)	((index+1) % MAX_CH_INFO)

typedef enum {
    STATE_IDLE,
    STATE_TRANSMITTING,
    STATE_WAITING_FOR_ACK
} PiconetState;

typedef struct {
    int size;
    int slotType; // slot type : 1, 3, 5 based on size
    double sdu_generation_time_us;
    int channel;
    int retransmission_count;
    bool data_collided;

    double tx_start_us;
    double tx_end_us;
    double ack_start_us;
    double ack_end_us;
} DataPacket;

typedef struct {
	uint32_t startClk;
	uint32_t endClk;
	uint8_t chId;
	uint8_t duration;
}S_SELECTED_CH_INFO;

typedef struct {
    bool available_channels[79];
    uint64_t bdAddr;
    uint32_t base_clk;
    uint32_t clk_offset;
    uint8_t noOfCh;
	uint8_t chInfoIndex;
    uint8_t bWifiStart;
    uint8_t bWifiStop;

    S_SELECTED_CH_INFO stChInfo[MAX_CH_INFO];
} S_HOPPING_INFO;

typedef struct {
    DataPacket data_queue[MAX_DATA_Q];
    int data_queue_front;
    int data_queue_rear;
    int data_queue_size;

    DataPacket current_tx_packet;
    bool is_transmitting;

    PiconetState current_state;
    double start_time_us; // data delivery start point

    S_HOPPING_INFO stHoppingInfo;

    long long numOfSduGenerated;
    long long numOfLossDueToQdelay;
    long long collisions;
    double total_delay_us;
    long long successful_transmissions;

    GreedyDL stGreedyDlData;
} Queue;

extern Queue piconet_queues[MAX_PICONNETS + 1];
extern uint64_t bdAddr[56];
#endif /* MARL_DIFFUSION_H_ */
