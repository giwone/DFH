/*
 * marl_diffusion.h
 *
 * Created on: 2024. 9. 4.
 * Author: widen
 */

#ifndef MARL_DIFFUSION_H_
#define MARL_DIFFUSION_H_

#define MAX_DATA_Q 100
#define MAX_CH_INFO 8
#define MAX_PICONNETS 40

typedef struct
{
    int numOfTx;
    int numOfReTx;
} Qstatus;

typedef struct
{
    int size;
    int type;
    double arrival_time; // Time when packet was queued
    double sdu_generation_time;
    int channel;
    int remaining_time;
    bool collision; // Collision status
} DataPacket;

typedef struct
{
    uint32_t startClk;
    uint32_t endClk;
    uint8_t chId;
    uint8_t duration;
} S_SELECTED_CH_INFO;

typedef struct
{
    bool available_channels[79];

    uint64_t bdAddr;   // Master's BD_ADDR
    uint32_t base_clk; // Base clock for each piconet for frequency hopping
    uint8_t noOfCh;    // Default is 79
    uint8_t chInfoIndex;
    uint8_t bWifiStart;
    uint8_t bWifiStop;

    S_SELECTED_CH_INFO stChInfo[MAX_CH_INFO];
} S_HOPPING_INFO;

typedef struct
{
    DataPacket data_queue[MAX_DATA_Q]; // Data queue
    int data_queue_front;
    int data_queue_rear;
    int data_queue_size;
    // Packets are no longer buffered if they exceed the queue size.
    int numOfSduGenerated;
    // Packets are no longer buffered if they exceed the queue size.
    double numOfLossDueToQdelay;
    DataPacket tx_queue[30]; // Transmission queue
    int chUsedTime;
    int tx_queue_size;
    int tx_queue_index; // Index of the packet currently being transmitted
    bool transmitting;  // Flag indicating if a transmission is in progress
    double next_arrival_time;
    // Start time to differentiate data transmission start points.
    int startClock;
    S_HOPPING_INFO stHoppingInfo;

    double avgDelay;
    double avgDelayInSec;
    double avgTxDelay;
    double avgTxDelayInSec;
    int numOfSdus;           // Number of successfully transmitted SDUs
    int numOfSdusPerSec;     // Number of successfully transmitted SDUs per second
    int collisions;          // Total number of PDU retransmissions
    int collisionsPerSec;    // Number of PDU retransmissions per second
    int total_transmissions; // Total number of PDU transmissions
    int numOfTxPerSec;       // Number of PDU transmissions per second (new TX per sec + collisions per sec)
    double maxDelayPerSec;
    double maxTxDelayPerSec;
    int state; // Current state (based on collision rate)

} Queue;

extern Queue piconet_queues[MAX_PICONNETS + 1];

#endif /* MARL_DIFFUSION_H_ */