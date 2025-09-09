/* ================== Greedy Download Mode (no ENC) ==================
 * - Master->Slave Segment: 64B(15%) + TruncNormal(μ=1000,σ=120,[600,1021]) (85%)
 * - Slave->Master Response: 1-slot after every Master TX block (ACK 64B 2:1, or NULL 0B)
 * - No Offered Load/Δt: Next segment is always ready immediately (GREEDY)
 * - EDR payload rate: 2 or 3 Mb/s (for airtime calculation)
 * - Slot length: 625 µs (master even → slave odd → master even …)
 * ================================================================== */
#include <stdlib.h>
#include <math.h>
#include <time.h>
//#include "rng.h"
#include "data_model.h"
/* ---- Basic Parameters ---- */
#define SLOT_US            625.0
#define DL_SMALL_P         0.15
#define DL_SMALL_BYTES     64
#define DL_BULK_MU         1000.0
#define DL_BULK_SIGMA      120.0
#define DL_BULK_MIN        600
#define DL_BULK_MAX        1021

#define ACK_BYTES          64
#define NULL_BYTES         0
#define DELAYED_ACK_RATIO  2

/* Airtime fragments (µs, EDR common) */
#define ACCESS_US          72.0
#define HEADER_US          54.0
#define GUARD_US           5.0
#define SYNC_US            11.0
#define TRAILER_US         2.0

/* ---- RNG ---- */
static inline double urand01(void){ return (double)rand() / (double)RAND_MAX; }
static inline double randn_std(void){
    double u1 = urand01(); if (u1 < 1e-12) u1 = 1e-12;
    double u2 = urand01();
    return sqrt(-2.0*log(u1)) * cos(2.0*M_PI*u2);
}

/* Truncated Normal -> Integer Bytes */
static int truncnorm_bytes(double mu, double sigma, int a, int b){
    if (a>b){ int t=a; a=b; b=t; }
    if (sigma<=0.0){ int v=(int)llround(mu); if(v<a)v=a; if(v>b)v=b; return v; }
    for(int it=0; it<10000; ++it){
        double x = mu + sigma*randn_std();
        if (x>=a && x<=b){
            double fx=floor(x), frac=x-fx;
            int xi = (urand01()<frac)?(int)(fx+1):(int)fx;
            if (xi<a) xi=a; if (xi>b) xi=b; return xi;
        }
    }
    int v=(int)llround(mu); if(v<a)v=a; if(v>b)v=b; return v;
}

/* ---- Slot Mapping (based on EDR3 user payload) ---- */
int slots_by_user_bytes(int size){
    if (size <= 83) return 1;      /* 3-DH1 */
    else if (size <= 552) return 3;/* 3-DH3 */
    else return 5;              /* 3-DH5 (<=1021) */
}

/* ---- Airtime (µs): EDR payload 2 or 3 Mb/s ---- */
/* Payload = user + 2B(EDR hdr) + 2B(CRC16) */
double airtime_us_edr(int user_bytes, int edr_mbps){
    double payload_bits = 8.0 * (user_bytes + 2 + 2);
    double payload_us   = payload_bits / (double)edr_mbps; /* 2 or 3 bits/us */
    return ACCESS_US + HEADER_US + GUARD_US + SYNC_US + payload_us + TRAILER_US;
}


/* Initialization */
void greedy_init(GreedyDL* g, int edr_mbps, unsigned seed){
    //if (!seed) seed=(unsigned)time(NULL);
    //srand(seed);
    g->edr_payload_mbps = (edr_mbps==2)?2:3;
    g->dl_since_last_ack = 0;
    g->dl_segments = 0; g->ul_responses = 0;
}

/* Master segment sample */
int greedy_sample_master_bytes(void){

	if(urand01() < DL_SMALL_P)
	{
		return DL_SMALL_BYTES;
	}

	return truncnorm_bytes(DL_BULK_MU, DL_BULK_SIGMA, DL_BULK_MIN, DL_BULK_MAX);
}

void greedy_slave_response(GreedyDL* g, ULResp* r){
    g->ul_responses++;
    if (g->dl_since_last_ack >= DELAYED_ACK_RATIO){
        r->user_bytes = ACK_BYTES; r->duration_slots = 1; r->is_ack = 1;
        g->dl_since_last_ack = 0;
    } else {
        r->user_bytes = NULL_BYTES; r->duration_slots = 1; r->is_ack = 0;
    }
}

void greedy_next_cycle(
		GreedyDL* g, double now_even_us, MSeg* mseg, ULResp* uresp,
		double* t_master_start_us, double* t_slave_start_us, double* now_even_us_next){
    /* 1) Master TX block */
    mseg->bytes = greedy_sample_master_bytes();
    mseg->slots = slots_by_user_bytes(mseg->bytes);
    mseg->airtime_us = airtime_us_edr(mseg->bytes, g->edr_payload_mbps);

    *t_master_start_us = now_even_us; /* Master even slot start */
    double t_master_end_us = now_even_us + mseg->slots * SLOT_US;

    g->dl_segments++;
    g->dl_since_last_ack++;

    /* 2) Slave 1-slot response (immediately on the next odd slot) */
    greedy_slave_response(g, uresp);
    *t_slave_start_us = t_master_end_us; /* Odd slot start */
    double t_slave_end_us = t_master_end_us + SLOT_US;

    (void)t_slave_end_us; /* Use if necessary */

    /* 3) Next cycle: Start immediately on the next even slot */
    *now_even_us_next = t_master_end_us + SLOT_US; /* (m slots) + 1 slot */
}

int greedy_generate_master_data(GreedyDL *g) {

	g->stMseg.bytes = greedy_sample_master_bytes();
	g->stMseg.slots = slots_by_user_bytes(g->stMseg.bytes);
	g->stMseg.airtime_us = airtime_us_edr(g->stMseg.bytes, g->edr_payload_mbps);

    g->dl_since_last_ack++;
    g->dl_segments++;
    g->dl_seg_total_bytes += g->stMseg.bytes;

    return g->stMseg.bytes;
}

int greedy_generate_slave_response(GreedyDL* g){
    if (g->dl_since_last_ack >= DELAYED_ACK_RATIO){
        g->stUlResp.user_bytes = ACK_BYTES; g->stUlResp.duration_slots = 1; g->stUlResp.is_ack = 1;
        g->dl_since_last_ack = 0;
        g->ul_responses++;
        g->ul_res_total_bytes += ACK_BYTES;
    } else {
        g->stUlResp.user_bytes = NULL_BYTES; g->stUlResp.duration_slots = 1; g->stUlResp.is_ack = 0;
    }
    return g->stUlResp.user_bytes;
}



/* ================= Example Usage (Main Loop Integration) =================
GreedyDL G; greedy_init(&G, 3, 0);  // EDR 3 Mb/s, seed auto
double now_even = 0.0;              // Current even slot start (aligned to simulator time)

for (int k=0; k<NUM_CYCLES; ++k){
    MSeg ms; ULResp ur;
    double tM, tS, next_even;
    greedy_next_cycle(&G, now_even, &ms, &ur, &tM, &tS, &next_even);

    // Here, register with the actual scheduler/radio layer:
    // schedule_master_tx(tM, ms.bytes, ms.slots, ms.airtime_us);
    // schedule_slave_tx(tS, ur.user_bytes, 1, airtime_us_edr(ur.user_bytes, G.edr_payload_mbps));

    now_even = next_even; // Next even slot for immediate next transmission
}
================================================================== */
