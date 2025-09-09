/*
 * data_model.h
 *
 *  Created on: 2025. 9. 7.
 *      Author: giwon
 */

#ifndef SRC_DATA_MODEL_H_
#define SRC_DATA_MODEL_H_

/* 슬레이브 응답(ACK/NULL), 1-slot 고정 */
typedef struct
{
	int user_bytes;
	int duration_slots;
	int is_ack;
} ULResp;

/* ---- Greedy 사이클 스케줄러 ----
 * 입력: now_even_us = 현재 "마스터가 송신 가능한 짝수 슬롯 시작 시각"
 * 출력:
 *  - M 세그먼트 크기/슬롯/에어타임
 *  - S 응답(ACK/NULL) 및 에어타임
 *  - 다음 사이클의 now_even_us_next (= 이번 M TX + S 응답 뒤 다음 짝수 슬롯 시작)
 */
typedef struct
{
	int bytes;
	int slots;
	double airtime_us;
} MSeg;

/* ---- 상태 ---- */
typedef struct {
    int		edr_payload_mbps;   /* 2 or 3 */
    MSeg	stMseg;
    ULResp	stUlResp;
    unsigned dl_since_last_ack;
    unsigned long long dl_segments;
    unsigned long long dl_seg_total_bytes;// 성공적으로 전달된 총 segment bytes
    unsigned long long num_of_test_slots;// 성공적으로 전달된 총 segment bytes
    unsigned long long ul_responses;
    unsigned long long ul_res_total_bytes;// 성공적으로 전달된 총 segment bytes
} GreedyDL;

extern void greedy_init(GreedyDL* g, int edr_mbps, unsigned seed);

//void greedy_next_cycle(GreedyDL* g, double now_even_us, MSeg* mseg, ULResp* uresp, double* t_master_start_us, double* t_slave_start_us, double* now_even_us_next);

//int greedy_sample_master_bytes(void);

extern int slots_by_user_bytes(int size);

extern double airtime_us_edr(int user_bytes, int edr_mbps);

//void greedy_slave_response(GreedyDL* g, ULResp* r);

extern int greedy_generate_master_data(GreedyDL *g);
extern int greedy_generate_slave_response(GreedyDL* g);
#endif /* SRC_DATA_MODEL_H_ */
