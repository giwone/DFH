/*
 * afh.h
 *
 *  Created on: 2024. 8. 29.
 *      Author: widen
 */

#ifndef AFH_H_
#define AFH_H_

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

extern uint8_t calculate_next_frequency(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels);
extern uint8_t get_permuteout(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels);
#endif /* AFH_H_ */
