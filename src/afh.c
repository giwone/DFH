#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define HOPPING_SEQUENCE_LENGTH 79 // Number of frequencies in the sequence (79 for basic Bluetooth)
#define BT_CLK_MASK 0x1FFFFFFF     // Mask for the 27-bit clock
#define BD_ADDR_MASK 0xFFFFFFFF    // Mask for the 28 bits of BD_ADDR (UAP/LAP)

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

// Function prototypes
uint8_t calculate_next_frequency(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels);

uint8_t remap_channel(uint8_t fk, bool *channel_map, uint8_t num_used_channels, uint8_t perm5_out, uint8_t E, uint8_t F_prime, uint8_t Y2);
uint32_t extract_bdaddr(uint64_t bdaddr);
uint8_t permute(uint8_t Z, uint16_t P);
void update_channel_map(bool *channel_map);

// Extract the 28-bit address (UAP/LAP) from the Bluetooth Device Address (BD_ADDR)
uint32_t extract_bdaddr(uint64_t bdaddr)
{
    return (uint32_t)(bdaddr & BD_ADDR_MASK);
}

// Perform the permutation using the control word P
uint8_t permute(uint8_t Z, uint16_t P)
{
    uint8_t Z_out[5];
    Z_out[0] = (Z & 0x01) >> 0; // Z0
    Z_out[1] = (Z & 0x02) >> 1; // Z1
    Z_out[2] = (Z & 0x04) >> 2; // Z2
    Z_out[3] = (Z & 0x08) >> 3; // Z3
    Z_out[4] = (Z & 0x10) >> 4; // Z4

    // Apply butterfly operations according to the control signals in P
    if (P & (1 << 12))
    {
        uint8_t temp = Z_out[0];
        Z_out[0] = Z_out[3];
        Z_out[3] = temp;
    } // P12: {Z0, Z3}
    if (P & (1 << 13))
    {
        uint8_t temp = Z_out[1];
        Z_out[1] = Z_out[2];
        Z_out[2] = temp;
    } // P13: {Z1, Z2}

    if (P & (1 << 10))
    {
        uint8_t temp = Z_out[2];
        Z_out[2] = Z_out[4];
        Z_out[4] = temp;
    } // P10: {Z2, Z4}
    if (P & (1 << 11))
    {
        uint8_t temp = Z_out[1];
        Z_out[1] = Z_out[3];
        Z_out[3] = temp;
    } // P11: {Z1, Z3}

    if (P & (1 << 8))
    {
        uint8_t temp = Z_out[1];
        Z_out[1] = Z_out[4];
        Z_out[4] = temp;
    } // P8: {Z1, Z4}
    if (P & (1 << 9))
    {
        uint8_t temp = Z_out[0];
        Z_out[0] = Z_out[3];
        Z_out[3] = temp;
    } // P9: {Z0, Z3}

    if (P & (1 << 6))
    {
        uint8_t temp = Z_out[0];
        Z_out[0] = Z_out[2];
        Z_out[2] = temp;
    } // P6: {Z0, Z2}
    if (P & (1 << 7))
    {
        uint8_t temp = Z_out[3];
        Z_out[3] = Z_out[4];
        Z_out[4] = temp;
    } // P7: {Z3, Z4}

    if (P & (1 << 4))
    {
        uint8_t temp = Z_out[0];
        Z_out[0] = Z_out[4];
        Z_out[4] = temp;
    } // P4: {Z0, Z4}
    if (P & (1 << 5))
    {
        uint8_t temp = Z_out[1];
        Z_out[1] = Z_out[3];
        Z_out[3] = temp;
    } // P5: {Z1, Z3}

    if (P & (1 << 2))
    {
        uint8_t temp = Z_out[1];
        Z_out[1] = Z_out[2];
        Z_out[2] = temp;
    } // P2: {Z1, Z2}
    if (P & (1 << 3))
    {
        uint8_t temp = Z_out[3];
        Z_out[3] = Z_out[4];
        Z_out[4] = temp;
    } // P3: {Z3, Z4}

    if (P & (1 << 0))
    {
        uint8_t temp = Z_out[0];
        Z_out[0] = Z_out[1];
        Z_out[1] = temp;
    } // P0: {Z0, Z1}
    if (P & (1 << 1))
    {
        uint8_t temp = Z_out[2];
        Z_out[2] = Z_out[3];
        Z_out[3] = temp;
    } // P1: {Z2, Z3}

    return (Z_out[0] << 0) | (Z_out[1] << 1) | (Z_out[2] << 2) | (Z_out[3] << 3) | (Z_out[4] << 4);
}
#define GET_SET_BIT(bdaddr, get_bit, set_bit) ((((bdaddr) >> (get_bit)) & 1) << (set_bit))
uint8_t calculate_next_frequency(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels)
{
    uint32_t bdaddr = extract_bdaddr(master_bdaddr); // Extract the UAP/LAP part of the BD_ADDR
    uint16_t X = (current_clk >> 2) & 0x1F;          // X = CLK[6:2]
    uint16_t Y1 = (current_clk >> 1) & 1;            // Y1 = CLK1
    uint16_t Y2 = (32 * Y1);                         // Y2 = 32 * Y1

    // A, B, C, D, E as defined in the table
    uint16_t A = ((bdaddr >> 23) & 0x1F) ^ ((current_clk >> 21) & 0x1F); // A = A27_23 ⊕ CLK25_21
    uint16_t B = (bdaddr >> 19) & 0x0F;                                  // B = A22_19
    // C = A8_6_4_2_0 ⊕ CLK20_16
    uint16_t C = (GET_SET_BIT(bdaddr, 8, 4) | GET_SET_BIT(bdaddr, 6, 3) | GET_SET_BIT(bdaddr, 4, 2) | GET_SET_BIT(bdaddr, 2, 1) | GET_SET_BIT(bdaddr, 0, 0)) ^ ((current_clk >> 16) & 0x1F);
    uint16_t D = ((bdaddr >> 10) & 0x1FF) ^ ((current_clk >> 7) & 0x1FF); // D = A18_10 ⊕ CLK15_7
    // E = A13_11_9_7_5_3_1
    uint16_t E = GET_SET_BIT(bdaddr, 13, 6) | GET_SET_BIT(bdaddr, 11, 5) | GET_SET_BIT(bdaddr, 9, 4) | GET_SET_BIT(bdaddr, 7, 3) | GET_SET_BIT(bdaddr, 5, 2) | GET_SET_BIT(bdaddr, 3, 1) | GET_SET_BIT(bdaddr, 1, 0);
    uint16_t F = (16 * ((current_clk >> 7) & 0x1FFFFF)) % 79;
    uint16_t F_prime = (16 * ((current_clk >> 7) & 0x1FFFFF)) % num_used_channels; // F' = 16 * CLK12_7 % N

    // Step 1: Add (X + A) mod 32
    uint16_t Z_prime = (X + A) % 32;

    // Step 2: XOR with A22-19 (simulated)
    Z_prime ^= B;

    // Step 3: Create control word P with XOR between Y1 and C
    uint16_t P = (((C ^ Y1) << 9) & 0x3E00) | (D & 0x1FF); // Combine (C ^ Y1) and D to form the control word for permutation

    // Step 4: Permutation operation using control word P
    uint16_t permuted_output = permute(Z_prime, P);

    // Step 5: Add E, F_prime, and Y2 (mod 79)
    uint16_t frequency = (permuted_output + E + F + Y2) % HOPPING_SEQUENCE_LENGTH;

    if (frequency < 40)
        frequency = frequency * 2;
    else
        frequency = (frequency - 40) * 2 + 1;

    // Step 6: Check if the selected frequency is available, if not, remap it
    if (!channel_map[frequency])
    {
        // Remap to an available frequency according to the standard formula
        frequency = remap_channel(frequency, channel_map, num_used_channels, permuted_output, E, F_prime, Y2);
    }

    return frequency;
}
// Calculate the next frequency based on the BD_ADDR, clock, and channel map

uint8_t get_permuteout(uint64_t master_bdaddr, uint32_t current_clk, bool *channel_map, uint8_t num_used_channels)
{
    uint32_t bdaddr = extract_bdaddr(master_bdaddr); // Extract the UAP/LAP part of the BD_ADDR
    uint16_t X = (current_clk >> 2) & 0x1F;          // X = CLK[6:2]
    uint16_t Y1 = (current_clk >> 1) & 1;            // Y1 = CLK1
    uint16_t Y2 = (32 * Y1);                         // Y2 = 32 * Y1

    // A, B, C, D, E as defined in the table
    uint16_t A = ((bdaddr >> 23) & 0x1F) ^ ((current_clk >> 21) & 0x1F); // A = A27_23 ⊕ CLK25_21
    uint16_t B = (bdaddr >> 19) & 0x0F;                                  // B = A22_19
    // C = A8_6_4_2_0 ⊕ CLK20_16
    uint16_t C = (GET_SET_BIT(bdaddr, 8, 4) | GET_SET_BIT(bdaddr, 6, 3) | GET_SET_BIT(bdaddr, 4, 2) | GET_SET_BIT(bdaddr, 2, 1) | GET_SET_BIT(bdaddr, 0, 0)) ^ ((current_clk >> 16) & 0x1F);
    uint16_t D = ((bdaddr >> 10) & 0x1FF) ^ ((current_clk >> 7) & 0x1FF); // D = A18_10 ⊕ CLK15_7
    // E = A13_11_9_7_5_3_1
    uint16_t E = GET_SET_BIT(bdaddr, 13, 6) | GET_SET_BIT(bdaddr, 11, 5) | GET_SET_BIT(bdaddr, 9, 4) | GET_SET_BIT(bdaddr, 7, 3) | GET_SET_BIT(bdaddr, 5, 2) | GET_SET_BIT(bdaddr, 3, 1) | GET_SET_BIT(bdaddr, 1, 0);
    uint16_t F = (16 * ((current_clk >> 7) & 0x1FFFFF)) % 79;
    uint16_t F_prime = (16 * ((current_clk >> 7) & 0x1FFFFF)) % num_used_channels; // F' = 16 * CLK12_7 % N

    // Step 1: Add (X + A) mod 32
    uint16_t Z_prime = (X + A) % 32;

    // Step 2: XOR with A22-19 (simulated)
    Z_prime ^= B;

    // Step 3: Create control word P with XOR between Y1 and C
    uint16_t P = (((C ^ Y1) << 9) & 0x3E00) | (D & 0x1FF); // Combine (C ^ Y1) and D to form the control word for permutation

    // Step 4: Permutation operation using control word P
    uint16_t permuted_output = permute(Z_prime, P);

    // Step 5: Add E, F_prime, and Y2 (mod 79)
    uint16_t frequency = (permuted_output + E + F + Y2) % HOPPING_SEQUENCE_LENGTH;

    if (frequency < 40)
        frequency = frequency * 2;
    else
        frequency = (frequency - 40) * 2 + 1;

    return frequency;
}

// Remap a blocked frequency to an available one using the standard formula k' = (PERM5out + E + F' + Y2) mod N
uint8_t remap_channel(uint8_t fk, bool *channel_map, uint8_t num_used_channels, uint8_t perm5_out, uint8_t E, uint8_t F_prime, uint8_t Y2)
{
    uint8_t k_prime = (perm5_out + E + F_prime + Y2) % num_used_channels;

    // Generate the mapping table
    uint8_t available_channels[HOPPING_SEQUENCE_LENGTH];
    uint8_t index = 0;
    for (int i = 0; i < HOPPING_SEQUENCE_LENGTH; i = i + 2)
    {
        if (channel_map[i])
        {
            available_channels[index++] = i;
        }
    }

    for (int i = 1; i < HOPPING_SEQUENCE_LENGTH; i = i + 2)
    {
        if (channel_map[i])
        {
            available_channels[index++] = i;
        }
    }

    // Select the k_prime-th available channel
    return available_channels[k_prime];
}

// Simulate channel map updates (e.g., due to interference detection)
void update_channel_map(bool *channel_map)
{
    static bool toggle = false;
    channel_map[40] = toggle;  // Toggle channel 40's availability
    channel_map[50] = !toggle; // Toggle channel 50's availability
    toggle = !toggle;

    printf("Updated channel map: Channel 40 %s, Channel 50 %s\n",
           channel_map[40] ? "available" : "blocked",
           channel_map[50] ? "available" : "blocked");
}
