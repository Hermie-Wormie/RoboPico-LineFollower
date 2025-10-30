#include "queue.h"
#include "semphr.h"

#define THRESHOLD 30000
#define MIN_THRESHOLD 100

/*
#define NARROW_WIDTH 30000
#define WIDE_WIDTH (NARROW_WIDTH*3)
#define VARIANCE 30
#define NARROW_RANGE_LOWER (NARROW_WIDTH - VARIANCE)
#define NARROW_RANGE_UPPER (NARROW_WIDTH + VARIANCE)
#define WIDE_RANGE_LOWER (WIDE_WIDTH - VARIANCE)
#define WIDE_RANGE_UPPER (WIDE_WIDTH + VARIANCE)
*/

// From main.c
extern QueueHandle_t barcodes_queue;

// From wifi.c
extern ip_addr_t remote_ip;
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

// barcode.c
#define WIDTH_IGNORE 500000 // 50ms. Any pulses above this will be treated as environmental and ignored (like when driving for long time and get triggered a random dark object)
#define MIN_DEBOUNCE 100    // Minimum value to count as a valid interrupt. Might have to be tuned for indiviudal sensors
#define ARRAY_SIZE 29       // Array size to hold barcode. 9(*) + <EmptyWhiteSmallBar> + 9(Char) + <EmptyWhiteSmallBar> + 9(*) = 29
#define MULTIPLIER 2.9      // Multipler to scale the identified number up. Any pulses above will count as Wide.
#define CALCULATION_IGNORE_THRESHOLD 1000 // occationally will get values that pass the debouce check, but are clearly wrong. They represent valid small bars, so we still keep them but ignore them for calculations

// All them barcodes right here, handtyped and checked by me. Screw GPT. Screw Claude. All that wasted time.
const uint16_t barcodes[44] = {
    0x0034, // 000110100 for '0'
    0x0121, // 100100001 for '1'
    0x0061, // 001100001 for '2' 
    0x0160, // 101100000 for '3'
    0x0031, // 000110001 for '4'
    0x0130, // 100110000 for '5'
    0x0070, // 001110000 for '6'
    0x0025, // 000100101 for '7'
    0x0124, // 100100100 for '8'
    0x0064, // 001100100 for '9'

    0x0109, // 100001001 for 'A'
    0x0049, // 001001001 for 'B'
    0x0148, // 101001000 for 'C'
    0x0019, // 000011001 for 'D'
    0x0118, // 100011000 for 'E'
    0x0058, // 001011000 for 'F'
    0x000D, // 000001101 for 'G'
    0x010C, // 100001100 for 'H'
    0x004C, // 001001100 for 'I'
    0x001E, // 000011110 for 'J'

    0x0103, // 100000011 for 'K'
    0x0043, // 001000011 for 'L'
    0x0142, // 101000010 for 'M'
    0x0013, // 000010011 for 'N'
    0x0112, // 100010010 for 'O'
    0x0052, // 001010010 for 'P'
    0x0007, // 000000111 for 'Q'
    0x0106, // 100000110 for 'R'
    0x0046, // 001000110 for 'S'
    0x0016, // 000010110 for 'T'

    0x0181, // 110000001 for 'U'
    0x00C1, // 011000001 for 'V'
    0x01C0, // 111000000 for 'W'
    0x0091, // 010010001 for 'X'
    0x0190, // 110010000 for 'Y'
    0x00D0, // 011010000 for 'Z' << Index 35

    0x0085, // 010000101 for '-'
    0x0184, // 110000100 for '.'
    0x00C4, // 011000100 for ' '
    0x00A8, // 010101000 for '$'
    0x00A2, // 010100010 for '/'
    0x008A, // 010001010 for '+'
    0x002A, // 000101010 for '%'
    0x0094  // 010010100 for '*'
};

const char barcode_chars[44] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',    // Indices 0-9
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',    // Indices 10-19
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',    // Indices 20-29
    'U', 'V', 'W', 'X', 'Y', 'Z',                        // Indices 30-35
    '-', '.', ' ', '$', '/', '+', '%', '*'               // Indices 36-42
};