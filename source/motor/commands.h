/* List of all recognised motor commands.
These are all 8 bits like '00001111'. These correspond to:

Bit     Command
[1]     Forward 
[2]     Reverse
[3]     Left
[4]     Right
[5]     Forward Magnitude
[6]     Reverse Magnitude
[7]     Left Magnitude
[8]     Right Magnitude 

With the special '00000000' command meaning stop.
So '10000000' means forward at slow speed, and '10001000' means forward at max speed.
There are also illegal commands like '110000000' which means forward and reverse which is not possible
Some combinations like 10101010 are valid (Fast Forward and Fast Left) But we gonna remove them for simplicity*/

// ----- Single Direction Commands -----
#define CMD_FORWARD_SLOW        0x80    // 10000000
#define CMD_FORWARD_FAST        0x88    // 10001000
#define CMD_REVERSE_SLOW        0x40    // 01000000
#define CMD_REVERSE_FAST        0x44    // 01000100
#define CMD_LEFT_SLOW           0x20    // 00100000
#define CMD_LEFT_FAST           0x22    // 00100010
#define CMD_RIGHT_SLOW          0x10    // 00010000
#define CMD_RIGHT_FAST          0x11    // 00010001

// ----- Diagonal Commands -----
#define CMD_DIAG_TOP_LEFT       0xA0    // 10100000
#define CMD_DIAG_TOP_RIGHT      0x90    // 10010000
#define CMD_DIAG_BOTTOM_LEFT    0x60    // 01100000
#define CMD_DIAG_BOTTOM_RIGHT   0x50    // 01010000