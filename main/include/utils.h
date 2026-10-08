#ifndef STORMBORN_UTILS_H
#define STORMBORN_UTILS_H
#include <stdint.h>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

typedef struct {
    int8_t forward;
    int8_t turn;
    uint8_t speed; // 0: brake 
    uint8_t flags;

    // 1st bit for brake
    uint8_t servo[2];
    // uint8_t servo[6];
    uint8_t checksum;
    // uint8_t __padding;
} __attribute__((__packed__)) ControlData;

typedef union {
    ControlData data;
    uint8_t buffer[sizeof(ControlData)];
} ControlPacket;

#define UDP_PORT 4210
#define WATCHDOG_TIMEOUT_MS 250

#define BLINK_GPIO GPIO_NUM_2

#endif // STORMBORN_UTILS_H
