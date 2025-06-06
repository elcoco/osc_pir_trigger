#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

// Available GPIO's = 21, 17, 16, 18, 15, 3, 2, 1, 0, 44, 43, 20, 19, 48, 47, 46, 45, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33

// Time between blinks when triggered
#define BLINK_DUTYCYCLE_MS 500

#define LOCAL_IP 10, 0, 0, 100
#define LOCAL_PORT 8888
#define TARGET_IP 10, 0, 0, 255
#define TARGET_PORT 53000
#define MAC_ADDR 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED

#define PIN_TRIGGER 0
#define PIN_LED     1

#define TIMEOUT_DEFAULT 10 * 1000


struct Led {
    uint8_t pin;
    uint8_t blink_enabled;  // When 1, led blinks using timer
};

struct Msg {
    const char *addr;    // OSC address
    const IPAddress ip;  // remote ip
    const int port;      // remote port
};

struct Trigger {
    uint8_t pin;
    uint8_t prev_state;
    struct Led *led;
    struct Msg *msg;
    int32_t t_ms;        // Keep track of time inbetween triggers
    uint32_t t_timeout_ms;   // The time between triggers
};

struct Msg msg_go {
    .addr = "/go",
    .ip = IPAddress(TARGET_IP),
    .port = TARGET_PORT
};

// Static IP/port of this device
const IPAddress local_ip(LOCAL_IP);
const int local_port = LOCAL_PORT;
byte mac[6] = {MAC_ADDR};

struct Led led_trigger = { .pin = PIN_LED,
                           .blink_enabled = 0 };

struct Trigger trigger = { .pin  = PIN_TRIGGER,
                          .prev_state = 0,
                          .led  = &led_trigger,
                          .msg = &msg_go,
                          .t_ms = -1,
                          .t_timeout_ms = TIMEOUT_DEFAULT };

#endif // !CONFIG_H
