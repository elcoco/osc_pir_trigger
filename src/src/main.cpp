#include <Arduino.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <EEPROM.h>

#include "esp32-hal-gpio.h"
#include "pins_arduino.h"   // Find custom pin definitions in here

#include "config.h"


/* ethernet.h     ->  https://docs.arduino.cc/libraries/ethernet/#Ethernet%20Class
 * example        ->  https://github.com/Johannf78/ESP32-W5500/blob/main/code/ESP32withW5500Try1.ino
 * board          ->  https://www.waveshare.com/wiki/ESP32-S3-ETH#Other_resource_link
 * schematics     ->  https://files.waveshare.com/wiki/ESP32-S3-ETH/ESP32-S3-ETH-Schematic.pdf
 * OSC example    ->  https://github.com/CNMAT/OSC/blob/master/examples/UDPSendMessage/UDPSendMessage.ino
 * OSC API        ->  https://github.com/CNMAT/OSC/blob/master/API.md
 * OSC debugging  ->  https://github.com/72nd/osc-utility
 */


#define STATE_TRIGGERED HIGH
#define EEPROM_SIZE 64

EthernetUDP Udp;

// timer for led blinking
hw_timer_t *timeout;

void isr_on_timer();
void wait_for_link();

void trigger_init(struct Trigger *trig);
int  trigger_check(struct Trigger *trig);
void trigger_debug(struct Trigger *trig);

void led_blink(struct Led *led, uint8_t n);
void osc_send_msg(struct Trigger *trig, int8_t msg);


void IRAM_ATTR isr_on_timer()
{
    if (trigger.led->blink_enabled) {
      digitalWrite(trigger.led->pin, !digitalRead(trigger.led->pin));
    }
}

void wait_for_link()
{
    uint8_t was_connected = 1;

    while (Ethernet.linkStatus() == LinkOFF) {

        // Use ISR to blink leds
        trigger.led->blink_enabled = 1;

        printf("Waiting for link\n");
        delay(500);
        was_connected = 0;
    }
    if (!was_connected)
        printf("Link is UP\n");

    // End in LED off state
    trigger.led->blink_enabled = 1;
    digitalWrite(trigger.led->pin, LOW);
}

void trigger_init(struct Trigger *trig)
{
    pinMode(trig->pin, INPUT_PULLDOWN);
    trig->prev_state = 0;

    pinMode(trig->led->pin, OUTPUT);
    trig->led->blink_enabled = 0;

    trig->t_ms = -1;
}

int trigger_check(struct Trigger *trig)
{
    uint8_t state = digitalRead(trig->pin);

    // only trigger when previous state was not pressed
    
    // Ignore repeated triggers
    if (state == STATE_TRIGGERED && trig->prev_state)
        asm("nop");
    // Button is unpressed after being pressed
    else if (state != STATE_TRIGGERED && trig->prev_state) {
        trig->prev_state = 0;
    }
    // A brand new button press
    else if (state == STATE_TRIGGERED) {

        // Triggers reset timeout so when a group of people walks in, the timer will be counting starting
        // from the last person walking in.

        if (trig->t_ms < 0 || (millis() - trig->t_ms) > T_TIMEOUT_MS) {
            trig->prev_state = 1;
            trig->t_ms = millis();
            return 1;
        }
        else {
            printf("Ignoring input, reset timeout: %ld\n", (trig->t_ms) ? (millis() - trig->t_ms) : -1);
            trig->t_ms = millis();
            delay(100);
        }
    }
    return 0;
}

void led_blink(struct Led *led, uint8_t n)
{
    for (uint8_t i=0 ; i<n ; i++) {
        digitalWrite(led->pin, 1);
        delay(10);
        digitalWrite(led->pin, 0);
        delay(BLINK_DUTYCYCLE_MS);
    }
}

void osc_send_msg(struct Trigger *trig)
{
    printf("triggered\n");
    Msg *msg = trig->msg;
    printf("Send: %s:%d%s\n", msg->ip.toString().c_str(), msg->port, msg->addr);
    OSCMessage osc_msg(msg->addr);
    //osc_msg.add(msg);
    Udp.beginPacket(msg->ip, msg->port);
    osc_msg.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    osc_msg.empty(); // free space occupied by message
}

void trigger_debug(struct Trigger *trig)
{
    /* Print message at boot */
    printf("Local IP: %s\n", local_ip.toString().c_str());
    printf("MSG %s:%d%s\n", trig->msg->ip.toString().c_str(), trig->msg->port, trig->msg->addr);
}

void setup() 
{

    SPI.begin(ETH_CLK, ETH_MISO, ETH_MOSI);
    delay(1000);

    Ethernet.init(ETH_CS); 
    Ethernet.begin(mac, local_ip);
    Udp.begin(local_port);

    if (!EEPROM.begin(EEPROM_SIZE)) {
        printf("failed to initialize EEPROM\n");
        delay(1000);
    }


    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        printf("ERROR: No Ethernet hardware detected!\n");
        while (1);
    }

    trigger_debug(&trigger);
    wait_for_link();
    trigger_init(&trigger);

    // Checks in ISR function if led blinking is enabled
    timeout = timerBegin(0, 80, true);
    timerAttachInterrupt(timeout, &isr_on_timer, true);
    timerAlarmWrite(timeout, BLINK_DUTYCYCLE_MS * 1000, true);
    timerAlarmEnable(timeout);

}

void loop() 
{
    wait_for_link();
    if (trigger_check(&trigger)) {
        osc_send_msg(&trigger);
        //led_blink(trigger.led, 1);
        //delay(T_TIMEOUT_MS);
        //printf("end timeout\n");
    }
    delay(.1);

}
