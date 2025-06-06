#include <Arduino.h>

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
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
        printf("Waiting for link\n");
        delay(500);
        was_connected = 0;
    }
    if (!was_connected)
        printf("Link is UP\n");
}

void trigger_init(struct Trigger *trig)
{
    pinMode(trig->pin, INPUT_PULLDOWN);
    trig->prev_state = 0;

    pinMode(trig->led->pin, OUTPUT);
    trig->led->blink_enabled = 0;
}

int trigger_check(struct Trigger *trig)
{
    uint8_t state = digitalRead(trig->pin);
    
    // Turn off led if timer is done
    if (trig->t_ms && (millis() - trig->t_ms) > trig->t_timeout_ms) {
        trig->led->blink_enabled = 0;
        digitalWrite(trigger.led->pin, LOW);
    }
    
    // Ignore repeated triggers
    if (state == STATE_TRIGGERED && trig->prev_state)
        asm("nop");
    // Button is unpressed after being pressed
    else if (state != STATE_TRIGGERED && trig->prev_state) {
        trig->prev_state = 0;
    }
    // Oh boy, a brand new button press
    else if (state == STATE_TRIGGERED) {


        if (trig->t_ms < 0 || (millis() - trig->t_ms) > trig->t_timeout_ms) {
            trig->prev_state = 1;
            trig->t_ms = millis();
            trig->led->blink_enabled = 1;
            printf("Setting timer for %d ms\n", trig->t_timeout_ms);
            return 1;
        }
        else {
            printf("Ignoring input, reset timeout: %ld\n", (trig->t_ms) ? (millis() - trig->t_ms) : -1);

            // Timeout is reset everytime there is a trigger within the timeout.
            // This ensures that when a group of people walks in, the timer will start counting from the last person walking in.
            // To disable this bevavior, uncomment next line
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
        delay(50);
        digitalWrite(led->pin, 0);
        delay(50);
    }
}

void osc_send_msg(struct Trigger *trig)
{
    printf("triggered\n");
    Msg *msg = trig->msg;
    printf("Send: %s:%d%s\n", msg->ip.toString().c_str(), msg->port, msg->addr);
    OSCMessage osc_msg(msg->addr);
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

uint32_t eeprom_read(uint32_t addr, uint8_t *buf, size_t size)
{
    uint8_t *bufp = buf;
    for (int i=0; i<size; i++, bufp++)
        *bufp = byte(EEPROM.read(addr+i));
    return 1;
}

uint32_t eeprom_write(uint32_t addr, uint8_t *buf, size_t size)
{
    uint8_t *bufp = buf;
    for (int i=0; i<size; i++, bufp++)
        EEPROM.write(addr+i, *bufp);
    EEPROM.commit();
    return 1;
}

void osc_recv_handler(OSCMessage &msg)
{
    /* Handle /trigger/timeout */
    if (msg.isInt(0)){
        uint32_t data = msg.getInt(0);
        printf("Reiceived new timeout over OSC, writing to EEPROM: %d\n", data);
        eeprom_write(0x00, (uint8_t*)&data, sizeof(data));
        trigger.t_timeout_ms = data;
        led_blink(trigger.led, 10);
    }
    else {
        printf("Received new timeout over OSC, wrong data type\n");
    }
}

void osc_get_msg()
{
    int size;
 
    if((size = Udp.parsePacket()) > 0) {
        OSCMessage msg;
        while(size--)
            msg.fill(Udp.read());
        if (!msg.dispatch("/trigger/timeout", osc_recv_handler))
            printf("Received unhandled message\n");
    }
}

void setup() 
{
    SPI.begin(ETH_CLK, ETH_MISO, ETH_MOSI);
    delay(1000);

    Ethernet.init(ETH_CS); 
    Ethernet.begin(mac, local_ip);
    Udp.begin(local_port);

    if (!EEPROM.begin(sizeof(trigger.t_timeout_ms))) {
        printf("failed to initialize EEPROM\n");
        delay(1000);
        while (1);
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

    uint32_t t_timeout_ms = 0;
    eeprom_read(0x00, (uint8_t*)&t_timeout_ms, sizeof(t_timeout_ms));
    if (t_timeout_ms == 0xFFFFFFFF) {
        printf("No timeout in eeprom, writing default: %d\n", TIMEOUT_DEFAULT);
        uint32_t t_default = TIMEOUT_DEFAULT;
        eeprom_write(0x00, (uint8_t*)&t_default, sizeof(t_default));
    }
    printf("Got timeout from EEPROM: %d\n", t_timeout_ms);
    trigger.t_timeout_ms = t_timeout_ms;
}

void loop() 
{
    wait_for_link();
    if (trigger_check(&trigger))
        osc_send_msg(&trigger);

    osc_get_msg();
    delay(100);

}
