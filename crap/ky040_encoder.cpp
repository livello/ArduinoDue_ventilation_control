#include "Arduino.h"
#define PinCLK 49                   // Used for generating interrupts using CLK signal
#define PinDT 51                    // Used for reading DT signal
#define PinSW 53                    // Used for the push button switch

volatile boolean TurnDetected;
volatile boolean up;


static long virtualPosition = 0;    // without STATIC it does not count correctly!!!
static boolean allow_next_step = false;
static int relay_on = 0;

void update_relays(){
    if(relay_on>0)
    digitalWrite(3+relay_on,LOW);
    relay_on = virtualPosition  / 5;
    if(relay_on>0)
    digitalWrite(3+relay_on,HIGH);
}

void ky040_CLK_interrupt() {
    Serial.print("+");
    if (!allow_next_step)
        return;
    allow_next_step = false;
    if (digitalRead(PinDT))
        virtualPosition++;
    else
        virtualPosition--;
    if(virtualPosition<0)
        virtualPosition=0;
    else if(virtualPosition>24)
        virtualPosition=24;
    if(abs(relay_on*5-virtualPosition)>2)
    update_relays();
}

void ky040_DT_interrupt() {
    Serial.print("-");
    allow_next_step = true;
}

void ky040_SW_interrupt() {
    Serial.print("Reset = ");      // Using the word RESET instead of COUNT here to find out a buggy encoder
    Serial.println(virtualPosition);
    virtualPosition = 0;
}


void setup_ky040() {
//    pinMode(PinCLK, INPUT);
//    pinMode(PinDT, INPUT);
//    pinMode(PinSW, INPUT);
    Serial.println("Setup ky040");
    attachInterrupt(PinCLK, ky040_CLK_interrupt, RISING);
    attachInterrupt(PinDT, ky040_DT_interrupt, RISING);
    attachInterrupt(PinSW, ky040_SW_interrupt, RISING);
}

void demo_ky040() {
        Serial.print("Count = ");
        Serial.println(virtualPosition);

}