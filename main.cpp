/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */
#include "Arduino.h"
#include <HardwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <IPAddress.h>
#include <WMath.h>
#include <EthernetServer.h>

#include "ChibiOS_ARM.h"
#include "dht.h"
#include "OneWire.h"
#include <ky040_encoder.h>
#include "sd_card_w5100.h"

#include <Modbus.h>
#include <ModbusIP.h>

#define SWITCH_ISTS 100

#define BOARD_LED           13      // I'm alive blinker
#define DHTPIN 33
#define LED_PIN 13
#define ONEWIRE_PIN 12
#define UDP_BROADCAST_PORT 3000

constexpr uint8_t pin_onewire{35};

dht dht_sensor;

OneWire dallas18b20(ONEWIRE_PIN);

int value = 0;
SEMAPHORE_DECL(sem, 0);
long udpMessageCount = 0;
char message_buffer[100];


byte mac[] = {
        0xDE,
        0xAD,
        0xBE,
        0xEF,
        0xFE,
        0xED};
ModbusIP modbusIP;
IPAddress ip_default(192, 168, 6, 20);
IPAddress ip_udp_client(192, 168, 6, 1);
EthernetServer server(80);
EthernetClient client;
EthernetUDP udpEthernet;
IPAddress broadcastIp;


int ethernet_config_by_hand = 0;
const int numPins = 4;
const int pins[] = {4, 5, 6, 7};    // Пины для реле
int pinState[] = {0, 0, 0, 0};  // Состояние пинов
int relayPins[] = {22,5,6,7};
float humidity_DHT22 = 0, temp_DHT22 = 0;
float temp_DS18B20;

struct {
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t unknown;
} stat1 = {0, 0, 0, 0, 0};

void sendRelayControlPageToEthernetClient();

void updateRelays();
void broadcastUdpMessage(const char*);

void show_ds18b20() {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];


    if (!dallas18b20.search(addr)) {
        Serial.println("No more addresses.");
        Serial.println();
        dallas18b20.reset_search();
        delay(250);
        return;
    }

    Serial.print("ROM =");
    for (i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0]) {
        case 0x10:
            Serial.println("  Chip = DS18S20");  // or old DS1820
            type_s = 1;
            break;
        case 0x28:
            Serial.println("  Chip = DS18B20");
            type_s = 0;
            break;
        case 0x22:
            Serial.println("  Chip = DS1822");
            type_s = 0;
            break;
        default:
            Serial.println("Device is not a DS18x20 family device.");
            return;
    }

    dallas18b20.reset();
    dallas18b20.select(addr);
    dallas18b20.write(0x44, 1);        // start conversion, with parasite power on at the end

//    delay(1000);     // maybe 750ms is enough, maybe not
    chThdSleepMilliseconds(1000);
    // we might do a dallas18b20.depower() here, but the reset will take care of it.

    present = dallas18b20.reset();
    dallas18b20.select(addr);
    dallas18b20.write(0xBE);         // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = dallas18b20.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    temp_DS18B20 = (float) raw / 16.0;
    Serial.print("  Temperature = ");
    Serial.print(temp_DS18B20);
    Serial.print(" Celsius, ");

}

static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(Thread1, arg) {

    while (!chThdShouldTerminateX()) {
        // Wait for signal from thread 2.
        chSemWait(&sem);

        // Turn LED off.
        digitalWrite(LED_PIN, LOW);

        // DISPLAY DATA
        uint32_t start = micros();
        int chk = dht_sensor.read22(DHTPIN);
        uint32_t stop = micros();

        stat1.total++;
        switch (chk) {
            case DHTLIB_OK:
                stat1.ok++;
                humidity_DHT22 = dht_sensor.humidity;
                temp_DHT22 = dht_sensor.temperature;
                Serial.print(humidity_DHT22);
                Serial.print(" %, temperature:");
                Serial.println(temp_DHT22);
                break;
            case DHTLIB_ERROR_CHECKSUM:
                stat1.crc_error++;
                Serial.print("Checksum error,\t");
                break;
            case DHTLIB_ERROR_TIMEOUT:
                stat1.time_out++;
                Serial.print("Time out error,\t");
                break;
            default:
                stat1.unknown++;
                Serial.print("Unknown error,\t");
                break;
        }

        Serial.print("A0:");
        Serial.print((analogRead(A0) / 4096.0 - 0.5) * 5000 - 175);
        Serial.print(" raw;");
        show_ds18b20();
    }
}


static THD_WORKING_AREA(waThread2, 64);
void broadcastUdpMessageTest(){
    udpEthernet.beginPacket(ip_udp_client, UDP_BROADCAST_PORT);
    udpEthernet.write("Hello from Arduino Due");
    udpEthernet.endPacket();
}
static THD_FUNCTION(Thread2, arg) {
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, HIGH);

        // Sleep for 200 milliseconds.
        chThdSleepMilliseconds(2000);

        // Signal thread 1 to turn LED off.
        chSemSignal(&sem);

        // Sleep for 200 milliseconds.
        chThdSleepMilliseconds(2000);

        sprintf(message_buffer, "Temp DHT22:%f. Humidity: %f%. Temp DS18B20:%f C \n", humidity_DHT22, temp_DHT22, temp_DS18B20);
        broadcastUdpMessage(message_buffer);
        broadcastUdpMessageTest();
    }
}

void broadcastUdpMessage(const char* message){
    char internal_message_buffer[100];
    udpMessageCount++;
    sprintf(internal_message_buffer, "%ld_%ld: %s", udpMessageCount, millis(), message);
    udpEthernet.beginPacket(broadcastIp, UDP_BROADCAST_PORT);
    udpEthernet.write(internal_message_buffer);
    udpEthernet.endPacket();
}

void threadChildsSetup() {

    // start blink thread
    chThdCreateStatic(waThread1, sizeof(waThread1),
                      NORMALPRIO + 2, Thread1, NULL);

    chThdCreateStatic(waThread2, sizeof(waThread2),
                      NORMALPRIO + 1, Thread2, NULL);

}

void setup() {
    Serial.begin(115200);
    delay(10);
//    setup_ky040();
    sd_card_w5100_setup();

    Ethernet.begin(mac);
    server.begin();
    udpEthernet.begin(UDP_BROADCAST_PORT);

    Serial.print("server is at ");
    delay(2000);
    Serial.println(Ethernet.localIP());
    broadcastIp = Ethernet.localIP();
    broadcastIp[3] = 255;

    modbusIP.config(mac, Ethernet.localIP());
    modbusIP.addIsts (SWITCH_ISTS);
    modbusIP.addHreg(SWITCH_ISTS-10);
    modbusIP.addHreg(SWITCH_ISTS-20);

    for(int i=0;i<numPins;i++)
    pinMode(pins[i], OUTPUT);

    analogReadResolution(12);
    chBegin(threadChildsSetup);


}

void loop() {
    delay(10);
    modbusIP.task ();
    client = server.available();

    if (client) {
        String webRequestType = client.readStringUntil('/');
        if (webRequestType.compareTo("GET ") == 0) {
            String requestedFileName = client.readStringUntil(' ');

            Serial.println("GET");
            Serial.println(requestedFileName);
            if (requestedFileName.length() > 0)
                client.write(sdW5100_readEntireFile(requestedFileName.c_str()),
                             sdW5100_getFileSize(requestedFileName.c_str()));
            else {
                sendRelayControlPageToEthernetClient();
            }
        } else if (webRequestType.compareTo("POST ") == 0) {
            String clientRequest = client.readString();
            if (clientRequest.indexOf("r0=on")>0)
                pinState[0] = 1;
            else
                pinState[0] = 0;
            if (clientRequest.indexOf("r1=on")>0)
                pinState[1] = 1;
            else
                pinState[1] = 0;
            if (clientRequest.indexOf("r2=on")>0)
                pinState[2] = 1;
            else
                pinState[2] = 0;
            if (clientRequest.indexOf("r3=on")>0)
                pinState[3] = 1;
            else
                pinState[3] = 0;
            updateRelays();


            sendRelayControlPageToEthernetClient();
        } else {
            Serial.println("." + webRequestType + ".");
            Serial.println("" + client.readString());
            sendRelayControlPageToEthernetClient();
        }
        client.stop();
    }
}

void updateRelays() {
    for(int i=0; i < 4; i++){
                if(pinState[i])
                    digitalWrite(relayPins[i],HIGH);
                else
                    digitalWrite(relayPins[i],LOW);
            }
}

void sendRelayControlPageToEthernetClient() {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html>");
    client.println("<head>");
    client.println( "<meta http-equiv=\"refresh\" content=\"30\">");
    client.println("<title>Zelectro. Relay + Ethernet shield.</title>");
    client.println("</head>");
    client.println("<body>");
    client.println("<h3>Zelectro. Relay + Ethernet shield.</h3>");
    client.println("<form method='post'>");
    client.print("<div>Relay 1 <input type='checkbox' ");
    if (pinState[0] == 1)
        client.print("checked");
    client.println(" name='r0'></div>");
    client.print("<div>Relay 2 <input type='checkbox' ");
    if (pinState[1] == 1)
        client.print("checked");
    client.println(" name='r1'></div>");
    client.print("<div>Relay 3 <input type='checkbox' ");
    if (pinState[2] == 1)
        client.print("checked");
    client.println(" name='r2'></div>");
    client.print("<div>Relay 4 <input type='checkbox' ");
    if (pinState[3] == 1)
        client.print("checked");
    client.println(" name='r3'></div>");
    client.println("<input type='submit' value='Refresh'>");
    client.println("</form>");
    client.println("Temperature:");
    client.print(temp_DS18B20);
    client.print(" celsius, ");
    client.print(temp_DHT22);
    client.print(" celsius, humidity(%):");
    client.print(humidity_DHT22);
    client.println("</body>");
    client.println("</html>");
}