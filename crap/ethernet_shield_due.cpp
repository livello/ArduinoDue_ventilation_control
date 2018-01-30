//
// Created by livello on 28.01.18.
//

#include "ethernet_shield_due.h"
#include "sd_card_w5100.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <EthernetServer.h>
#include <IPAddress.h>
#include <ModbusIP.h>


#define UDP_BROADCAST_PORT 3000
#define SWITCH_ISTS 100

IPAddress ip_default(192, 168, 6, 20);
IPAddress ip_udp_client(192, 168, 6, 1);
EthernetServer server(80);
EthernetClient client;
EthernetUDP udpEthernet;
IPAddress broadcastIp;

ModbusIP modbusIP;

extern int pinState[];  // Состояние пинов
extern int relayPins[];
extern float humidity_DHT22, temp_DHT22;
extern float temp_DS18B20;
long udpMessageCount = 0;
char message_buffer[100];


byte mac[] = {
        0xDE,
        0xAD,
        0xBE,
        0xEF,
        0xFE,
        0xED};

void setup_ethernet_shield_due(){
    Ethernet.begin(mac);
    server.begin();
    udpEthernet.begin(UDP_BROADCAST_PORT);
    broadcastIp = Ethernet.localIP();
    broadcastIp[3] = 255;

    modbusIP.config(mac, Ethernet.localIP());
    modbusIP.addIsts(SWITCH_ISTS);
    modbusIP.addHreg(SWITCH_ISTS - 10);
    modbusIP.addHreg(SWITCH_ISTS - 20);

}
void broadcastUdpMessageTest() {
    udpEthernet.beginPacket(ip_udp_client, UDP_BROADCAST_PORT);
    udpEthernet.write("Hello from Arduino Due");
    udpEthernet.endPacket();
}

void broadcastUdpMessage(const char *message) {
    char internal_message_buffer[100];
    udpMessageCount++;
    sprintf(internal_message_buffer, "%ld_%ld: %s", udpMessageCount, millis(), message);
    udpEthernet.beginPacket(broadcastIp, UDP_BROADCAST_PORT);
    udpEthernet.write(internal_message_buffer);
    udpEthernet.endPacket();
}
void ethernet_shield_due_loop(){
    modbusIP.task();
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
            if (clientRequest.indexOf("r0=on") > 0)
                pinState[0] = 1;
            else
                pinState[0] = 0;
            if (clientRequest.indexOf("r1=on") > 0)
                pinState[1] = 1;
            else
                pinState[1] = 0;
            if (clientRequest.indexOf("r2=on") > 0)
                pinState[2] = 1;
            else
                pinState[2] = 0;
            if (clientRequest.indexOf("r3=on") > 0)
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
void sendRelayControlPageToEthernetClient() {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html>");
    client.println("<head>");
    client.println("<meta http-equiv=\"refresh\" content=\"30\">");
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
void updateRelays() {
    for (int i = 0; i < 4; i++) {
        if (pinState[i])
            digitalWrite(relayPins[i], HIGH);
        else
            digitalWrite(relayPins[i], LOW);
    }
}