//
// Created by livello on 28.01.18.
//

#ifndef VENTILATIONCONTROL_ARDUINODUE_ETHERNET_SHIELD_DUE_H
#define VENTILATIONCONTROL_ARDUINODUE_ETHERNET_SHIELD_DUE_H

#endif //VENTILATIONCONTROL_ARDUINODUE_ETHERNET_SHIELD_DUE_H

void setup_ethernet_shield_due();
void broadcastUdpMessage(const char *);
void ethernet_shield_due_loop();
void sendRelayControlPageToEthernetClient();
void updateRelays();