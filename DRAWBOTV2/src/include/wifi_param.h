#pragma once

// PC qui reçoit Teleplot
extern const char* hostIP;
extern const int teleplotPort;

// GUI TCP
#define GUI_PORT 1234

void wifiInit();
void wifiHandleClient();
void wifiSendTelemetry();
void wifiSendToGUI();