#ifndef __ESPNOW_H
#define __ESPNOW_H

#include <esp_now.h>
#include <WiFi.h>

//#define CHANNEL 1

#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

class EspNow {

public:
	esp_now_peer_info_t slave;

	EspNow();
	void InitESPNow(void);
	void deletePeer(void); 
	static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
	void ScanForSlave(void);
	void sendData(void *buf, int len);
	bool manageSlave(void);

};
#endif
