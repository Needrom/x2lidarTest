#include<M5Stack.h>
//#include "sdk/include/angles.h"
#include "doProcess.h"
#include "mapData.h"
#include "espnow.h"
#include "X2driver.h"
#define TFT_GREY 0x5AEB // New colour

int state = 0;

EspNow espnow;
X2 lidar;

void setLcd() {
  M5.Lcd.clear(WHITE);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(3, 10);
  M5.Lcd.println("lidar Test");
}

void disPlay(void) {
  Serial.printf("display \r\n");
  for (int i = 0; i < 720; i++) {
      float oldAng = from_degrees((i / 2.0));
      float Ang = from_degrees((i / 2.0));

      float oldX = (sin(oldAng) * lidar.oldmap.mapdata[i]/5);
      float oldY = (cos(oldAng) * lidar.oldmap.mapdata[i]/5);
      float X = (sin(Ang) * lidar.dismap.mapdata[i]/5);
      float Y = (cos(Ang) * lidar.dismap.mapdata[i]/5);

      if (lidar.oldmap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-oldX + 160, oldY + 120, BLACK);  
      }

      if (lidar.dismap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-X + 160, Y + 120, WHITE);  
      }

      lidar.oldmap.mapdata[i] = lidar.dismap.mapdata[i];
  }
}

void sendMap() {
//   ScanForSlave();

    // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (espnow.slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = espnow.manageSlave();
     if (isPaired) {
      // pair success or already paired
      // Send data to device

        uint8_t head[2] = {0xaa, 0x55};
        espnow.sendData(head, 2);
        for (int i = 0; i < 6; i++) {
          espnow.sendData(lidar.tmpData.mapdata + i * 120, sizeof(uint16_t) * 120);
        }
        memset(lidar.tmpData.mapdata, 0, 1500);
        
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }
}

static void dis_task(void *arg) {
  Serial.printf("uart_task\r\n");
  while(1) {
    if (lidar.disPlayFlag){
      sendMap();
      disPlay(); 
      lidar.disPlayFlag = 0;
    }
    delay(30);
  }
}

void setup() {
  Serial.begin(230400);
  M5.begin(true, false, false);
  
  // put your setup code here, to run once:
//  BtnSet();
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  setLcd();
  //Serial.printf("got crc %x\r\n", cover_crc(msg_data, data, 80));
  M5.Lcd.drawCircle(160, 120 , 4, RED);

  espnow.InitESPNow();

  xTaskCreatePinnedToCore(dis_task, "lidar", 5 * 1024, NULL, 2, NULL, 0);
}

void loop() {
//  readBtn();
  while (Serial1.available()) {
    lidar.lidar_data_deal(Serial1.read());
  }
  delay(1);
}
