#include<M5Stack.h>
#include "sdk/include/angles.h"
#include "doProcess.h"
#include "mapData.h"
#include "espnow.h"
#define TFT_GREY 0x5AEB // New colour

int state = 0;

DisplayData dismap;
DisplayData tmpmap;
DisplayData oldmap;

unsigned char *buff;
unsigned char *dataBuff;
unsigned char *ptr;
uint8_t readByte = 0;
float startAngle = 0;
float endAngle = 0;
char firstData = 0;
int disPlayFlag = 0;
int sendingFlag = 0;

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

      float oldX = (sin(oldAng) * oldmap.mapdata[i]/5);
      float oldY = (cos(oldAng) * oldmap.mapdata[i]/5);
      float X = (sin(Ang) * dismap.mapdata[i]/5);
      float Y = (cos(Ang) * dismap.mapdata[i]/5);

      if (oldmap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-oldX + 160, oldY + 120, BLACK);  
      }

      if (dismap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-X + 160, Y + 120, WHITE);  
      }

      oldmap.mapdata[i] = dismap.mapdata[i];
  }
}

void bufCopy(uint8_t *buff, uint8_t len) {
    float angleI = 0;
    float distanceI = 0;
    float angCorrectI = 0;     
    float X = 0;
    float Y = 0;
   
    int leng = len / 2;
    float preAngle = endAngle - startAngle;
    if (preAngle < 0.0) {
      preAngle = preAngle + 360.0 * 64;
    }
    preAngle = preAngle/((leng - 1) * 1.0);

    for (int i = 0; i < len; i+= 2) {
      angleI = preAngle*(i/2.0) + startAngle;
      distanceI = int((buff[i] | (buff[i + 1] << 8)) / 4.0);

      angCorrectI = (atan(21.8*(155.3 - distanceI) / (155.3*distanceI)) * 180 / 3.1415) * 64.0;
      angleI = (angleI + angCorrectI) / 64.0;

      if (angleI > 360.0) {
        angleI = angleI - 360;
      }

//      if (angleI > 30 && angleI < 90) {
//        Serial.printf("angleI:%f, distance:%f\r\n", angleI, distanceI);
//      }
      angleI = angleI * 3.1415 / 180;
//      angleI = normalize_angle(angleI);
  
      int recvAngle = floorf(angleI * 180 / 3.1415 * 2);
      float recvDistanceI = distanceI;
     
      tmpmap.mapdata[recvAngle] = distanceI;
//      Serial.printf("ori:%f dist:%d\r\n", distanceI, uint16_t(distanceI));
      tmpData[recvAngle] = uint16_t(distanceI);
    }
}
/*
0xaa, 0x55, 0x0, 0x28, 0x2f, 0x69, 0xb3, 0x76,
0x6b, 0x33,
0xcc, 0x13, 0x79,
0x13, 0x9d, 0x13, 0xc8, 0x13, 0xf0, 0x13, 0xd, 0x14, 0x2d, 0x14, 0x51,
0x14, 0x75, 0x14, 0x9d, 0x14, 0xc1, 0x14, 0xe9, 0x14, 0x11, 0x15, 0x35,
0x15, 0x5d, 0x15, 0x91, 0x15, 0xbd, 0x15, 0xf1, 0x15, 0x25, 0x16, 0x64,
0x16, 0x94, 0x16, 0xd4, 0x16, 0xf8, 0x16, 0xc8, 0x16, 0x94, 0x16, 0xa8,
0x16, 0xd4, 0x16, 0xd, 0x17, 0x51, 0x17, 0x95, 0x17, 0xe5, 0x17, 0x31,
0x18, 0x94, 0x18, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x32, 0x3e,
0x38, 0x3e, 0x2, 0x40, 0xaa, 0x55, 0x0, 0x28, 0xd, 0x77, 0x97, 0x84,
0x82, 0xb3, 0x5e, 0x8, 0x7, 0x8, 0xe2, 0x8, 0x2, 0x42, 0xda, 0x44, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x16, 0x4b, 0x18, 0x4a, 0xb4, 0x4a, 0xfc, 0x4a,
0xa8, 0x4a, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x16, 0x30, 0x2a,
0x48, 0x0, 0x0, 0x9a, 0x1, 0x9d, 0x1, 0xa9, 0x1, 0xa5, 0x1, 0xa9, 0x1,
0xa5, 0x1, 0xa5, 0x1, 0xad, 0x1, 0xb5, 0x1, 0xb5, 0x1, 0xb9, 0x1, 0xb5,
0x1, 0xad, 0x1, 0xa9, 0x1, 0xa5, 0x1, 0xa1, 0x1, 0x99, 0x1, 0x99, 0x1,
0x99, 0x1, 0x99, 0x1
*/

uint8_t msg_data[] = {0xaa, 0x55, 0x0, 0x28, 0x2f, 0x69, 0xb3, 0x76};

uint8_t data[] = {
    0xcc, 0x13, 0x79, 0x13, 0x9d, 0x13, 0xc8, 0x13, 0xf0, 0x13, 0xd,  0x14,
    0x2d, 0x14, 0x51, 0x14, 0x75, 0x14, 0x9d, 0x14, 0xc1, 0x14, 0xe9, 0x14,
    0x11, 0x15, 0x35, 0x15, 0x5d, 0x15, 0x91, 0x15, 0xbd, 0x15, 0xf1, 0x15,
    0x25, 0x16, 0x64, 0x16, 0x94, 0x16, 0xd4, 0x16, 0xf8, 0x16, 0xc8, 0x16,
    0x94, 0x16, 0xa8, 0x16, 0xd4, 0x16, 0xd,  0x17, 0x51, 0x17, 0x95, 0x17,
    0xe5, 0x17, 0x31, 0x18, 0x94, 0x18, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
    0x0,  0x0,  0x32, 0x3e, 0x38, 0x3e, 0x2,  0x40
};

uint16_t cover_crc(uint8_t *msg, uint8_t *data, uint16_t data_len) {
  uint16_t crc = 0x000;
  uint16_t length = data_len / 2;
  uint16_t *msg_ptr = (uint16_t *)msg;
  uint16_t *data_ptr = (uint16_t *)data;

  if(data_len % 2 == 1) {
    return 0xffff;
  }

  for (uint16_t i = 0; i < 4; i++) {
    crc ^= *msg_ptr++;
  }

  for(uint16_t i = 0; i < length; i++) {
    crc ^= *data_ptr++;
  }

  return crc >> 8 | crc << 8;
}

typedef struct {
  uint8_t buf[1024];
  uint16_t length;
} _lidar_data_t;

_lidar_data_t lidar_data;

void lidar_data_deal(uint8_t data_in) {
  static uint8_t state = 0;
  static uint8_t last_value = 0x00;
  static uint8_t rx_buf[1024] = {0};
  static uint8_t rx_ptr;

  rx_buf[rx_ptr++] = data_in;

  if(rx_ptr > 1022) {
    rx_ptr = 0;
  }

  if(data_in == 0x55 && last_value == 0xaa) {
    state = 1;
  }
  
  if(state == 1) {
    state = 0;
    if(rx_ptr > 10 && rx_buf[0] == 0xaa && rx_buf[1] == 0x55) {
      memcpy(lidar_data.buf, rx_buf, 8);
      memcpy(&lidar_data.buf[8], &rx_buf[10], rx_ptr - 2 - 10);
      lidar_data.length = rx_ptr;
//      Serial.printf("got len %d, len should be %d\r\n", rx_ptr - 2, rx_buf[3]*2 + 10);
//      Serial.printf("go crc: %x, crc need be %x\r\n", cover_crc(lidar_data.buf, &lidar_data.buf[8], rx_ptr - 2 - 10), rx_buf[8] << 8 | rx_buf[9]);
      startAngle = float((lidar_data.buf[4] |(lidar_data.buf[5] << 8)) >> 1);
      endAngle = float((lidar_data.buf[6] |( lidar_data.buf[7] << 8)) >> 1);

//      Serial.printf("startAngle: %f, endAngle: %f \r\n", startAngle / 64, endAngle / 64);
      //check bcc
      uint16_t bcc = cover_crc(lidar_data.buf, &lidar_data.buf[8], rx_ptr - 2 - 10);
      if (bcc == ((rx_buf[8] << 8) | rx_buf[9])) {
//        Serial.printf("start: %f , end: %f \r\n", startAngle /64, endAngle / 64);
        if (lidar_data.buf[2] == 0x01&& disPlayFlag == 0) {
          memcpy(dismap.mapdata, tmpmap.mapdata, sizeof(float) * 720);
//          memcpy(sendingMap.mapdata, tmpmap.mapdata, sizeof(float) * 720);
          disPlayFlag = 1;
//          sendingFlag = 1;
          memset(tmpmap.mapdata, 0, sizeof(float) * 1000);
        }
        else {
          bufCopy(&lidar_data.buf[8], rx_ptr - 2 - 10);  
        }
        
      }
      else {
        //crc error or startAng < endAng
        Serial.printf("startAngle: %f, endAngle: %f \r\n", startAngle / 64, endAngle / 64);
      }
    }
    memset(rx_buf, 0, rx_ptr);
    rx_ptr = 0;
    rx_buf[rx_ptr++] = 0xaa;
    rx_buf[rx_ptr++] = 0x55;
  }
  last_value = data_in;
}

static void map_send(void *arg){
  Serial.printf("map_send\r\n");
  while(1) {
//    sendMap();
    delay(20);
  }
}

void sendMap() {
//   ScanForSlave();

    // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
     if (isPaired) {
      // pair success or already paired
      // Send data to device

        /*
        
        for (int i = 0; i < 180; i++) {
          sendingData[i] = tmpData[i*4 + 180];
          //Serial.printf("I:%d, distance:%d\r\n", i, tmpData[i*4]);
        }
        sendData(sendingData, sizeof(uint16_t) * 90);
        */

        uint8_t head[2] = {0xaa, 0x55};
        sendData(head, 2);
        for (int i = 0; i < 6; i++) {
          sendData(tmpData + i * 120, sizeof(uint16_t) * 120);
        }
        memset(tmpData, 0, 1500);
        
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
    if (disPlayFlag){
      sendMap();
      disPlay(); 
      disPlayFlag = 0;
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

  InitESPNow();

//  xTaskCreatePinnedToCore(map_send, "map", 5 * 1024, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(dis_task, "lidar", 5 * 1024, NULL, 2, NULL, 0);
}

void loop() {
//  readBtn();
  while (Serial1.available()) {
    lidar_data_deal(Serial1.read());
  }
  
  delay(1);
}
