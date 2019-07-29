#include<M5Stack.h>
#include<Math.h>
#include"ydlidar_cmd.h"
#include"doProcess.h"

#define TFT_GREY 0x5AEB // New colour

unsigned char *buff;
unsigned char *dataBuff;
unsigned char *ptr;
uint8_t readByte = 0;
float startAngle = 0;
float endAngle = 0;
char firstData = 0;

void setLcd() {
  M5.begin();
  M5.Lcd.clear(WHITE);
  M5.Lcd.fillScreen(TFT_GREY);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(3, 10);
}

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  setLcd();
//  buff = (unsigned char *)malloc(50);
//  dataBuff = (unsigned char *)malloc(500);
//  memset(dataBuff, 0, 500);
//  ptr = buff;
//  M5.Lcd.print("lidar test start");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()) {
    Serial1.readBytes(buff, 2);
//    M5.Lcd.print(*buff);
    if (buff == NULL) {
      M5.Lcd.println("buff is Null");
//      Serial.println("buff is Null");
    }

    if (buff[0] == 0xAA && buff[1] == 0x55) {
      Serial1.readBytes(buff, 8);

      if (buff[2] == 0x00) {
//        M5.Lcd.print("point cloud data")
      }
      else if (buff[2] == 0x01) {
        //M5.Lcd.print("start data");
      }

//      M5.Lcd.print("receive data number");
//      M5.Lcd.println(buff[3]);
      int dataCount = buff[3];
      
      startAngle = ((buff[4] |(buff[5] << 8)) >> 1);
      startAngle = startAngle / 64.0;
//      M5.Lcd.print("start Angle");
//      M5.Lcd.println(startAngle);

      endAngle = ((buff[6] |( buff[7] << 8)) >> 1);
      endAngle = endAngle / 64.0;
//      M5.Lcd.print("end Angle");
//      M5.Lcd.println(endAngle);

      Serial1.readBytes(dataBuff, (dataCount - 1) * 2);
//      while(Serial1.available()) {
//        Serial1.readBytes(dataBuff, dataCount - 1);         
//      }
      
      float diffAngle = endAngle - startAngle;
      if (diffAngle < 0) {
        diffAngle = endAngle - startAngle + 360;
      }
      float preAngle = diffAngle / (dataCount - 1);

      float angleI = 0;
      float distanceI = 0;
      float angCorrectI = 0;     
      float X = 0;
      float Y = 0;
      //M5.Lcd.fillScreen(TFT_GREY);
      for (int i = 0; i < (dataCount- 1) * 2; i+=2) {
        angleI = preAngle*(i - 1) + startAngle;
        distanceI = (dataBuff[i*2] | (dataBuff[i*2 + 1] << 8));
        distanceI = distanceI / 4.0;
        Serial.println(distanceI);
        if (distanceI > 0) {
          angCorrectI = atan(21.8*(155.3 - distanceI) / (155.3*distanceI))* 180 / 3.141592653589793;
          angleI = angleI + angCorrectI;
        }
        
        Y = sin(angleI * 57.295) * distanceI / 100.0;
        X = cos(angleI * 57.295) * distanceI / 100.0;
        //M5.Lcd.drawPixel(X + 120, Y + 160, WHITE);
      }
//      memset(dataBuff, 0, 500);
//      M5.Lcd.fillScreen(TFT_GREY);
//      M5.Lcd.setCursor(3, 10);
    }
    //ptr = buff;
  }
}
