#include "X2driver.h"

X2::X2(void) {
  ;;
}

uint16_t X2::cover_crc(uint8_t *msg, uint8_t *data, uint16_t data_len) {
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

void X2::lidar_data_deal(uint8_t data_in) {
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

void X2::bufCopy(uint8_t *buff, uint8_t len) {
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
      tmpData.mapdata[recvAngle] = uint16_t(distanceI);
    }
}
