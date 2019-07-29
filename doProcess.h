#ifndef __DOPROCESS_H
#define __DOPROCESS_H

#include "dataStruct.h"
Data data(500);

uint8_t state = 0;

uint8_t doProcess(int *_state) {
	switch (*_state) {
		//all the begining
		case 0:
		{
			unsigned char *buff = NULL;
			buff = (unsigned char *)malloc(sizeof(unsigned char) * 2);
			Serial.readBytes(buff, 2);
			if (buff[0] == 0xAA && buff[1] == 0x55) {
				*_state++;
				memcpy(data.dataPtr, buff, 2);
				data.dataPtr += 2;
			}
			free(buff);
			
			break;
		}
		//check all the flag and param here
		case 1:
		{
			unsigned char *buff = NULL;
			buff = (unsigned char *)malloc(sizeof(unsigned char) * 6);
			Serial.readBytes(buff, 6);
			
			//Whether package is the start of the circle
			if (buff[0] == 0x00) {
				//point cloud data
				data.dataType = 0;
			}
			else if (buff[0] == 0x01) {
				data.dataType = 1;
				//start package
			}
			else {
				//error 
				;;
			}
			
			//the number of the data point
			data.numOfPoi = int(buff[1]);

			data.startAng = int(buff[2] | (buff[3] << 8)) >> 1;

			data.endAng   = int(buff[4] | (buff[5] << 8)) >> 1;
			memcpy(data.dataPtr, buff, 6);
			data.dataPtr += 6;
			*_state++;

			break;
		}
		case 2:
		{
			unsigned char *bcc = NULL;
			bcc = (unsigned char *)malloc(sizeof(unsigned char) * 2);
			Serial.readBytes(bcc, 2);

			//alloc the mem to the data
			//2 Byte for one point
			unsigned char *buff = NULL;
			buff = (unsigned char *)malloc(sizeof(unsigned char) * 2 * data.numOfPoi);
			Serial.readBytes(buff, 2 * data.numOfPoi);
			memcpy(data.dataPtr, buff, 2 * data.numOfPoi);
			
			//bcc check
			char *bccBuff = (char *)data.dataPackage;
			char bccOut  = 0x0000;
			for (int i = 0; i < data.numOfPoi + 4;) {
				bccOut ^= bccBuff[i];
			}
			if (bccOut != *(char *)bcc) {
				Serial.println("bcc check failed");
			}			

			//back to the initial state
			*_state = 0;
			break;
		}
	}
}

#endif
