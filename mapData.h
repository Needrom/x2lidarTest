#ifndef __MAPDATA_H
#define __MAPDATA_H

struct MapData {
	float mapdata[1024] = {0};
	int datalen = 0;
};

struct DisplayData {
	float mapdata[1500] = {0};
	int count = 0;
};

uint16_t sendingData[1500] = {0};
uint16_t tmpData[1500] = {0};

#endif
