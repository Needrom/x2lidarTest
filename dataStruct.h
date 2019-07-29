#ifndef __DATASTRUCT_H
#define __DATASTRUCT_H

//struct of data package
class Data {
public:
	Data(int buff_size){
		dataPackage = (unsigned char *)malloc(sizeof(unsigned char) * buff_size);
		dataPtr = dataPackage;
	}

	unsigned char *dataPtr = NULL;
	unsigned char *dataPackage = NULL;
	int dataType  = 0;
	int numOfPoi  = 0;
	int startAng  = 0;
	int endAng    = 0;
	int crc		  = 0;
};



#endif 
