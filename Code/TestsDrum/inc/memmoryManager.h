#ifndef MEMMORYMANAGER_H_
#define MEMMORYMANAGER_H_

#include <stdint.h>
#include <stdio.h>

void eraseSector(void);
void initMemory(uint8_t padNum);

class TestClass{
public:
	uint8_t simpleVal = 9;
	uint8_t someAnother = 32;

	void printString(void){
		printf("Hello this is test class");
	}
};

void changeValueInFlash(void);

#endif /* MEMMORYMANAGER_H_ */
