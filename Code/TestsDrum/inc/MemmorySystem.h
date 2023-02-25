#ifndef MEMMORYSYSTEM_H_
#define MEMMORYSYSTEM_H_

#include <stdint.h>
#include <stdio.h>

void eraseSector(void);
void initMemory(void);

void mainWork(void);

void changeValueInFlash(void);

class MemmorySystem {
public:
	static MemmorySystem* getInstance() {
		static MemmorySystem instance;
		return &instance;
	}
    void test();

private:
    MemmorySystem();
};

#endif /* MEMMORYSYSTEM_H_ */
