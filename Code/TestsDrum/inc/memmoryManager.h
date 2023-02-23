#ifndef MEMMORYMANAGER_H_
#define MEMMORYMANAGER_H_

#include <stdint.h>
#include <stdio.h>

void eraseSector(void);
void initMemory(void);

void mainWork(void);

void changeValueInFlash(void);

#endif /* MEMMORYMANAGER_H_ */
