#ifndef DRUMMANAGER_H_
#define DRUMMANAGER_H_

#include  <stdint.h>
#include "n25q128a.h"

#define NUMBER_OF_PADS 6
#define PAD_ADDRESS_START N25Q128A_FLASH_SIZE - N25Q128A_SECTOR_SIZE // Last page

void initHelloDrums(void);
void requestPiezo(void);
void checkHelloDrums(void);
void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
void readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);

#endif /* DRUMMANAGER_H_ */
