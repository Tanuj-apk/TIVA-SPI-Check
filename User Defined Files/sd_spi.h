#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>

void SPI_Init(void);
uint8_t SPI_Transfer(uint8_t data);

void SD_Select(void);
void SD_Deselect(void);

int SD_Init(void);
int SD_ReadBlock(uint32_t sector, uint8_t *buffer);
int SD_WriteBlock(uint32_t sector, const uint8_t *buffer);

#endif
