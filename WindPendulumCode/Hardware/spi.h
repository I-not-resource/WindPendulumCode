#ifndef __SPI_H_
#define __SPI_H_
#include "Sys.h"
 #if __SPI_ENABLE

void SPIx1_Init(void);
void SPIx2_Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI1_ReadWriteByte(u8 TxData);
u8 SPI2_ReadWriteByte(u8 TxData);



 #endif //__SPI_ENABLE

#endif // __SPI_H_

