#ifndef _G_CONFIG_H
#define _G_CONFIG_H

#define SPI_MAX_DMA_BYTES   4096
#define SPI_DUMMY_BYTE      0xFF
#define MAX_DATA_R_BUF_SIZE      2 * 1024 * 1024
#define VALID_DATA_OFFSET 0
#define IMX500_HEADER_LEN 12

#define REG_DATA_SIZE_0  0x701  // Data size byte 0 (MSB)
#define REG_DATA_SIZE_1  0x702  // Data size byte 1
#define REG_DATA_SIZE_2  0x703  // Data size byte 2
#define REG_DATA_SIZE_3  0x704  // Data size byte 3 (LSB)
#define REG_DATA_READY   0x705 
#define REG_DATA_START   0x706


#endif  // _G_CONFIG_H