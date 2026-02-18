#include <stdint.h>
#include <stdbool.h>
#include <sd_spi.h>
#include <ff.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

#define SD_CS_PORT GPIO_PORTA_BASE
#define SD_CS_PIN  GPIO_PIN_3

#define CMD0   0
#define CMD8   8
#define CMD17  17
#define CMD24  24
#define CMD55  55
#define ACMD41 41

void SD_Select(void)
{
    GPIOPinWrite(SD_CS_PORT, SD_CS_PIN, 0);
}

void SD_Deselect(void)
{
    GPIOPinWrite(SD_CS_PORT, SD_CS_PIN, SD_CS_PIN);
}

void SD_PowerDelay(void)
{
    for(volatile uint32_t i = 0; i < 500000; i++);
}

void SPI_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, SD_CS_PIN);

    SD_Deselect();

    SSIConfigSetExpClk(SSI0_BASE,
                       SysCtlClockGet(),
                       SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER,
                       400000,      // 400 kHz for init
                       8);

    SSIEnable(SSI0_BASE);
}

uint8_t SPI_Transfer(uint8_t data)
{
    uint32_t rx;
    SSIDataPut(SSI0_BASE, data);
    SSIDataGet(SSI0_BASE, &rx);
    return (uint8_t)rx;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r;
    uint16_t retry = 0;

    SD_Deselect();
    SPI_Transfer(0xFF);
    SD_Select();

    SPI_Transfer(0x40 | cmd);
    SPI_Transfer(arg >> 24);
    SPI_Transfer(arg >> 16);
    SPI_Transfer(arg >> 8);
    SPI_Transfer(arg);
    SPI_Transfer(crc);

    do {
        r = SPI_Transfer(0xFF);
    } while (r == 0xFF && retry++ < 500);

    return r;
}

static uint8_t sd_is_sdhc = 0;

int SD_Init(void)
{
    uint8_t r;
    uint8_t ocr[4];

    /* Ensure CS high and give initial clocks */
    SD_Deselect();
    SPI_Transfer(0xFF);

    /* Power-up delay (important) */
    SD_PowerDelay();

    /* 80 dummy clocks with CS high */
    SD_Deselect();
    for (int i = 0; i < 10; i++)
        SPI_Transfer(0xFF);

    /* CMD0: Go to idle state */
    r = SD_SendCommand(CMD0, 0, 0x95);
    SD_Deselect();
    SPI_Transfer(0xFF);

    if (r != 0x01)
        return -1;

    /* CMD8: Voltage check */
    r = SD_SendCommand(CMD8, 0x1AA, 0x87);

    /* Read R7 response (4 bytes) */
    ocr[0] = SPI_Transfer(0xFF);
    ocr[1] = SPI_Transfer(0xFF);
    ocr[2] = SPI_Transfer(0xFF);
    ocr[3] = SPI_Transfer(0xFF);

    SD_Deselect();
    SPI_Transfer(0xFF);

    if (r != 0x01)
        return -2;

    /* ACMD41 initialization loop */
    do {
        r = SD_SendCommand(CMD55, 0, 0x01);
        SD_Deselect();
        SPI_Transfer(0xFF);

        r = SD_SendCommand(ACMD41, 0x40000000, 0x01);
        SD_Deselect();
        SPI_Transfer(0xFF);
    } while (r != 0x00);

    /* CMD58: Read OCR to detect SDHC */
    r = SD_SendCommand(58, 0, 0x01);
    if (r != 0x00)
        return -3;

    ocr[0] = SPI_Transfer(0xFF);
    ocr[1] = SPI_Transfer(0xFF);
    ocr[2] = SPI_Transfer(0xFF);
    ocr[3] = SPI_Transfer(0xFF);

    SD_Deselect();
    SPI_Transfer(0xFF);

    /* Check CCS bit (bit 30) */
    if (ocr[0] & 0x40)
        sd_is_sdhc = 1;
    else
        sd_is_sdhc = 0;

    return 0;
}

int SD_ReadBlock(uint32_t sector, uint8_t *buffer)
{
    uint8_t token;

    if (SD_SendCommand(CMD17, sector, 0x01) != 0)
        return -1;

    while ((token = SPI_Transfer(0xFF)) == 0xFF);
    if (token != 0xFE) return -2;

    for (int i = 0; i < 512; i++)
        buffer[i] = SPI_Transfer(0xFF);

    SPI_Transfer(0xFF);
    SPI_Transfer(0xFF);

    SD_Deselect();
    SPI_Transfer(0xFF);
    return 0;
}

int SD_WriteBlock(uint32_t sector, const uint8_t *buffer)
{
    uint8_t r;
    uint16_t i;
    uint32_t addr = sector;

    if (!sd_is_sdhc)
        addr <<= 9;   // SDSC needs byte addressing

    /* CMD24: Write single block */
    r = SD_SendCommand(CMD24, addr, 0xFF);
    if (r != 0x00)
    {
        SD_Deselect();
        return -1;
    }

    /* Data token */
    SPI_Transfer(0xFE);

    /* Send data */
    for (i = 0; i < 512; i++)
        SPI_Transfer(buffer[i]);

    /* Dummy CRC */
    SPI_Transfer(0xFF);
    SPI_Transfer(0xFF);

    /* Data response */
    r = SPI_Transfer(0xFF);
    if ((r & 0x1F) != 0x05)
    {
        SD_Deselect();
        return -2;
    }

    /* WAIT UNTIL CARD IS READY */
    while (SPI_Transfer(0xFF) == 0x00)
        ;   // <-- THIS is what was hanging

    SD_Deselect();
    SPI_Transfer(0xFF);

    return 0;
}

/* Return current time packed into a DWORD */
DWORD get_fattime(void)
{
    return  ((DWORD)(2026 - 1980) << 25) |  /* Year = 2026 */
            ((DWORD)2 << 21) |              /* Month = Feb */
            ((DWORD)12 << 16) |             /* Day */
            ((DWORD)14 << 11) |             /* Hour */
            ((DWORD)50 << 5) |              /* Min */
            ((DWORD)0 >> 1);                /* Sec / 2 */
}

MMC_disk_status()
{

}
