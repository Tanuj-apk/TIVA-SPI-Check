#include <stdint.h>
#include <stdbool.h>

/* ================= DRIVERLIB ================= */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

/* ================= FATFS (SD) ================= */
#include "sd_spi.h"
#include "ff.h"

/* ================= GLOBAL ================= */
uint32_t g_sysClock;

/* ================= FATFS OBJECTS ================= */
FATFS fs;
FIL file;
UINT bw;
FRESULT res;

/* ================= DELAY ================= */
void delay_ms(uint32_t ms)
{
    uint32_t cycles = g_sysClock / 3000;
    while(ms--) SysCtlDelay(cycles);
}
/////////////////////////////////////////////////////
//////////////////// CH376 USB //////////////////////
/////////////////////////////////////////////////////
#define CMD1 0x57
#define CMD2 0xAB
#define CHECK_EXIST   0x06
#define SET_USB_MODE  0x15
#define DISK_CONNECT  0x30
#define DISK_MOUNT    0x31
#define SET_FILE_NAME 0x2F
#define FILE_CREATE   0x34
#define BYTE_WRITE    0x3C
#define WRITE_DATA    0x2D
#define UPDATE_SIZE   0x3D
#define FILE_CLOSE    0x36
#define RESET_CMD   0x05


void CH376_Send(uint8_t b)
{
    UARTCharPut(UART0_BASE, b);
}
uint8_t CH376_Read(void)
{
    while(!UARTCharsAvail(UART0_BASE));
    return UARTCharGet(UART0_BASE);
}

bool CH376_Wait(uint32_t timeout)
{
    while(timeout--)
    {
        if(UARTCharsAvail(UART0_BASE)) return true;
        SysCtlDelay(100);
    }
    return false;
}

/* ===================== DRIVER ===================== */

void CH376_Reset(void)
{
    /* Send: 57 AB 05 */
    UARTCharPut(UART0_BASE, CMD1);
    UARTCharPut(UART0_BASE, CMD2);
    UARTCharPut(UART0_BASE, RESET_CMD);

    /* CH376 reboot time */
    delay_ms(200);

    /* Flush any garbage bytes from UART */
    while(UARTCharsAvail(UART0_BASE))
        UARTCharGet(UART0_BASE);
}

bool CH376_CheckExist(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(CHECK_EXIST);
    CH376_Send(0x01);

    if(!CH376_Wait(50000)) return false;
    return (CH376_Read() == 0xFE);
}

bool CH376_SetUSBMode(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(SET_USB_MODE);
    CH376_Send(0x06);

    if(!CH376_Wait(50000)) return false;

    uint8_t r1 = CH376_Read();
    uint8_t r2 = CH376_Read();

    return (r1 == 0x51 && r2 == 0x15);
}

bool CH376_DiskConnect(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(DISK_CONNECT);

    if(!CH376_Wait(50000)) return false;
    return (CH376_Read() == 0x14);
}

bool CH376_Mount(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(DISK_MOUNT);

    if(!CH376_Wait(80000)) return false;
    return (CH376_Read() == 0x14);
}

void CH376_SetFileName(const char *name)
{
    int i = 0;
    int j = 0;

    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(SET_FILE_NAME);

    CH376_Send(SET_FILE_NAME);   // required

    /* Send name part (max 8) */
    while(name[i] && name[i] != '.' && i < 8)
        CH376_Send(name[i++]);

    while(i < 8) CH376_Send(' ');   // pad

    /* Send extension */
    if(name[i] == '.') i++;
    for(j=0;j<3;j++)
    {
        if(name[i]) CH376_Send(name[i++]);
        else CH376_Send(' ');
    }

    CH376_Send(0x00);
    delay_ms(20);
}


bool CH376_CreateFile(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(FILE_CREATE);

    if(!CH376_Wait(800000)) return false;
    return (CH376_Read() == 0x14);
}

bool CH376_Write(char *data)
{
    uint16_t len = 0;
    while(data[len]) len++;

    delay_ms(50);

    /* Tell length */
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(BYTE_WRITE);
    CH376_Send(len & 0xFF);
    CH376_Send(0x00);

    if(!CH376_Wait(80000)) return false;
    if(CH376_Read() != 0x1E) return false;

    delay_ms(10);   // IMPORTANT

    /* Send data */
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(WRITE_DATA);

    uint16_t i;
    for(i=0;i<len;i++)
        CH376_Send(data[i]);

    if(!CH376_Wait(80000)) return false;
    CH376_Read();   // consume 0x19

    delay_ms(10);   // IMPORTANT

    /* Update file size */
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(UPDATE_SIZE);

    if(!CH376_Wait(80000)) return false;
    CH376_Read();

    return true;
}


void CH376_Close(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(FILE_CLOSE);
    CH376_Send(0x01);

    CH376_Wait(80000);
    CH376_Read();
}

void USB_Write(void)
{
    CH376_Reset();
    if(!CH376_CheckExist()) while(1);
    if(!CH376_SetUSBMode()) while(1);
    if(!CH376_DiskConnect()) while(1);
    if(!CH376_Mount()) while(1);

//    CH376_Reset();
    CH376_SetFileName("BBBB    TXT");
    if(!CH376_CreateFile()) while(1);

    CH376_Write("HELLO FROM USB\r\n");
    CH376_Close();
}
/////////////////////////////////////////////////////
//////////////////// SD CARD ////////////////////////
/////////////////////////////////////////////////////
void SD_WRITE()
{
    if (SD_Init() != 0)
        while(1);

    res = f_mount(&fs, "", 1);
    if (res != FR_OK) while (1);

    res = f_open(&file, "JOB.TXT", FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) while (1);

    res = f_lseek(&file, f_size(&file));
    if (res != FR_OK) while (1);

    char msg[] = "HELLO FROM TI\r\n";

    res = f_write(&file, msg, sizeof(msg)-1, &bw);
    if (res != FR_OK || bw == 0) while (1);

    f_sync(&file);
    f_close(&file);

    SysCtlDelay(SysCtlClockGet() / 3);
}

/////////////////////////////////////////////////////
//////////////////// MAIN ///////////////////////////
/////////////////////////////////////////////////////

int main(void)
{
    g_sysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN |
            SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_480),
                       120000000);

    SPI_Init();


    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, g_sysClock, 9600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    delay_ms(500);

    SD_WRITE();

    delay_ms(500);

    USB_Write();

    while(1);
}
