#include <stdint.h>
#include <stdbool.h>

/* ================= DRIVERLIB ================= */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "inc/hw_can.h"    /* provides CAN_INT_INTID_STATUS etc. */
#include "inc/hw_ints.h"

/* ================= FATFS (SD) ================= */
#include "sd_spi.h"
#include "ff.h"

/* ================= GLOBAL ================= */
uint32_t g_sysClock;

#define EEPROM_ADDR 0x50

uint8_t eeprom_page[64];

volatile bool log_eeprom_write_flag = false;
/* Store raw CAN payload to write into EEPROM */
uint8_t log_frame_copy[8];
uint16_t eeprom_log_addr = 0x0100;   // start address for log storage

bool finish_flag = 0;
bool SD_BUTTON = 0;
bool USB_BUTTON = 0;
#define BTN_PORT   GPIO_PORTD_BASE
#define BTN_SD     GPIO_PIN_0   // PD0 → EEPROM → SD
#define BTN_USB    GPIO_PIN_1   // PD1 → EEPROM → USB


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

/* ---------------- LOG MESSAGE DEFINITIONS ---------------- */

#define LOG_MESSAGE_CAN_ID     0x0210
#define LOG_MSG_TYPE1          0x02
#define LOG_MSG_TYPE2          0x10

static uint8_t sRXBufLog[8];
static tCANMsgObject sRXMsgObjLog;

volatile uint16_t log_data1 = 0;
volatile uint16_t log_data2 = 0;
volatile uint16_t log_data4 = 0;
volatile uint16_t log_data5 = 0;
volatile bool log_update_flag = false;

void Buttons_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinTypeGPIOInput(BTN_PORT, BTN_SD | BTN_USB);

    GPIOPadConfigSet(BTN_PORT,
                     BTN_SD | BTN_USB,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);   // internal pull-up
}

/* ---------------- CAN1 INTERRUPT HANDLER ---------------- */

void CAN1IntHandler(void)
{
    uint32_t cause = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

    if (cause == CAN_INT_INTID_STATUS)
    {
        /* Clear controller status interrupt */
        (void)CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
    }
    else if (cause == 1)   /* Message Object 1 = LOG RX */
    {
        CANMessageGet(CAN1_BASE, 1, &sRXMsgObjLog, true);

        if (sRXBufLog[0] == LOG_MSG_TYPE1 &&
            sRXBufLog[1] == LOG_MSG_TYPE2)
        {
            /* Copy raw frame for EEPROM logging */
            for (int i = 0; i < 8; i++)
                log_frame_copy[i] = sRXBufLog[i];

            /* Build 48-bit packed payload */
            uint64_t packed = 0;

            packed |= ((uint64_t)sRXBufLog[2] << 40);
            packed |= ((uint64_t)sRXBufLog[3] << 32);
            packed |= ((uint64_t)sRXBufLog[4] << 24);
            packed |= ((uint64_t)sRXBufLog[5] << 16);
            packed |= ((uint64_t)sRXBufLog[6] << 8);
            packed |= ((uint64_t)sRXBufLog[7]);

            /* Extract fields */
            log_data1 = (packed >> 34) & 0x3FFF;   /* 14 bits */
            log_data2 = (packed >> 24) & 0x03FF;   /* 10 bits */
            log_data4 = (packed >> 13) & 0x07FF;   /* 11 bits */
            log_data5 = (packed)       & 0x1FFF;   /* 13 bits */

            log_update_flag = true;
            log_eeprom_write_flag = true;
        }

        CANIntClear(CAN1_BASE, 1);
    }
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
#define FILE_OPEN     0x32
#define BYTE_LOCATE   0x39


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

bool CH376_OpenFile(void)
{
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(FILE_OPEN);

    if(!CH376_Wait(80000)) return false;
    return (CH376_Read() == 0x14);
}

bool CH376_AppendMode(void)
{
    /* Move file pointer to end */
    CH376_Send(CMD1);
    CH376_Send(CMD2);
    CH376_Send(BYTE_LOCATE);

    /* 0xFFFFFFFF → means END OF FILE */
    CH376_Send(0xFF);
    CH376_Send(0xFF);
    CH376_Send(0xFF);
    CH376_Send(0xFF);

    if(!CH376_Wait(80000)) return false;
    return (CH376_Read() == 0x14);
}

void USB_Write(void)
{
    CH376_Reset();
    if(!CH376_CheckExist()) while(1);
    if(!CH376_SetUSBMode()) while(1);
    if(!CH376_DiskConnect()) while(1);
    if(!CH376_Mount()) while(1);

//    CH376_Reset();
    CH376_SetFileName("EEEE    TXT");

    if(!CH376_OpenFile()){
        if(!CH376_CreateFile()) while(1);
    }

    if(!CH376_AppendMode()) while(1);

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

    res = f_open(&file, "VEX.TXT", FA_OPEN_ALWAYS | FA_WRITE);
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
//////////////////// EEPROM /////////////////////////
/////////////////////////////////////////////////////
void I2C0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false); // 100kHz
}
void EEPROM_WriteByte(uint16_t memAddr, uint8_t data)
{
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, EEPROM_ADDR, false);

    // Send High Address
    I2CMasterDataPut(I2C0_BASE, (memAddr >> 8) & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    // Send Low Address
    I2CMasterDataPut(I2C0_BASE, memAddr & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));

    // Send Data
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
}

uint8_t EEPROM_ReadByte(uint16_t memAddr)
{
    uint8_t data;

    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, EEPROM_ADDR, false);

    // Send High Address
    I2CMasterDataPut(I2C0_BASE, (memAddr >> 8) & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    // Send Low Address
    I2CMasterDataPut(I2C0_BASE, memAddr & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));

    // Switch to Read Mode
    I2CMasterSlaveAddrSet(I2C0_BASE, EEPROM_ADDR, true);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE));

    data = I2CMasterDataGet(I2C0_BASE);

    return data;
}

void EEPROM_WritePage(uint16_t memAddr, uint8_t *data, uint8_t len)
{
    uint8_t i;

    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, EEPROM_ADDR, false);

    I2CMasterDataPut(I2C0_BASE, (memAddr >> 8) & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, memAddr & 0xFF);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));

    for(i = 0; i < len; i++)
    {
        I2CMasterDataPut(I2C0_BASE, data[i]);

        if(i == (len - 1))
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        else
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

        while(I2CMasterBusy(I2C0_BASE));
    }
}
void EEPROM_ReadPage(uint16_t pageStartAddr, uint8_t *buffer, uint8_t len)
{
    uint8_t i;

    for(i = 0; i < len; i++)
    {
        buffer[i] = EEPROM_ReadByte(pageStartAddr + i);
    }
}
void EEPROM_Write_Data(){
// -------- Byte Write --------
    EEPROM_WriteByte(0x0000, 0x54); delay_ms(6);
    EEPROM_WriteByte(0x0001, 0x41); delay_ms(6);
    EEPROM_WriteByte(0x0002, 0x4E); delay_ms(6);
    EEPROM_WriteByte(0x0003, 0x55); delay_ms(6);
    EEPROM_WriteByte(0x0004, 0x4A); delay_ms(6);
    EEPROM_WriteByte(0x0005, 0x20); delay_ms(6);
    EEPROM_WriteByte(0x0006, 0x53); delay_ms(6);
    EEPROM_WriteByte(0x0007, 0x4F); delay_ms(6);
    EEPROM_WriteByte(0x0008, 0x4E); delay_ms(6);
    EEPROM_WriteByte(0x0009, 0x49); delay_ms(6);
    EEPROM_WriteByte(0x000A, 0x00); delay_ms(6);
    delay_ms(100);
}
void EEPROM_Read_Data(){
    // -------- Byte Read --------
    EEPROM_ReadByte(0x0000); delay_ms(6);
    EEPROM_ReadByte(0x0001); delay_ms(6);
    EEPROM_ReadByte(0x0002); delay_ms(6);
    EEPROM_ReadByte(0x0003); delay_ms(6);
    EEPROM_ReadByte(0x0004); delay_ms(6);
    EEPROM_ReadByte(0x0005); delay_ms(6);
    EEPROM_ReadByte(0x0006); delay_ms(6);
    EEPROM_ReadByte(0x0007); delay_ms(6);
    EEPROM_ReadByte(0x0008); delay_ms(6);
    EEPROM_ReadByte(0x0009); delay_ms(6);
    EEPROM_ReadByte(0x000A); delay_ms(6);
}

static uint8_t hex_char(uint8_t v)
{
    return (v < 10) ? ('0' + v) : ('A' + v - 10);
}
static int u16_to_dec(char *out, uint16_t val)
{
    char buf[6];   // max 65535 → 5 digits
    int i = 0, j = 0;

    if (val == 0)
    {
        out[0] = '0';
        return 1;
    }

    while (val > 0)
    {
        buf[i++] = (val % 10) + '0';
        val /= 10;
    }

    while (i > 0)
        out[j++] = buf[--i];

    return j;
}
static void decode_log_frame(uint8_t *b,
                             uint16_t *d1,
                             uint16_t *d2,
                             uint16_t *d4,
                             uint16_t *d5)
{
    uint64_t packed = 0;

    packed |= ((uint64_t)b[2] << 40);
    packed |= ((uint64_t)b[3] << 32);
    packed |= ((uint64_t)b[4] << 24);
    packed |= ((uint64_t)b[5] << 16);
    packed |= ((uint64_t)b[6] << 8);
    packed |= ((uint64_t)b[7]);

    *d1 = (packed >> 34) & 0x3FFF;
    *d2 = (packed >> 24) & 0x03FF;
    *d4 = (packed >> 13) & 0x07FF;
    *d5 = (packed)       & 0x1FFF;
}
void EEPROM_to_SD(void)
{
    uint16_t total_bytes = (eeprom_log_addr - 0x0100) & ~0x07;
    if (total_bytes == 0) return;

    if (SD_Init() != 0) return;
    if (f_mount(&fs, "", 1) != FR_OK) return;
    if (f_open(&file, "EEPROM.TXT", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) return;

    f_lseek(&file, f_size(&file));

    uint16_t addr = 0x0100;

    while (addr < eeprom_log_addr)
    {
        EEPROM_ReadPage(addr, eeprom_page, 8);   // read one CAN frame

        uint16_t d1, d2, d4, d5;
        decode_log_frame(eeprom_page, &d1, &d2, &d4, &d5);

        char line[48];
        int idx = 0;

        /* D1= */
        line[idx++] = 'D';
        line[idx++] = '1';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d1);

        /* space */
        line[idx++] = ' ';

        /* D2= */
        line[idx++] = 'D';
        line[idx++] = '2';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d2);

        /* space */
        line[idx++] = ' ';

        /* D4= */
        line[idx++] = 'D';
        line[idx++] = '4';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d4);

        /* space */
        line[idx++] = ' ';

        /* D5= */
        line[idx++] = 'D';
        line[idx++] = '5';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d5);

        line[idx++] = '\r';
        line[idx++] = '\n';

        f_write(&file, line, idx, &bw);

        addr += 8;
    }

    f_sync(&file);
    f_close(&file);
}

void EEPROM_to_USB(void)
{
    uint16_t total_bytes = (eeprom_log_addr - 0x0100) & ~0x07;
    if (total_bytes == 0) return;

    CH376_Reset();
    if(!CH376_CheckExist()) return;
    if(!CH376_SetUSBMode()) return;
    if(!CH376_DiskConnect()) return;
    if(!CH376_Mount()) return;

    CH376_SetFileName("EEPROM  TXT");

    if(!CH376_OpenFile())
        if(!CH376_CreateFile()) return;

    if(!CH376_AppendMode()) return;

    uint16_t addr = 0x0100;

    while (addr < eeprom_log_addr)
    {
        EEPROM_ReadPage(addr, eeprom_page, 8);

        uint16_t d1, d2, d4, d5;
        decode_log_frame(eeprom_page, &d1, &d2, &d4, &d5);

        char line[48];
        int idx = 0;

        line[idx++] = 'D';
        line[idx++] = '1';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d1);

        line[idx++] = ' ';
        line[idx++] = 'D';
        line[idx++] = '2';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d2);

        line[idx++] = ' ';
        line[idx++] = 'D';
        line[idx++] = '4';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d4);

        line[idx++] = ' ';
        line[idx++] = 'D';
        line[idx++] = '5';
        line[idx++] = '=';
        idx += u16_to_dec(&line[idx], d5);

        line[idx++] = '\r';
        line[idx++] = '\n';

        line[idx] = '\0';

        CH376_Write(line);

        addr += 8;
    }

    CH376_Close();
}

/////////////////////////////////////////////////////
//////////////////// MAIN ///////////////////////////
/////////////////////////////////////////////////////

int main(void)
{
    finish_flag = 0;
    g_sysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN |
            SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_480),
                       120000000);
    Buttons_Init();
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

    /* Enable GPIOB and CAN1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1));

    /* PB0 = CAN1RX, PB1 = CAN1TX */
    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Init CAN1 @ 500 kbps */
    CANInit(CAN1_BASE);
    CANBitRateSet(CAN1_BASE, g_sysClock, 500000);

    /* Configure RX message object */
    sRXMsgObjLog.ui32MsgID       = LOG_MESSAGE_CAN_ID;
    sRXMsgObjLog.ui32MsgIDMask   = 0;
    sRXMsgObjLog.ui32Flags       = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_EXTENDED_ID;
    sRXMsgObjLog.ui32MsgLen      = 8;
    sRXMsgObjLog.pui8MsgData     = sRXBufLog;

    CANMessageSet(CAN1_BASE, 1, &sRXMsgObjLog, MSG_OBJ_TYPE_RX);

    /* Enable CAN1 Interrupt */
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN1);

    CANEnable(CAN1_BASE);


    I2C0_Init();

    EEPROM_Write_Data();

    EEPROM_Read_Data();

    delay_ms(500);

//    SD_WRITE();
//
//    delay_ms(500);
//
//    USB_Write();


//    while(1);
    while(1)
    {
        uint32_t val = GPIOPinRead(BTN_PORT, BTN_SD | BTN_USB);

        bool sdPressed  = ((val & BTN_SD)  == 0);   // active low
        bool usbPressed = ((val & BTN_USB) == 0);

        if(sdPressed)
        {
            delay_ms(40);
            if((GPIOPinRead(BTN_PORT, BTN_SD) & BTN_SD) == 0)
            {
                EEPROM_to_SD();
                while((GPIOPinRead(BTN_PORT, BTN_SD) & BTN_SD) == 0);
            }
        }

        if(usbPressed)
        {
            delay_ms(40);
            if((GPIOPinRead(BTN_PORT, BTN_USB) & BTN_USB) == 0)
            {
                EEPROM_to_USB();
                while((GPIOPinRead(BTN_PORT, BTN_USB) & BTN_USB) == 0);
            }
        }
        if (log_eeprom_write_flag)
        {
            log_eeprom_write_flag = false;

            /* Write 8-byte CAN frame to EEPROM */

            EEPROM_WritePage(eeprom_log_addr, log_frame_copy, 8);

            /* Wait EEPROM write cycle (5–6 ms typical) */
            delay_ms(6);

            /* Move to next log slot */
            eeprom_log_addr += 8;

            /* Optional: prevent overflow */
            if (eeprom_log_addr >= 0x7FFF)
                eeprom_log_addr = 0x0100;
        }
    }

}
