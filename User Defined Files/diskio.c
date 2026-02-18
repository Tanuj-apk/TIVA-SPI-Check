#include "diskio.h"
#include "sd_spi.h"
#include "ff.h"

DSTATUS disk_initialize(BYTE drv)
{
    if (SD_Init() == 0)
        return 0;
    return STA_NOINIT;
}

DSTATUS disk_status(BYTE drv)
{
    return 0;
}

DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t sector, UINT count)
{
    for (UINT i = 0; i < count; i++)
    {
        if (SD_ReadBlock(sector + i, buff + (512 * i)) != 0)
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE *buff, LBA_t sector, UINT count)
{
    for (UINT i = 0; i < count; i++)
    {
        if (SD_WriteBlock(sector + i, buff + (512 * i)) != 0)
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void *buff)
{
    return RES_OK;
}
