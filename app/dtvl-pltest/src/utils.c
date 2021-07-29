#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "utils.h"

void * ap_modules_phy_base[MAX] = {
    (void *)0xa0880000,	/*SSI0*/
    (void *)0xa0881000,	/*SSI1*/
    (void *)0xa08a2000,	/*SSI2*/
    (void *)0xa0895000,	/*I2C0*/
    (void *)0xa0896000,	/*I2C1*/
    (void *)0xa08a0000,	/*I2C2*/
    (void *)0xa0897000,	/*I2C3*/
    (void *)0xE1006000,	/*COM_I2C*/
    (void *)0xe1009000,	/*MUX_PIN*/
    (void *)0xe100c000,	/*GPIO*/
    (void *)0xa0882000,	/*UART0*/
    (void *)0xa0883000,	/*UART1*/
    (void *)0xa0884000,	/*UART2*/
    (void *)0xe100b000,	/*COM_UART*/
    (void *)0xa0899000,	/*AP_PWR*/
    (void *)0xA0110000,	/*CTL(AHB) */
    (void *)0xA0861000,	/*SDMMC2*/
    (void *)0xe100a000,	/*DDR_PWR*/
};
void * ap_modules_virt_base[MAX] = {NULL};
unsigned int ap_modules_length[MAX] = {
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
    0x1000,
};
static int map_fd = -1;
void *map_module(int module)
{
    void *map = NULL;

    if (module >= MAX || !ap_modules_phy_base[module])
        return (void *)-EINVAL;

    if (map_fd < 0) {
        map_fd = open("/dev/dtvl_cdev", O_RDWR);
    }

    if (map_fd < 0)
        return (void *)-EIO;

    if (ap_modules_virt_base[module]) {
        map = ap_modules_virt_base[module];
    } else {
        map = mmap(NULL, ap_modules_length[module], PROT_READ | PROT_WRITE, MAP_SHARED, map_fd, (off_t)ap_modules_phy_base[module]);
        if (map == MAP_FAILED) {
            return (void *)-ENOMEM;
        }
        ap_modules_virt_base[module] = map;
    }

    return map;
}

void unmap_module(int module)
{
    if (module >= MAX)
        return;
    if (ap_modules_virt_base[module]) {
        munmap(ap_modules_virt_base[module], ap_modules_length[module]);
        ap_modules_virt_base[module] = NULL;
    }
    return;
}
