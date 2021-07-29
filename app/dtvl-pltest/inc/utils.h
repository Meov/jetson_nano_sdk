#ifndef UTILS_H
#define UTILS_H

enum {
    SPI0,
    SPI1,
    SPI2,
    I2C0,
    I2C1,
    I2C2,
    I2C3,
    COM_I2C,
    MUX_PIN,
    GPIO,
    UART0,
    UART1,
    UART2,
    COM_UART,
    AP_PWR,
    AP_CTL,
    SDMMC2,
    DDR_PWR,
    MAX = 256,
};

void *map_module(int module);
void unmap_module(int module);
#endif
