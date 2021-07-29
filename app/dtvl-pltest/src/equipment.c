/*********************************************************************** 
*Copyright(c)2008,by Gohigh Data Networks Technology Co.,LTD 
*All rights reserved.
*
*Appiled body:
*Project name:	
*Project function:
*Header file:
*File list:
*Author:vehicle to everything
*Date:
*Version:			v0.0.1
*Version history:	None
*说明：
*
**********************************************************************/
/******************************* 包含文件声明 *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <pthread.h>
#include "main.h"
#include "equipment.h"
#include "gpio_operate.h"
#include "XdjaKeyApi.h"

/******************************* 局部常数和类型定义 **************************/

/******************************* 宏和常量定义 *********************************/
#define MMC_DEV_NAME        "csd_revision"
#define UNKNOWN                   0xFF

#define UART0_DEV_FILE        "/dev/ttyS0"
#define GPS_DEV_FILE            "/dev/ttyS1"
#define GPS_DEV_FILE_3205     "/dev/ttyS0"
#define UART2_DEV_FILE        "/dev/ttyS2"
#define COMUART_DEV_FILE   "/dev/ttyS3"
#define CONSOLE_DEV_FILE    "/dev/console"
#define SELECT_TIMEOUT_500MS    (500*1000)
#define GPS_DATA_BUF_SIZE   2048
#define NMEA_HEAD_LEN           3
#define GPS_RST                         181
#define BUFF_LEN                        1024

#define WIFI_BCM4343S_PWDN        217           /* GPIO217为WIFI REG EN*/
#define BT_BCM4343S_PWDN            102           /* GPIO102为BT REG EN*/
#define BT_BCM4343S_CHIPEN          199
/* ZELAI_TEST: made by zelai.wang,  CTAO_TEST: made by ctao*/
#define ZELAI_TEST 0
#define CTAO_TEST 1
#define LSUSB_INFO_SIZE 5

#define max(a, b)  (((a) > (b)) ? (a) : (b)) 

/******************************* 全局变量定义/初始化 *************************/
static int g_ttyS0_fd = -1;         //ttyS0 fd
static int g_ttyS0_fd_3205 = -1;         //ttyS0 fd
static int g_ttyS1_fd = -1;         //ttyS1 fd
static int g_ttyS2_fd = -1;         //ttyS2 fd
static int g_ttyS3_fd = -1;         //ttyS3 fd
static int g_console_fd = -1;      //console

/* IO板卡硬件版本号*/
int hwver = 0xff;

/* WIFI init flag*/
int wifi_flag = -1;

/* uart2 bt*/
char bcm4343S_baud[4] = {0x1, 0x9, 0x10, 0x0};                      /* 发送数据，同步波特率*/
char bcm4343S_baudResp[12] = {0x4, 0xe, 0xa, 0x1, 0x9, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};     /* 接收数据比较确定UART是否同步*/
char bcm4343S_HCIrst[4] = {0x1, 0x3, 0xc, 0x0};
char bcm4343S_HCIrstResp[7] = {0x4, 0xe, 0x4, 0x1, 0x3, 0xc, 0x0};
unsigned char sn_eq[33] = {0};
unsigned char sn_io[33] = {0};

unsigned int rac=0;
unsigned int mmcSdioRelAddress = MMCI_RCA;

typedef enum MmcDrvStatusTag
{
    MMC_DRV_IDLE,
    MMC_DRV_CMD_IN_PROGRESS, /* Not used at present */
    MMC_DRV_DMA_IN_PROGRESS
}SdioMmcDrvStatus;

volatile SdioMmcDrvStatus sdioDrvStatus = MMC_DRV_IDLE;

struct sdio_cccr {
    unsigned int sdio_vsn;
    unsigned int sd_vsn;
    unsigned int multi_block:1,
                       low_speed:  1,
                       wide_bus:   1,
                       high_power: 1,
                       high_speed: 1;
};

static struct sdio_cccr cccr_record={
    0,
    0,
    0,
};

/* I2C参数*/
unsigned long i2cx_parent_rate = 0;
unsigned long osc_clk_rate = 26000000;

volatile unsigned  * I2Cx_CON; 
volatile unsigned  * I2Cx_TAR; 
volatile unsigned  * I2Cx_HS_MADDR; 
volatile unsigned  * I2Cx_DATA_CMD; 
volatile unsigned  * I2Cx_SS_SCL_HCNT; 
volatile unsigned  * I2Cx_SS_SCL_LCNT; 
volatile unsigned  * I2Cx_FS_SCL_HCNT; 
volatile unsigned  * I2Cx_FS_SCL_LCNT; 
volatile unsigned  * I2Cx_HS_SCL_HCNT; 
volatile unsigned  * I2Cx_HS_SCL_LCNT; 
volatile unsigned  * I2Cx_INTR_STAT; 
volatile unsigned  * I2Cx_INTR_EN; 
volatile unsigned  * I2Cx_RAW_INTR_STAT; 
volatile unsigned  * I2Cx_RX_TL; 
volatile unsigned  * I2Cx_TX_TL; 
volatile unsigned  * I2Cx_CLR_INTR; 
volatile unsigned  * I2Cx_CLR_RX_UNDER; 
volatile unsigned  * I2Cx_CLR_RX_OVER; 
volatile unsigned  * I2Cx_CLR_TX_OVER; 
volatile unsigned  * I2Cx_CLR_TX_ABRT; 
volatile unsigned  * I2Cx_CLR_ACTIVITY; 
volatile unsigned  * I2Cx_CLR_STOP_DET; 
volatile unsigned  * I2Cx_CLR_START_DET; 
volatile unsigned  * I2Cx_CLR_GEN_CALL; 
volatile unsigned  * I2Cx_ENABLE; 
volatile unsigned  * I2Cx_STATUS; 
volatile unsigned  * I2Cx_TXFLR; 
volatile unsigned  * I2Cx_RXFLR; 
volatile unsigned  * I2Cx_SDA_HOLD; 
volatile unsigned  * I2Cx_TX_ABRT_SOURCE;

/* SPI参数*/
volatile unsigned  * SSIx_CTRL0; 
volatile unsigned  * SSIx_CTRL1; 
volatile unsigned  * SSIx_EN; 
volatile unsigned  * SSIx_SE; 
volatile unsigned  * SSIx_BAUD; 
volatile unsigned  * SSIx_TXFTL; 
volatile unsigned  * SSIx_RXFTL; 
volatile unsigned  * SSIx_TXFL; 
volatile unsigned  * SSIx_RXFL; 
volatile unsigned  * SSIx_STS; 
volatile unsigned  * SSIx_IE; 
volatile unsigned  * SSIx_IS; 
volatile unsigned  * SSIx_RIS; 
volatile unsigned  * SSIx_TXOIC; 
volatile unsigned  * SSIx_RXOIC; 
volatile unsigned  * SSIx_RXUIC; 
volatile unsigned  * SSIx_IC; 
volatile unsigned  * SSIx_DMAC; 
volatile unsigned  * SSIx_DMATDL; 
volatile unsigned  * SSIx_DMARDL; 
volatile unsigned  * SSIx_DATA; 
volatile unsigned  * CTL_SSIx_PROTOCOL_CTRL; 

extern void *SSI0_MODEULE_BASE;
extern void *SSI1_MODEULE_BASE;
extern void *SSI2_MODEULE_BASE;
extern void *I2C0_MODEULE_BASE;
extern void *I2C1_MODEULE_BASE;
extern void *I2C2_MODEULE_BASE;
extern void *I2C3_MODEULE_BASE;
extern void *COM_I2C_MODEULE_BASE;
extern void *MUX_PIN_MODEULE_BASE;
extern void *GPIO_MODEULE_BASE;
extern void *UART0_MODEULE_BASE;
extern void *UART1_MODEULE_BASE;
extern void *UART2_MODEULE_BASE;
extern void *COM_UART_MODEULE_BASE;
extern void *AP_PWR_MODEULE_BASE;
extern void *AP_CTL_MODEULE_BASE;
extern void *SDMMC2_MODEULE_BASE;
extern void *DDR_PWR_MODEULE_BASE;

/* sensor变量定义*/
TEST_ITEM SensorList[]=
{   
    //1.menu name         2.function name

#ifdef MMC34160PJ
    {" MMC34160PJ test    ", sensor_mmc34160pj_test},
#endif

#ifdef L3GD20
    {" L3GD20 test        ", sensor_l3gd20_test},
#endif

#ifdef MMA8653FCR1
    {" MMA8653FCR1 test   ", sensor_mma8653fcr1_test},
#endif

    {NULL,                   NULL}
};

/* 系统操作*/
TEST_ITEM OptList[]=
{   
    //1.menu name         2.function name
    {" Power Off       ", power_off },
    {" Reset           ", Rst_On },
    {NULL,NULL}
};

/****************************functions*************************************/
/************************************************************************
* 函数名: gps_alps_test 
* 功能描述: 测试ALPS GPS芯片输出正确性
* 输入参数: 
* 输出参数: 无
* 返回值: 无 
* 备注: GPIO181为GPS的RST管脚，低电平有效。信号输出波特率230400
*
*************************************************************************/
void gps_alps_test(void)
{
    int ret = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int Count = 0;
    int MaxFd = 0;
    fd_set readSet;
    struct timeval struTimeVal;
    char GpsBuf[GPS_DATA_BUF_SIZE] = {0};/* GPS数据缓冲区 */

    HARDWARE_TEST("Begin GPS_ALPS_TEST Test!\n\r");

    /* set gpio uart1 function*/
    PIN_Mux_set(UART1_TX, 0);     /* uart1_tx 端口连接到外部管脚 U0TXD(gpio179)*/
    PIN_Mux_set(UART1_RX, 0);     /* uart1_rx 端口连接到外部管脚 U0RXD(gpio180)*/

    /* 设置RST管脚GPIO181为OUTPUT模式，高电平状态*/
    PIN_Mux_set(GPS_RST, 2);
    gpio_direction(GPS_RST, GPIO_DIR_OUTPUT);
    gpio_output_set(GPS_RST, 1);

    g_ttyS1_fd = open(GPS_DEV_FILE, O_RDWR | O_NOCTTY);
    if(g_ttyS1_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", GPS_DEV_FILE);
        goto open_err;
    }

    HARDWARE_TEST("Open %s success.\n", GPS_DEV_FILE);

    tcflush(g_ttyS1_fd, TCIFLUSH);

    ret = tcgetattr(g_ttyS1_fd, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", GPS_DEV_FILE);
        goto tcgetattr_err;
    }

    /* ALPS GPS芯片串口波特率为230400*/
    cfsetispeed(&old_tty_opt, B230400);	//设置输入波特率230400
    cfsetospeed(&old_tty_opt, B230400);	//设置输出波特率230400

    //cfsetispeed(&old_tty_opt, B115200);	//设置输入波特率115200
    //cfsetospeed(&old_tty_opt, B115200);	//设置输出波特率115200

    //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
    //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600

    new_tty_opt = old_tty_opt;

    /* 设置为不需要回车或者换行就能发送出去*/
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &new_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* 接收ALPS GPS发送过来的信号*/
    while(1)
    {
        FD_ZERO(&readSet);
        FD_SET(g_ttyS1_fd, &readSet);

        MaxFd = g_ttyS1_fd + 1;

        struTimeVal.tv_sec = 0;
        struTimeVal.tv_usec = SELECT_TIMEOUT_500MS;

        ret = select(MaxFd, &readSet, NULL, NULL, &struTimeVal);
		
        if(ret < 0)
        {
            HARDWARE_TEST("select tty failed.(err: %d).\n", ret);
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select tty timeout.\n");
            continue;
        }

        if(FD_ISSET(g_ttyS1_fd, &readSet))
        {

            memset(GpsBuf, 0, GPS_DATA_BUF_SIZE);

            Count = read(g_ttyS1_fd, GpsBuf, (GPS_DATA_BUF_SIZE-1));
            if(Count < 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d).\n", Count);
                continue;
            }

            /* 将ttyS1读到的字符串保存到日志文件中*/
            //HW_LOG("Read ttyS1: %s.\n", GpsBuf);

            if(NULL != strstr(GpsBuf, "RMC"))
            {				
                HARDWARE_TEST("Read GPS NMEA char!\n");
                break;
            }
        }
    }

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;
    HARDWARE_TEST("GPS_ALPS_TEST Success!\n");

    return;

tcsetattr_2_err:

    tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);

tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;

open_err:
    HARDWARE_TEST("GPS_ALPS_TEST Failed!\n");

    return;
}

/************************************************************************
* 函数名: uart1_gps_test 
* 功能描述: 测试UART1 GPS芯片输出正确性
* 输入参数: 
* 输出参数: 0: false   1: success
* 返回值: 无 
* 备注: GPIO181为GPS的RST管脚，低电平有效。信号输出波特率115200
*
*************************************************************************/
int uart1_gps_test(void)
{
    int ret = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int Count = 0;
    int MaxFd = 0;
    fd_set readSet;
    struct timeval struTimeVal;
    char GpsBuf[GPS_DATA_BUF_SIZE] = {0};/* GPS数据缓冲区 */
    int cnt = 0;
    int allnum = 0;

    HARDWARE_TEST("Begin UART1 GPS TEST Test!\n\r");
    /* 执行"xds -q"，退出xds程序，以免XDS读走GPS数据*/
    system("xds -q");
    /* kill千寻相关的程序，以免千寻程序读走GPS数据*/
    system("killall qxwz");
    usleep(1000);

    memset(GpsBuf, 0, GPS_DATA_BUF_SIZE);

    if (0xff == hwver)
    {
        /* ADC连续多次不间断读取会出现读取不到数据的情况*/
        hwver = io_board_hardware_ver_test();
    }
    usleep(3000);

    /* set gpio uart1 function*/
    PIN_Mux_set(UART1_TX, 0);     /* uart1_tx 端口连接到外部管脚 U0TXD(gpio179)*/
    PIN_Mux_set(UART1_RX, 0);     /* uart1_rx 端口连接到外部管脚 U0RXD(gpio180)*/

    g_ttyS1_fd = open(GPS_DEV_FILE, O_RDWR | O_NOCTTY);
    if(g_ttyS1_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", GPS_DEV_FILE);
        goto open_err;
    }

    HARDWARE_TEST("Open %s success.\n", GPS_DEV_FILE);

    ret = tcflush(g_ttyS1_fd, TCIFLUSH);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcgetattr_err;
    }

    ret = tcgetattr(g_ttyS1_fd, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", GPS_DEV_FILE);
        goto tcgetattr_err;
    }

    /* 设置GPS芯片串口波特率*/
    if((UMCC1_EVB == hwver) || (SUZUNE_EVK_ES1 == hwver))
    {
        /* ALPS GPS波特率设置*/
        cfsetispeed(&old_tty_opt, B230400);	//设置输入波特率230400
        cfsetospeed(&old_tty_opt, B230400);	//设置输出波特率230400
    }
    else
    {
        cfsetispeed(&old_tty_opt, B115200);	//设置输入波特率115200
        cfsetospeed(&old_tty_opt, B115200);	//设置输出波特率115200
        /* 不运行XDS，GPS波特率为默认9600*/
        //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
        //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600
    }

    //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
    //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600

    new_tty_opt = old_tty_opt;

    /* 设置为不需要回车或者换行就能发送出去*/
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &new_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* 接收GPS输出的信号*/
    while(1)
    {
        FD_ZERO(&readSet);
        FD_SET(g_ttyS1_fd, &readSet);

        MaxFd = g_ttyS1_fd + 1;

        struTimeVal.tv_sec = 3;    /* 3s超时*/
        struTimeVal.tv_usec = 0;

        ret = select(MaxFd, &readSet, NULL, NULL, &struTimeVal);

        if(ret < 0)
        {
            HARDWARE_TEST("select tty failed.(err: %d).\n", ret);

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select tty timeout.\n");

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }

        if(FD_ISSET(g_ttyS1_fd, &readSet))
        {
            Count = read(g_ttyS1_fd, GpsBuf+allnum, (GPS_DATA_BUF_SIZE-1-allnum));
            if(Count < 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d)\n", Count);

                if (5 <= cnt)
                {
                    goto tcsetattr_1_err;
                }
                cnt++;
                continue;
            }

            if((allnum < GPS_DATA_BUF_SIZE) && (0 != GPS_DATA_BUF_SIZE-1 - allnum))
            {
                allnum += Count;
                continue;
            }

            /* 将ttyS1读到的字符串保存到日志文件中*/
            HARDWARE_TEST("Read ttyS1: %s.\n", GpsBuf);

            if((NULL != strstr(GpsBuf, "RMC")) || (NULL != strstr(GpsBuf, "GGA")) || (NULL != strstr(GpsBuf, "UBX")) || (NULL != strstr(GpsBuf, "VTG"))
                 || (NULL != strstr(GpsBuf, "GSA")) || (NULL != strstr(GpsBuf, "GSV")) || (NULL != strstr(GpsBuf, "GLL")) || (NULL != strstr(GpsBuf, "nothing")))
            {				
                HARDWARE_TEST("Read GPS string!\n");
                break;
            }
            allnum = 0;
        }

        /* 防止xds守护进程再次启动xds*/
        system("xds -q");
        usleep(100);
    }

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;
    /* 测试完成启动xds*/
    system("xds -d");
    /* 测试完成启动千寻相关的程序*/
    system("/system/bin/qxwz &");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Success!\n");

    return 1;

tcsetattr_2_err:

    tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);

tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;

open_err:
    /* 测试完成启动xds*/
    system("xds -d");
    /* 测试完成启动千寻相关的程序*/
    system("/system/bin/qxwz &");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Failed!\n");

    return 0;
}

/************************************************************************
* 函数名: uart0_gps_test4VU3205 
* 功能描述: 测试UART0 GPS芯片输出正确性
* 输入参数: 
* 输出参数: 0: false   1: success
* 返回值: 无 
* 备注: GPIO为GPS的RST管脚，低电平有效。信号输出波特率115200
*
*************************************************************************/
int uart0_gps_test4VU3205(void)
{
    int ret = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int Count = 0;
    int MaxFd = 0;
    fd_set readSet;
    struct timeval struTimeVal;
    char GpsBuf[GPS_DATA_BUF_SIZE] = {0};/* GPS数据缓冲区 */
    int cnt = 0;
    int allnum = 0;

    HARDWARE_TEST("Begin UART1 GPS TEST Test!\n\r");
    /* 执行"xds -q"，退出xds程序，以免XDS读走GPS数据*/
    system("xds -q");
    /* kill千寻相关的程序，以免千寻程序读走GPS数据*/
    system("killall qxwz");
    usleep(1000);

    memset(GpsBuf, 0, GPS_DATA_BUF_SIZE);

    if (0xff == hwver)
    {
        /* ADC连续多次不间断读取会出现读取不到数据的情况*/
        hwver = io_board_hardware_ver_test();
    }
    usleep(3000);

    /* set gpio uart0 function*/
    PIN_Mux_set(UART0_TX, 0);     /* uart0_tx 端口连接到外部管脚 U0TXD(gpio76)*/
    PIN_Mux_set(UART0_RX, 0);     /* uart0_rx 端口连接到外部管脚 U0RXD(gpio77)*/

    g_ttyS0_fd_3205 = open(GPS_DEV_FILE_3205, O_RDWR | O_NOCTTY);
    if(g_ttyS0_fd_3205 < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", GPS_DEV_FILE_3205);
        goto open_err;
    }

    HARDWARE_TEST("Open %s success.\n", GPS_DEV_FILE_3205);

    ret = tcflush(g_ttyS0_fd_3205, TCIFLUSH);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE_3205);
        goto tcgetattr_err;
    }

    ret = tcgetattr(g_ttyS0_fd_3205, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", GPS_DEV_FILE_3205);
        goto tcgetattr_err;
    }

    /* 设置GPS芯片串口波特率*/
    if((UMCC1_EVB == hwver) || (SUZUNE_EVK_ES1 == hwver))
    {
        /* ALPS GPS波特率设置*/
        cfsetispeed(&old_tty_opt, B230400);	//设置输入波特率230400
        cfsetospeed(&old_tty_opt, B230400);	//设置输出波特率230400
    }
    else
    {
        cfsetispeed(&old_tty_opt, B115200);	//设置输入波特率115200
        cfsetospeed(&old_tty_opt, B115200);	//设置输出波特率115200
        /* 不运行XDS，GPS波特率为默认9600*/
        //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
        //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600
    }

    //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
    //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600

    new_tty_opt = old_tty_opt;

    /* 设置为不需要回车或者换行就能发送出去*/
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;

    ret = tcsetattr(g_ttyS0_fd_3205, TCSANOW, &new_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE_3205);
        goto tcsetattr_1_err;
    }

    /* 接收GPS输出的信号*/
    while(1)
    {
        FD_ZERO(&readSet);
        FD_SET(g_ttyS0_fd_3205, &readSet);

        MaxFd = g_ttyS0_fd_3205 + 1;

        struTimeVal.tv_sec = 3;    /* 3s超时*/
        struTimeVal.tv_usec = 0;

        ret = select(MaxFd, &readSet, NULL, NULL, &struTimeVal);

        if(ret < 0)
        {
            HARDWARE_TEST("select tty failed.(err: %d).\n", ret);

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select tty timeout.\n");

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }

        if(FD_ISSET(g_ttyS0_fd_3205, &readSet))
        {
            Count = read(g_ttyS0_fd_3205, GpsBuf+allnum, (GPS_DATA_BUF_SIZE-1-allnum));
            if(Count < 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d)\n", Count);

                if (5 <= cnt)
                {
                    goto tcsetattr_1_err;
                }
                cnt++;
                continue;
            }

            if((allnum < GPS_DATA_BUF_SIZE) && (0 != GPS_DATA_BUF_SIZE-1 - allnum))
            {
                allnum += Count;
                continue;
            }

            /* 将ttyS1读到的字符串保存到日志文件中*/
            HARDWARE_TEST("Read ttyS0: %s.\n", GpsBuf);

            if((NULL != strstr(GpsBuf, "RMC")) || (NULL != strstr(GpsBuf, "GGA")) || (NULL != strstr(GpsBuf, "UBX")) || (NULL != strstr(GpsBuf, "VTG"))
                 || (NULL != strstr(GpsBuf, "GSA")) || (NULL != strstr(GpsBuf, "GSV")) || (NULL != strstr(GpsBuf, "GLL")) || (NULL != strstr(GpsBuf, "nothing")))
            {				
                HARDWARE_TEST("Read GPS string!\n");
                break;
            }
            allnum = 0;
        }

        /* 防止xds守护进程再次启动xds*/
        system("xds -q");
        usleep(100);
    }

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);

    ret = tcsetattr(g_ttyS0_fd_3205, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE_3205);
        goto tcsetattr_2_err;
    }

    close(g_ttyS0_fd_3205);
    g_ttyS0_fd_3205 = -1;
    /* 测试完成启动xds*/
    system("xds -d");
    /* 测试完成启动千寻相关的程序*/
    system("/system/bin/qxwz &");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Success!\n");

    return 1;

tcsetattr_2_err:

    tcsetattr(g_ttyS0_fd_3205, TCSANOW, &old_tty_opt);

tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS0_fd_3205);
    g_ttyS0_fd_3205 = -1;

open_err:
    /* 测试完成启动xds*/
    system("xds -d");
    /* 测试完成启动千寻相关的程序*/
    system("/system/bin/qxwz &");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Failed!\n");

    return 0;
}

/************************************************************************
* 函数名: gnss_cn0_test 
* 功能描述: 获取UART1上GPS的CN0值
* 输入参数: 待返回的cn0值
* 输出参数: 0: false   1: success
* 返回值: 无 
* 备注: CN0值为GPS NEMA协议$GPGSV的第7个字段值
*
*************************************************************************/
int gnss_cn0_test(int* cn0)
{
    int ret = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int Count = 0;
    int MaxFd = 0;
    fd_set readSet;
    struct timeval struTimeVal;
    char GpsBuf[GPS_DATA_BUF_SIZE] = {0};/* GPS数据缓冲区 */
    char exBuf[1024] = {0};
    char tmpBuf[100] = {0};
    int svcn0[20] = {0};
    int cnt = 0;
    char *Begb = NULL;
    char *Endb = NULL;
    int num = 0;
    int allnum = 0;
    int i = 0;

    HARDWARE_TEST("begin gnss C/N0 test!\n\r");
    /* 执行"xds -q"，退出xds程序，以免XDS读走GPS数据*/
    system("xds -q");
    usleep(1000);

    memset(GpsBuf, 0, GPS_DATA_BUF_SIZE);

    if (0xff == hwver)
    {
        /* ADC连续多次不间断读取会出现读取不到数据的情况*/
        hwver = io_board_hardware_ver_test();
    }
    usleep(3000);

    /* set gpio uart1 function*/
//    PIN_Mux_set(UART1_TX, 0);     /* uart1_tx 端口连接到外部管脚 U0TXD(gpio179)*/
//    PIN_Mux_set(UART1_RX, 0);     /* uart1_rx 端口连接到外部管脚 U0RXD(gpio180)*/

    g_ttyS1_fd = open(GPS_DEV_FILE, O_RDWR | O_NOCTTY);
    if(g_ttyS1_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", GPS_DEV_FILE);
        goto open_err;
    }

    HARDWARE_TEST("Open %s success.\n", GPS_DEV_FILE);

    tcflush(g_ttyS1_fd, TCIFLUSH);

    ret = tcgetattr(g_ttyS1_fd, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", GPS_DEV_FILE);
        goto tcgetattr_err;
    }

    /* 设置GPS芯片串口波特率*/
    if((UMCC1_EVB == hwver) || (SUZUNE_EVK_ES1 == hwver))
    {
        cfsetispeed(&old_tty_opt, B230400);	//设置输入波特率230400
        cfsetospeed(&old_tty_opt, B230400);	//设置输出波特率230400
    }
    else
    {
        cfsetispeed(&old_tty_opt, B115200);	//设置输入波特率115200
        cfsetospeed(&old_tty_opt, B115200);	//设置输出波特率115200
        /* 不运行XDS，GPS波特率为默认9600*/
        //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
        //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600
    }

    //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
    //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600

    new_tty_opt = old_tty_opt;

    /* 设置为不需要回车或者换行就能发送出去*/
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &new_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", GPS_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* 接收GPS输出的信号*/
    while(1)
    {
        FD_ZERO(&readSet);
        FD_SET(g_ttyS1_fd, &readSet);

        MaxFd = g_ttyS1_fd + 1;

        struTimeVal.tv_sec = 3;     /* 3s超时*/
        struTimeVal.tv_usec = 0;

        ret = select(MaxFd, &readSet, NULL, NULL, &struTimeVal);

        if(ret < 0)
        {
            HARDWARE_TEST("select tty failed.(err: %d)\n", ret);

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select tty timeout\n");

            if (5 <= cnt)
            {
                goto tcsetattr_1_err;
            }
            cnt++;
            continue;
        }

        if(FD_ISSET(g_ttyS1_fd, &readSet))
        {
            Count = read(g_ttyS1_fd, GpsBuf+allnum, (GPS_DATA_BUF_SIZE-1 - allnum));
            if(Count < 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d)\n", Count);

                if (5 <= cnt)
                {
                    goto tcsetattr_1_err;
                }
                cnt++;
                continue;
            }

            if((allnum < GPS_DATA_BUF_SIZE) && (0 != GPS_DATA_BUF_SIZE-1 - allnum))
            {
                /*HARDWARE_TEST("GNSS[$NEMA]: %s.\n\nallnum:%d Count: %d GPS_DATA_BUF_SIZE-1 - allnum: %d\n\n", 
                                                    GpsBuf, allnum, Count, GPS_DATA_BUF_SIZE-1 - allnum);*/
                allnum += Count;
                continue;
            }

            /* 起始地址*/
            Begb = GpsBuf;
            num = 0;
            allnum = 0;
            memset(exBuf, 0, sizeof(exBuf)); 
            memset(tmpBuf, 0, sizeof(tmpBuf));
            /* 先找到两个'$'符之间的字段*/
            Endb = strchr(Begb, '$');
            if (Endb == NULL)
            {
                continue;
            }
            /* 从第一个'$'字符的位置往后偏移一位*/
            Begb = Endb + 1;
            while(1)
            {
                Endb = strchr(Begb, '$');
                if(Endb == NULL)
                {
                    allnum = 0;
                    HARDWARE_TEST("GNSS[$NEMA]: %s\n", GpsBuf);
                    HARDWARE_TEST("GNSS[$single of NEMA message]: %s\n", tmpBuf);
                    HARDWARE_TEST("GNSS[$GPGSV]: %s\n", exBuf);
                    break;
                }
                num = Endb-Begb;
                /* 截取两个'$'符之间的字符串*/
                strncpy(tmpBuf, Begb, num);
                if(strstr(tmpBuf, "GSV"))
                {
                    strncpy(exBuf + allnum, Begb, num);
                    allnum += num;
                    /* 处理GSV字段的cn0值*/
                    svcn0[i] = StrToInts(tmpBuf, num);
                    HARDWARE_TEST("C/N0[%d]: 0x%x\n", i, svcn0[i]);
                    i++;
                    if (20 <= i)
                    {
                        break;
                    }
                }
                /* 指向下个数的起始地址*/
                Begb = Endb + 1;
            }

            /* GPS NEMA协议未能输出GSV字段，不能获取C/N0值，报错退出*/
            if (0 == strcmp(exBuf, ""))
            {
                HARDWARE_TEST("GPS can't output string \"GPGSV or BDGSV\"\n");
                goto tcsetattr_1_err;
            }

            if (20 <= i)
            {
                /* 攒够20个C/N0值，正常返回*/
                memcpy((void *)cn0, (void *)svcn0, sizeof(svcn0));
                i = 0;
                break;
            }
            /* 未攒够20个C/N0值，继续收串口数据分析*/
            continue;
        }

        /* 防止xds守护进程再次启动xds*/
        system("xds -q");
        usleep(100);
    }

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);

    ret = tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes\n", GPS_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;
    /* 测试完成启动xds*/
    system("xds -d");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Success!\n");

    return 1;

tcsetattr_2_err:

    tcsetattr(g_ttyS1_fd, TCSANOW, &old_tty_opt);

tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS1_fd);
    g_ttyS1_fd = -1;

open_err:
    /* 测试完成启动xds*/
    system("xds -d");
    usleep(100);
    HARDWARE_TEST("UART0 GPS TEST Failed!\n");

    return 0;
}

/************************************************************************
* 函数名: StrToInts 
* 功能描述: 将一串以逗号间隔的以字符串形式表示的整数转化为数组
*
* 输入参数: char *str: 待转化的字符串，传入的为$GPGSV字段消息
*                           int length: 字符串长度
*
* 输出参数: 
* 返回值: 转化后的数字
* 备注: CN0值为GPS NEMA协议$GPGSV的第7个字段值
*
*************************************************************************/
int StrToInts(char *str, int length)
{
    char *Begb = NULL;
    char *Endb = NULL;
    char s2[20] = {0};
    char gsvBuf[100] = {0};
    int i = 0;
    int data = 0;

    if(strcmp(str, "") == 0)
    {
        return 0;
    }

    if(length > 100)
    {
        length = 100;
    }
    strncpy(gsvBuf, str, length);

    HARDWARE_TEST("StrToInts string: %s.\n", gsvBuf);

    Begb = gsvBuf;   //起始地址
//    HARDWARE_TEST("1.   Begb: %s.\n", Begb);
    while(1)
    {
        Endb = strchr(Begb, ',');
        i++;
        
        if(Endb == NULL)
        {
            break;
        }

        if (i < 8)
        {
             /* 第7个逗号后是cn0值，指向下个数的起始地址*/
            Begb = Endb + 1;
//            HARDWARE_TEST("2.   i: %d Begb: %s.\n", i, Begb);
            continue;
        }

        memset(s2, 0, sizeof(s2)); 
        strncpy(s2, Begb, Endb-Begb);//截取两个逗号间的字符串
        data = atoi(s2);//string to double
        HARDWARE_TEST("s2: %s. Transform data: %d.\n", s2, data);
        break;
    }

    return data;
}

/************************************************************************
* 函数名: comuart_test 
* 功能描述: 测试COM UART接口正确性，控制台接comuart串口，做输入或者输出
* 输入参数: 
* 输出参数: 0: false   1: success
* 返回值: 无 
* 备注: 设备外接串口，从控制台输入，看comuart是否能收到数据
*
*************************************************************************/
int comuart_test(void)
{
    int ret = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int Count = 0;
    int num = 0;
    int allnum = 0;
    int MaxFd = 0;
    fd_set readSet;
    struct timeval struTimeVal;
    char recvBuf[BUFF_LEN] = {0};
    char rxBuf[BUFF_LEN] = {0};
    char txBuf[20] = "This is COMUART";
    int cnt = 0;

    HARDWARE_TEST("begin COM UART test!\n\r");

    /* COMUART作为控制台的功能删除*/
    system("echo \"0 4 1 7\" > /proc/sys/kernel/printk");     /* 关闭内核输出log*/
    system("ln -sf /sbin/fake_getty /sbin/getty");                  /* 创建假的getty软链接*/
    system("chmod a+x /sbin/getty");
#if 0
    system("chmod +x /bin/agit_setup");             /* 设置agit_setup脚本执行权限*/
    system("source /bin/agit_setup");                   /* 执行agit_setup脚本*/
    system("suspend_uart_users /dev/ttyS3");     /* 执行agit_setup脚本内函数，删除ttyS3控制台功能*/
    usleep(5000);
#else
    system("chmod +x /bin/comuart_setup");             /* 设置comuart_setup脚本执行权限*/
    system("source /bin/comuart_setup ttyS3");                   /* 执行comuart_setup脚本*/
    usleep(5000);
#endif

    /* set gpio comuart function*/
    //PIN_Mux_set(COM_UART_TX, 0);     /* uart1_tx 端口连接到外部管脚 U0TXD(gpio179)*/
    //PIN_Mux_set(COM_UART_RX, 0);     /* uart1_rx 端口连接到外部管脚 U0RXD(gpio180)*/
#if 1
    /* 初始化COMUART ttyS3*/
    g_ttyS3_fd = open(COMUART_DEV_FILE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(g_ttyS3_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", COMUART_DEV_FILE);
        goto open_err;
    }

    HARDWARE_TEST("Open %s success.\n", COMUART_DEV_FILE);

    tcflush(g_ttyS3_fd, TCIFLUSH);

    ret = tcgetattr(g_ttyS3_fd, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", COMUART_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* 设置COMUART串口波特率*/
    //cfsetispeed(&old_tty_opt, B230400);	//设置输入波特率230400
    //cfsetospeed(&old_tty_opt, B230400);	//设置输出波特率230400

    cfsetispeed(&old_tty_opt, B115200);	//设置输入波特率115200
    cfsetospeed(&old_tty_opt, B115200);	//设置输出波特率115200

    //cfsetispeed(&old_tty_opt, B9600);	//设置输入波特率9600
    //cfsetospeed(&old_tty_opt, B9600);	//设置输出波特率9600

    new_tty_opt = old_tty_opt;

    /* 设置为不需要回车或者换行就能发送出去*/
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tty_opt.c_oflag &= ~OPOST;

    ret = tcsetattr(g_ttyS3_fd, TCSANOW, &new_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", COMUART_DEV_FILE);
        goto tcsetattr_1_err;
    }
#else
    /* 初始化控制台console, 系统已将COMUART映射为控制台*/
    g_console_fd = open(CONSOLE_DEV_FILE, O_RDWR | O_NOCTTY);
    if(g_console_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", CONSOLE_DEV_FILE);
        goto tcsetattr_1_err;
    }

    HARDWARE_TEST("Open %s success.\n", CONSOLE_DEV_FILE);
#endif

#if 1
    Count = write(g_ttyS3_fd, txBuf, 20);//实际写入的长度 
    if(Count > 0)
    { 
        HARDWARE_TEST("COMUART ttyS3 send data length is %d\n", Count); 
    }
    else
    { 
        goto tcsetattr_2_err; 
    } 
    usleep(100);
#else
    Count = write(g_console_fd, txBuf, 20);//实际写入的长度 
    if(Count > 0)
    { 
        HARDWARE_TEST("COMUART console send data length is %d\n", Count); 
    }
    else
    { 
        goto tcsetattr_1_err; 
    } 
    usleep(100);
#endif

    Count = 0;
    memset(recvBuf, 0, BUFF_LEN);
    /* 接收信号*/
    while(1)
    {
        FD_ZERO(&readSet);
#if 1
        FD_SET(g_ttyS3_fd, &readSet);
        MaxFd = g_ttyS3_fd + 1;
#else
        FD_SET(g_console_fd, &readSet);
        MaxFd = g_console_fd + 1;
#endif

        struTimeVal.tv_sec = 5;    /* timeout value */
        struTimeVal.tv_usec = 0;

        ret = select(MaxFd, &readSet, NULL, NULL, &struTimeVal);
		
        if(ret < 0)
        {
            HARDWARE_TEST("select tty failed.(err: %d).\n", ret);

            if (3 <= cnt)
            {
                goto tcsetattr_2_err;
            }
            cnt++;
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select tty timeout.\n");

            if (5 <= cnt)
            {
                goto tcsetattr_2_err;
            }
            cnt++;
            continue;
        }
#if 1
        if(FD_ISSET(g_ttyS3_fd, &readSet))
        {
            Count = read(g_ttyS3_fd, recvBuf + allnum, (BUFF_LEN-1-allnum));
            if(Count <= 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d).\n", Count);

                if (3 <= cnt)
                {
                    goto tcsetattr_2_err;
                }
                cnt++;
                continue;
            }
            else
            {
                allnum += Count;
                /* (sizeof("This is COMUART")-1)为减去自动补的'\0'的1字节长度*/
                if (allnum < (sizeof("This is COMUART")-1))
                {
                    HARDWARE_TEST("allnum:%d, sizeof(\"This is COMUART\"):%d\nCOMUART Partial Read: %s\n", allnum, sizeof("This is COMUART"), recvBuf);
                    continue;
                }
                else
                {
                    /* 显示COMUART读到的数据*/
                    HARDWARE_TEST("COMUART ttyS3 Read: %s\n", recvBuf);
                    break;
                }
            }
        }
#else
        if(FD_ISSET(g_console_fd, &readSet))
        {

            memset(recvBuf, 0, BUFF_LEN);
            Count = read(g_console_fd, recvBuf, (BUFF_LEN-1));
            if(Count <= 0)
            {
                HARDWARE_TEST("Read tty failed.(err: %d).\n", Count);
                usleep(20000);      /* 睡眠20ms*/

                if (3 <= cnt)
                {
                    goto tcsetattr_1_err;
                }
                cnt++;
                continue;
            }
            else
            {
                if (Count <= (BUFF_LEN - 1 - num))
                {
                    strncpy(rxBuf + num, recvBuf, Count);
                    num += Count;
                }
                else
                {
                    strncpy(rxBuf + num, recvBuf, BUFF_LEN-1-num);
                    num = 0;
                }
                HARDWARE_TEST("console buffer: %s\n", recvBuf);

                if (NULL == strstr(recvBuf, "\n"))
                {
                    /* 未检测到回车键继续等待输入*/
                    continue;
                }
                /* 显示COMUART读到的数据*/
                HARDWARE_TEST("COMUART console Read: %s\n", rxBuf);
                break;
            }
        }
#endif
    }

#if 1
    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);

    ret = tcsetattr(g_ttyS3_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", COMUART_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS3_fd);
    g_ttyS3_fd = -1;
#else
    close(g_console_fd);
    g_console_fd = -1;
#endif

    /* 恢复COMUART作为控制台的功能*/
    system("ln -sf /bin/busybox /sbin/getty");                  /* 创建真的getty软链接*/
    system("chmod a+x /sbin/getty");
#if 0
    system("chmod +x /bin/agit_setup");             /* 设置agit_setup脚本执行权限*/
    system("source /bin/agit_setup");                   /* 执行agit_setup脚本*/
    system("suspend_uart_users");     /* 执行agit_setup脚本内函数，恢复ttyS3控制台功能*/
#else
    system("chmod +x /bin/comuart_setup");             /* 设置comuart_setup脚本执行权限*/
    system("source /bin/comuart_setup getty");                   /* 执行comuart_setup脚本*/
#endif
    system("echo \"7 4 1 7\" > /proc/sys/kernel/printk");     /* 打开内核输出log*/
    usleep(500);

    HARDWARE_TEST("COMUART test Success!\n");

    return 1;

tcsetattr_2_err:
    tcsetattr(g_ttyS3_fd, TCSANOW, &old_tty_opt);

tcsetattr_1_err:
#if 0
    close(g_console_fd);
    g_console_fd = -1;
#else
    close(g_ttyS3_fd);
    g_ttyS3_fd = -1;
#endif
open_err:
    /* 恢复COMUART作为控制台的功能*/
    system("ln -sf /bin/busybox /sbin/getty");                  /* 创建真的getty软链接*/
    system("chmod a+x /sbin/getty");
#if 0
    system("chmod +x /bin/agit_setup");             /* 设置agit_setup脚本执行权限*/
    system("source /bin/agit_setup");                   /* 执行agit_setup脚本*/
    system("suspend_uart_users /dev/ttyS3");     /* 执行agit_setup脚本内函数，恢复ttyS3控制台功能*/
#else
    system("chmod +x /bin/comuart_setup");             /* 设置comuart_setup脚本执行权限*/
    system("source /bin/comuart_setup getty");                   /* 执行comuart_setup脚本*/
#endif
    system("echo \"7 4 1 7\" > /proc/sys/kernel/printk");     /* 打开内核输出log*/
    usleep(500);

    HARDWARE_TEST("COMUART TEST Failed!\n");

    return 0;
}


/************************************************************************
* 函数名: uart0_test 
* 功能描述: 测试uart0发收正确性
* 输入参数: 
* 输出参数: 无
* 返回值: 无 
* 备注: uart0为rx与tx收发直接短接
*
*************************************************************************/
int uart0_test(void)
{
    int ret = 0;
    char tx_buf[UART0_TEST_NUM] = {0};
    char rx_buf[UART0_TEST_NUM] = {0};
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int count = 0;
    int len = 0;
    int MaxFd = 0;
    fd_set fs_read; 
    struct timeval tv_timeout; 

    HARDWARE_TEST("Begin UART0_TEST Test!\n\r");

    /* set gpio uart0 function*/
    PIN_Mux_set(UART0_TX, 0);     //uart0_tx 端口连接到外部管脚 U0TXD(gpio76)
    PIN_Mux_set(UART0_RX, 0);     //uart0_rx 端口连接到外部管脚 U0RXD(gpio77)

    g_ttyS0_fd = open(UART0_DEV_FILE, O_RDWR | O_NOCTTY |O_NDELAY);
    if(g_ttyS0_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", UART0_DEV_FILE);
        goto open_err;
    }
	
    HARDWARE_TEST("Open %s success.\n", UART0_DEV_FILE);

    /**1. tcgetattr函数用于获取与终端相关的参数。 
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中 
    */ 
    ret = tcgetattr(g_ttyS0_fd, &old_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", UART0_DEV_FILE);
        goto tcgetattr_err;
    }

    new_tty_opt = old_tty_opt;

    /**2. 修改所获得的参数*/ 
    new_tty_opt.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能 
    new_tty_opt.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位 
    new_tty_opt.c_cflag &= ~CRTSCTS;//无硬件流控 
    new_tty_opt.c_cflag |= CS8;//8位数据长度 
    new_tty_opt.c_cflag &= ~CSTOPB;//1位停止位 
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    new_tty_opt.c_iflag |= IGNPAR;//无奇偶检验位
//    new_tty_opt.c_oflag = 0; //输出模式
//    new_tty_opt.c_lflag = 0; //不激活终端模式

    new_tty_opt.c_oflag &= ~(ICANON | OPOST); //不需要回车就能read buffer中的数据
    
    new_tty_opt.c_cc[VTIME] = 0;    //读取超时时间
    new_tty_opt.c_cc[VMIN] = 0;    //读取的最小字符数
    cfsetospeed(&new_tty_opt, B115200);     //设置波特率 
    cfsetispeed(&new_tty_opt, B115200);	//设置输入波特率115200

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/ 
    tcflush(g_ttyS0_fd, TCIOFLUSH);//溢出数据可以接收，但不读 
    ret = tcsetattr(g_ttyS0_fd, TCSANOW, &new_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART0_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* UART0自发自收测试*/
#if 1
    write(g_ttyS0_fd, uart0_test_buf, UART0_TEST_NUM);
    usleep(5000);
    count = read(g_ttyS0_fd, rx_buf, UART0_TEST_NUM);
    if(count < 0)
    {
        HARDWARE_TEST("Read tty failed.(err: %d).\n", count);
        goto readerr;
    }
    /* 将ttyS1读到的字符串保存到日志文件中*/
    HW_LOG("Read ttyS0: %s.\n", rx_buf);

    if(0 != strcmp(uart0_test_buf, rx_buf))
    {
        HARDWARE_TEST("TTYS0 Write and Read are different!");
        goto readerr;
    }
#else
    HARDWARE_TEST("Please input any statement.\n\reg:This is ttyS0\n\r");
    /* 阻塞读取控制台console输入的字符*/
    while (1)
    {
        FD_ZERO(&fs_read); 
        FD_SET(g_ttyS0_fd, &fs_read);

        MaxFd = g_ttyS0_fd + 1;

        tv_timeout.tv_sec = 0;
        tv_timeout.tv_usec = SELECT_TIMEOUT_500MS;

        count = read(STDIN_FILENO, tx_buf, UART0_TEST_NUM);
        if(count < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        count = count-1;
        if(0 == count)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /* 控制台console读取到的字符，通过串口自发自收测试*/
        len = write(g_ttyS0_fd, tx_buf, count);//实际写入的长度 
        if(len > 0)
        { 
            HARDWARE_TEST("ttyS0 send data length is %d\n", len); 
        }
        else
        { 
            goto writeerr; 
        } 
        sleep(1);

        ret = select(MaxFd, &fs_read, NULL, NULL, &tv_timeout);
        if(ret < 0)
        {
            HARDWARE_TEST("select ttyS0 failed.(err: %d).\n", ret);
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select ttyS0 timeout.\n");
            goto readerr;
        }

        if(FD_ISSET(g_ttyS0_fd, &fs_read))
        {
            len = read(g_ttyS0_fd, rx_buf, UART0_TEST_NUM);
            if(len < 0)
            {
                HARDWARE_TEST("Read ttyS0 failed.(err: %d).\n", len);
                goto readerr;
            }
            HARDWARE_TEST("Read ttyS0 length %d.\n", len);
            /* 将ttyS0读到的字符串保存到日志文件中*/
            HW_LOG("Read ttyS1: %s.\n", rx_buf);
        }

        ret = strncmp(tx_buf, rx_buf, len);
        if(0 != ret)
        {
            HARDWARE_TEST("TTYS0 Write and Read are different!");
            goto readerr;
        }
        break;
    }
#endif

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);
    tcflush(g_ttyS0_fd, TCIOFLUSH);

    ret = tcsetattr(g_ttyS0_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART0_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS0_fd);
    g_ttyS0_fd = -1;
    HARDWARE_TEST("UART0 test Success!\n");

    return 1;

tcsetattr_2_err:
readerr:
    tcsetattr(g_ttyS0_fd, TCSANOW, &old_tty_opt);
writeerr:
tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS0_fd);
    g_ttyS0_fd = -1;

open_err:
    HARDWARE_TEST("UART0 test Failed!\n");

    return 0;
}

/************************************************************************
* 函数名: uart2_test 
* 功能描述: 测试uart2发收正确性
* 输入参数: 
* 输出参数: 无
* 返回值: 无 
* 备注: uart1为rx与tx收发直接短接
*                             uart2 cts和uart2 rts作GPIO直连收发对接判断连通性。
*
*************************************************************************/
void uart2_test(void)
{
    int ret = 0;
    char tx_buf[UART2_TEST_NUM] = {0};
    char rx_buf[UART2_TEST_NUM] = {0};
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int count = 0;
    int len = 0;
    int MaxFd = 0;
    fd_set fs_read; 
    struct timeval tv_timeout; 

    HARDWARE_TEST("Begin UART2_TEST Test!\n\r");

    /* set gpio uart0 function*/
    PIN_Mux_set(UART2_TX, 0);     //uart2_tx 端口连接到外部管脚 U0TXD(gpio76)
    PIN_Mux_set(UART2_RX, 0);     //uart2_rx 端口连接到外部管脚 U0RXD(gpio77)

    g_ttyS2_fd = open(UART2_DEV_FILE, O_RDWR | O_NOCTTY |O_NDELAY);
    if(g_ttyS2_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", UART2_DEV_FILE);
        goto open_err;
    }
	
    HARDWARE_TEST("Open %s success.\n", UART2_DEV_FILE);

    /**1. tcgetattr函数用于获取与终端相关的参数。 
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中 
    */ 
    ret = tcgetattr(g_ttyS2_fd, &old_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", UART2_DEV_FILE);
        goto tcgetattr_err;
    }

    new_tty_opt = old_tty_opt;

    /**2. 修改所获得的参数*/ 
    new_tty_opt.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能 
    new_tty_opt.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位 
    new_tty_opt.c_cflag &= ~CRTSCTS;//无硬件流控 
    new_tty_opt.c_cflag |= CS8;//8位数据长度 
    new_tty_opt.c_cflag &= ~CSTOPB;//1位停止位 
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    new_tty_opt.c_iflag |= IGNPAR;//无奇偶检验位
//    new_tty_opt.c_oflag = 0; //输出模式
//    new_tty_opt.c_lflag = 0; //不激活终端模式

    new_tty_opt.c_oflag &= ~(ICANON | OPOST); //不需要回车就能read buffer中的数据
    
    new_tty_opt.c_cc[VTIME] = 0;    //读取超时时间
    new_tty_opt.c_cc[VMIN] = 0;    //读取的最小字符数
    cfsetospeed(&new_tty_opt, B115200);     //设置波特率 
    cfsetispeed(&new_tty_opt, B115200);	//设置输入波特率115200

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/ 
    tcflush(g_ttyS2_fd, TCIOFLUSH);//溢出数据可以接收，但不读 
    ret = tcsetattr(g_ttyS2_fd, TCSANOW, &new_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART2_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* UART2自发自收测试*/
#if 0
    write(g_ttyS2_fd, uart2_test_buf, UART2_TEST_NUM);
    usleep(5000);
    count = read(g_ttyS2_fd, rx_buf, UART2_TEST_NUM);
    if(count < 0)
    {
        HARDWARE_TEST("Read tty failed.(err: %d).\n", count);
        goto readerr;
    }
    /* 将ttyS2读到的字符串保存到日志文件中*/
    HW_LOG("Read ttyS2: %s.\n", rx_buf);

    if(0 != strcmp(uart2_test_buf, rx_buf))
    {
        HARDWARE_TEST("TTYS2 Write and Read are different!");
        goto readerr;
    }
#else
    HARDWARE_TEST("Please input any statement.\n\reg:This is ttyS2\n\r");
    /* 阻塞读取控制台console输入的字符*/
    while (1)
    {
        FD_ZERO(&fs_read); 
        FD_SET(g_ttyS2_fd, &fs_read);

        MaxFd = g_ttyS2_fd + 1;

        tv_timeout.tv_sec = 0;
        tv_timeout.tv_usec = SELECT_TIMEOUT_500MS;

        count = read(STDIN_FILENO, tx_buf, UART2_TEST_NUM);
        if(count < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        count = count-1;
        if(0 == count)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /* 控制台console读取到的字符，通过串口自发自收测试*/
        len = write(g_ttyS2_fd, tx_buf, count);//实际写入的长度 
        if(len > 0)
        { 
            HARDWARE_TEST("ttyS2 send data length is %d\n", len); 
        }
        else
        { 
            goto writeerr; 
        } 
        sleep(1);

        ret = select(MaxFd, &fs_read, NULL, NULL, &tv_timeout);
        if(ret < 0)
        {
            HARDWARE_TEST("select ttyS2 failed.(err: %d).\n", ret);
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select ttyS2 timeout.\n");
            goto readerr;
        }

        if(FD_ISSET(g_ttyS2_fd, &fs_read))
        {
            len = read(g_ttyS2_fd, rx_buf, UART2_TEST_NUM);
            if(len < 0)
            {
                HARDWARE_TEST("Read ttyS2 failed.(err: %d).\n", len);
                goto readerr;
            }
            HARDWARE_TEST("Read ttyS2 length %d.\n", len);
            /* 将ttyS2读到的字符串保存到日志文件中*/
            HW_LOG("Read ttyS2: %s.\n", rx_buf);
        }

        ret = strncmp(tx_buf, rx_buf, len);
        if(0 != ret)
        {
            HARDWARE_TEST("TTYS2 Write and Read are different!");
            goto readerr;
        }
        break;
    }
#endif

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);
    tcflush(g_ttyS2_fd, TCIOFLUSH);

    ret = tcsetattr(g_ttyS2_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART2_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS2_fd);
    g_ttyS2_fd = -1;
    HARDWARE_TEST("UART2_TEST Success!\n");

    return;

tcsetattr_2_err:
readerr:
    tcsetattr(g_ttyS2_fd, TCSANOW, &old_tty_opt);
writeerr:
tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS2_fd);
    g_ttyS2_fd = -1;

open_err:
    HARDWARE_TEST("UART2_TEST Failed!\n");

    return;
}

/************************************************************************
* 函数名: module_hardware_ver_test 
* 功能描述:判断DMD模组硬件版本ADCI2值
*
* 输入参数:None
* 
* 输出参数:DMD模组硬件类型
*  
* 返回值: 无 
* 备注: ADC Channel4 为硬件版本通道
*
*************************************************************************/
int module_hardware_ver_test(void)
{
    FILE *mmc_ptr = NULL;
    char eMCP_type[2] = {UNKNOWN};
    char path[100] = {0};
    int ret;
    int fd_hdver = -1;
    unsigned short hardversion = ~0;
    int cmp_val = 0;

    HARDWARE_TEST("Begin module hardware version test!\n\r");

    fd_hdver = open("/dev/dtvl_cdev", O_RDWR);
    if (0 >= fd_hdver)
    {
        HARDWARE_TEST("Open /dev/dtvl_cdev driver Failed, Return %d.\n", fd_hdver);
        goto err1;
    }

    ret = read(fd_hdver, &hardversion, sizeof(hardversion));
    if (-1 == ret)
    {
        HARDWARE_TEST("Read(ADC Channel4 voltage) module hardware version Failed, Return %d.\n", ret);
        goto err2;
    }
    else
    {
        HARDWARE_TEST("Read(ADC Channel4 voltage) module hardware version hardversion:%d.\n", hardversion);
    }

    sprintf(path, "/sys/class/mmc_host/mmc0/mmc0\\:0001/%s", MMC_DEV_NAME);
    mmc_ptr = fopen("/sys/class/mmc_host/mmc0/mmc0\:0001/csd_revision", "r");
    if (NULL == mmc_ptr)
    {
        HARDWARE_TEST("fopen %s Failed.\n", path);
        goto err2;
    }

    /*Read location 0*/
    fseek(mmc_ptr, 0, SEEK_SET);
    ret = fread(eMCP_type, sizeof(char), 2, mmc_ptr);
    if (0 > ret)
    {
        HARDWARE_TEST("Read(/sys/class/mmc_host/mmc0/mmc0\\:0001/) eMCP type Failed, Return %d.\n", ret);
        goto err3;
    }
    else
    {
        HARDWARE_TEST("Read eMCP type:%c%c.\n", eMCP_type[0], eMCP_type[1]);
    }
    fclose(mmc_ptr);
    close(fd_hdver);

    /*leadcore 1.0:ADC voltage-0.49v, A/D Conversion Data-717*/
    if((617 <= hardversion) && (hardversion <= 817))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is LEADCORE 1.0.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return LEADCORE_V1;
    }
    /*linktester 1.0:ADC voltage-0.89v, A/D Conversion Data-1310*/
    else if((1210 <= hardversion) && (hardversion <= 1410))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is LINKTESTER 1.0.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return DTLINKTESTER_V1;
    }
    /*Samsung KDDI:ADC voltage-1.12v, A/D Conversion Data-1638*/
    else if((1538 <= hardversion) && (hardversion <= 1738))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is Samsung KDDI.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return SAMSUNGKDDI_V1;
    }
	/*Samsung KDDI:ADC voltage-2.15v, A/D Conversion Data-3143, base on linktester 2.0*/
    else if((3043 <= hardversion) && (hardversion <= 3243))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is Samsung KDDI.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return SAMSUNGKDDI_V2;
    }
    /*leadcore:ADC voltage-1.4v, A/D Conversion Data-2048*/
    else if ((1948 <= hardversion) && (hardversion <= 2148))
    {
        cmp_val = strncmp(eMCP_type, "07", 2);
        if (0 >= cmp_val)
        {
            /* old eMCP: It's eMCP EXT CSD revision is 07(type char) */
            HARDWARE_TEST("Read ADC Channel4 voltage:0x%x, Read eMCP type:%c%c, return cmp_val:%d, Module type is LEADCORE 2.0.\n", hardversion, eMCP_type[0], eMCP_type[1], cmp_val);
            HARDWARE_TEST("module hardware version test Success!\n");
            return LEADCORE_V2;
        }
        else
        {
            /* new eMCP: It's eMCP EXT CSD revision is 08(type char) */
            HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, return cmp_val:%d, Module type is LEADCORE 3.0.\n", hardversion, eMCP_type[0], eMCP_type[1], cmp_val);
            HARDWARE_TEST("module hardware version test Success!\n");
            return LEADCORE_V3;
        }
    }
    /*linktester 2.0:ADC voltage-1.68v, A/D Conversion Data-2458*/
    else if((2358 <= hardversion) && (hardversion <= 2558))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is LINKTESTER 2.0.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return DTLINKTESTER_V2;
    }
    /*mitrastar:ADC voltage-1.9v, A/D Conversion Data-2785*/
    else if((2685 <= hardversion) && (hardversion <= 2885))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is MITRASTAR 1.0.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return TWMITRASTAR_V1;
    }
    /*ALPS Alpine 1.0:ADC voltage-2.31v, A/D Conversion Data-3379*/
    else if((3279 <= hardversion) && (hardversion <= 3479))
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is Alps Alpine 1.0.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return DMD3A_JPALPS_V1;
    }
    else
    {
        HARDWARE_TEST("Read ADC Channel4 voltage is 0x%x, Read eMCP type:%c%c, Module type is UNKNOW.\n", hardversion, eMCP_type[0], eMCP_type[1]);
        HARDWARE_TEST("module hardware version test Success!\n");
        return UNKNOWN;
    }

err3:
    fclose(mmc_ptr);
err2:
    close(fd_hdver);
err1:
    HARDWARE_TEST("Module type is UNKNOW(0x%02x).\n\rmodule hardware version test Failed!", UNKNOWN);

    return UNKNOWN;
}

/************************************************************************
* 函数名: io_board_hardware_ver_test 
* 功能描述:判断IO板卡硬件版本ADCI1值
*
* 输入参数:None
* 
* 输出参数:IO板卡硬件类型
*  
* 返回值: 无 
* 备注: ADC Channel3 为硬件版本通道
*               每种板卡之间留有数字+-150的余量，模拟值为+-0.1v
*************************************************************************/
int io_board_hardware_ver_test(void)
{
    int ret;
    float value = 0;
    unsigned short hardversion = ~0;
	int module_version = 0;
	int get_adci;

//    HARDWARE_TEST("Begin io board hardware version test!\n");
	module_version = module_hardware_ver_test();

	get_adci = get_adci_from_device(&hardversion);
	if (0==get_adci)
	{
		HARDWARE_TEST("ADCI1 hardversion %d from hd_version\n", hardversion);
	}
	else
	{
		HARDWARE_TEST("open /proc/dtvl/hd_version fail\n",__FUNCTION__,__LINE__);
		value = pmu_lc1160_adc_read(ADCI1, &hardversion);
		HARDWARE_TEST("ADCI1 convert value: %f, hardversion %d\n", value, hardversion);
	}

    /*DTVL3100 OBU/RSU: ADC voltage-0v, A/D Conversion Data-0*/
    if((0==hardversion)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3100 OBU/RSU.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3100;
    }
    /*VU3004/VU3005: ADC voltage-0.234v, A/D Conversion Data-342; 
    	VU3004 BATINSN 0.058v; VU3005 BATINSN 0.254v; GNSS-qx*/
    else if((242<=hardversion)&&(hardversion<=442)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is VU3004/VU3005.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return VU300X;
    }
	/* if use, open it. 20200925 add this code, from  NR-master to NR-slave.*/
	#if 0
	/*NR-master: ADC voltage-0.575v, A/D Conversion Data-841, GNSS-qx*/
    else if((741<=hardversion)&&(hardversion<=941)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is NR-master.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return NR_MASTER;
    }	
	/*NR-slave: ADC voltage-0.900v, module 31, A/D Conversion Data-1316; BATINSN 0.254v; GNSS-qx*/
    else if((1216<=hardversion)&&(hardversion<=1416)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is NR-slave.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return NR_SLAVE;
    }
	#endif
    /*DTVL3000 OBU && DTVL3000 RSU: ADC voltage-(0.5v-1.5v), A/D Conversion Data-(732-2194)*/
    else if((632<=hardversion)&&(hardversion<= 2294)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3000 OBU/RSU.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3000;
    }
    /*UMCC1 EVB: ADC voltage-0.9v, module 3a, A/D Conversion Data-1317; BATINSN 1.2245; */
    else if((1217<=hardversion)&&(hardversion<=1417)&&(DMD3A_JPALPS_V1==module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is UMCC1 EVB.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return UMCC1_EVB;
    }
	/*VU3205: ADC voltage-1.64v, module 31, A/D Conversion Data-2399; BATINSN 0.254v; */
    else if((2299<=hardversion)&&(hardversion<=2499)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is VU3205.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return VU3205;
    }
    /*DTVL3110 RSU: ADC voltage-(1.95v-2.35v), A/D Conversion Data-(2852-3438)*/
    else if((2752<=hardversion)&&(hardversion <= 3538)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3110 RSU.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3110;
    }
	/*DTVL3110E RSU: ADC voltage-(1.95v-2.35v), A/D Conversion Data-(2852-3438), base on DMD3A module*/
    else if((2752<=hardversion)&&(hardversion <= 3538)&&(DMD3A_JPALPS_V1==module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3110E RSU.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3110E;
    }
	#if 0
    /*SUZUNE EVK ES1: ADC voltage-1.22v, A/D Conversion Data-1785*/
    else if((1635 <= hardversion) && (hardversion <= 1935))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is SUZUNE EVK ES1.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return SUZUNE_EVK_ES1;
    }
    /*DTVL3000 RSU: ADC voltage-1.5v, A/D Conversion Data-2194*/
    else if ((2044 <= hardversion) && (hardversion <= 2344))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3000.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3000;
    }
	#endif
    /*DTVL3100-VBOX: ADC voltage-1.8v, A/D Conversion Data-2633*/
    else if((2533<=hardversion)&&(hardversion<=2733)&&(DMD3A_JPALPS_V1!=module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is DTVL3100 VBOX.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return DTVL3100_VBOX;
    }
	/* SUZUNE3: 0.4355V(637): 537-737, batinsn 1.2245v(3376): 3276-3476 */
	else if((537<=hardversion)&&(hardversion<=737)&&(DMD3A_JPALPS_V1==module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is SUZUNE3.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return SUZUNE3;
    }
	/* VU400X: 1.4V(2048): 1948-2148, batinsn 2.308v(1791): 1691-1891 */
	else if((1948<=hardversion)&&(hardversion<=2148)&&(DMD3A_JPALPS_V1==module_version))
    {
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is VU400X.\n", hardversion);
        HARDWARE_TEST("IO board hardware version test Success!\n");
        return VU400X;
    }
	else
	{
        HARDWARE_TEST("Read ADC Channel3 voltage is 0x%x, IO board type is UNKNOW.\n", hardversion);
        HARDWARE_TEST("IO board type is UNKNOW(0x%02x).\n", UNKNOWN);
        return UNKNOWN;
    }
}

int child_board_hardware_ver_test(int board_num)
{
	/*gpio79 value 1: PKB board, board 1
	 *gpio79 value 0: SUWB board, board 2
	*/
	int adc_ret;
	unsigned short value_batinsn = ~0;
	FILE *board_fd;
	char buf[5] = {0};

	/* set gpio79 for board mode */
	if(0==set_gpio_direction(79,0))
	{
		HARDWARE_TEST("%s line%d: set gpio value ok\n", __FUNCTION__, __LINE__);
	}

	if (board_num==1)
	{
		if(0==set_gpio_value(79, 1))
			HARDWARE_TEST("%s line%d: set gpio value ok\n", __FUNCTION__, __LINE__);
	}
	if (board_num==2)
	{
		if(0==set_gpio_value(79, 0))
			HARDWARE_TEST("%s line%d: set gpio value ok\n", __FUNCTION__, __LINE__);
	}

	/* judge version file exist */
	if(access("/proc/dtvl/pltestadc", F_OK) != 0){
  		HARDWARE_TEST("can't find /proc/dtvl/pltestadc, turn to i2c read\n");
		adc_ret = pmu_lc1160_adc_read(BATINSN, &value_batinsn);
	}else{
		board_fd = fopen("/proc/dtvl/pltestadc", "r");
        if (NULL == board_fd){
            /* ²»ÄÜ´ò¿ªlogÎÄ¼þ*/
            HARDWARE_TEST("Can't open lsusb.log file!\n\r");
        }
        if (0 != fseek(board_fd, 0, SEEK_SET)){
            /* ÒÆ¶¯Ö¸ÕëÎ»ÖÃÊ§°Ü*/
            fclose(board_fd);
        }

        fread(buf, 1, 5, board_fd);
		HARDWARE_TEST("read: %s\n", buf);
		fclose(board_fd);
		value_batinsn = atoi(buf);
	}
	HARDWARE_TEST("%s line%d: BATINSN:%d\n", __FUNCTION__, __LINE__, value_batinsn);
	/*batinsn 2.308v(3376), 3276-3476*/
	if((value_batinsn>=3276) && (value_batinsn<=3476)){
		HARDWARE_TEST("Subboard batinsn value %d, type in range\n", value_batinsn);
		return SUBBOARD2_SUWB_V1;
	}else{
		HARDWARE_TEST("Subboard batinsn value %d, type not in range\n", value_batinsn);
		return UNKNOWN;
	}
	
}

/************************************************************************
* 函数名: module_firmware_ver_test 
* 功能描述:查询模组固件版本版本号
*
* 输入参数:None
* 
* 输出参数:模组固件版本版本号
*  
* 返回值: 版本号字符串长度
* 备注: uname -a命令返回值如下
* [root@dtvl3000 ~]#uname -a
* Linux dtvl3000 3.10.101@DTVL_SDK DMD3A_V1.0.0.0M_Build_202003051535 #1 SMP PREEMPT Thu Mar 5 15:31:02 CST 2020 3a_1.0.X.M:b36696:: armv7l GNU/Linux
* [root@dtvl3000 ~]#
*************************************************************************/
int module_firmware_ver_test(char *ver_buf)
{
    int length = 0;
    char buf[BUFF_LEN] = {0};
    char cmp_buf[] = "DMD";
    FILE *fd_log = NULL;
    int ret = 0;
    char *ptr_b = NULL;
    char *ptr_e = NULL;

    HARDWARE_TEST("Begin reach firmware version!\n\r");
    memset(buf, 0, sizeof(buf));

    /* 执行"uname -a"命令，并将结果保存到文件中*/
    system("uname -a > firmware.log");
    usleep(1000);

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./firmware.log", F_OK))
    {
        fd_log = fopen("./firmware.log", "r");
        if (NULL == fd_log)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open firmware.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_log, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, BUFF_LEN, fd_log);

        /* 从log文件中查找是否有有效版本号信息*/
        ptr_b = strstr(buf, cmp_buf);
        if (NULL == ptr_b)
        {
            /* 未找到固件版本信息*/
            HARDWARE_TEST("Can't find result of firmware version!\n\r");
            goto err1;
        }

        ptr_e = strchr(buf, '#');
        if (NULL == ptr_e)
        {
            /* 未找到固件版本信息*/
            HARDWARE_TEST("Can't find result of firmware version!\n\r");
            goto err1;
        }
        HW_LOG("uname -a:\n\r%s\n\r", buf);

        length = ptr_e - ptr_b - 1;
        memcpy((void *)ver_buf, (void *)ptr_b, length);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("reach firmware version Success!\n\r");
    fclose(fd_log);
    system("rm -rf ./firmware.log");
    return length;

err1:
    fclose(fd_log);
    system("rm -rf ./firmware.log");
err2:
    HARDWARE_TEST("reach firmware version failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: signature_check_test 
* 功能描述:查询模组是否已签名
*
* 输入参数:None
* 
* 输出参数:已签名返回1; 
*                           未签名返回2;
*  
* 返回值: 已签名:69 is 96; 未签名:69 is shit; 失败:0;
* 备注: cat /proc/ffdata命令返回值如下
* [root@dtvl3000 ~]#cat /proc/ffdata
* 0. info is valid
* 1. 0x7
* 2. 1805[0x70d]
* 3. 0xffffffff
* 4. 0x1
* 5. 0x000000000000000d
* 6. 0x0000000017777777
* 7. OBU[0x0]
* 8. V1@LinkTester[0x20001]
* 9. 0x32a5c337900f9599
* 10. 69 is shit
* [root@dtvl3000 ~]#
*************************************************************************/
int signature_check_test(void)
{
    int flag = 0;
    char buf[BUFF_LEN] = {0};
    char cmp1_buf[] = "69 is 96";
    char cmp2_buf[] = "69 is shit";
    FILE *fd_log = NULL;
    int ret = 0;
    char *ptr_b = NULL;
    char *ptr_e = NULL;

    HARDWARE_TEST("Begin signature check!\n\r");
    memset(buf, 0, sizeof(buf));

    /* 执行"cat /proc/ffdata"命令，并将结果保存到文件中*/
    system("cat /proc/ffdata > signature.log");
    usleep(1000);

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./signature.log", F_OK))
    {
        fd_log = fopen("./signature.log", "r");
        if (NULL == fd_log)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open signature.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_log, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, BUFF_LEN, fd_log);

        /* 从log文件中查找是否有判断签名的信息*/
        ptr_b = strstr(buf, cmp1_buf);
        //HARDWARE_TEST("read signature character string: %s\n\r", buf);
        if (NULL == ptr_b)
        {
            ptr_e = strstr(buf, cmp2_buf);
            if (NULL == ptr_e)
            {
                /* 未找到判断签名的信息*/
                HARDWARE_TEST("Can't find result of signature check!\n\r");
                goto err1;
            }
            else
            {
                /* 模组未签名*/
                flag = 2;
                HARDWARE_TEST("module haven't signature!\n\r");
            }
        }
        else
        {
            /* 模组已签名*/
            flag = 1;
            HARDWARE_TEST("module have signature!\n\r");
        }

        HW_LOG("cat /proc/ffdata:\n\r%s\n\r", buf);
    }
    else
    {
        goto err2;
    }

    fclose(fd_log);
    system("rm -rf ./signature.log");
    return flag;

err1:
    fclose(fd_log);
    system("rm -rf ./signature.log");
err2:
    HARDWARE_TEST("signature check failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: module_cid_check_test 
* 功能描述:查询模组存储芯片EMMC的CID号
*
* 输入参数:None
* 
* 输出参数:成功返回CID号及长度; 
*                           失败返回0;
*  
* 返回值: 
* 备注:
* 
*************************************************************************/
int module_cid_check_test(void *cid)
{
    FILE *cid_ptr = NULL;
    unsigned char cid_buf[33] = {0};
    int length = 0;

    HARDWARE_TEST("Get emcp cid.\r\n");
    memset(cid_buf, 0, sizeof(cid_buf));

    cid_ptr = fopen("/sys/class/mmc_host/mmc0/mmc0\:0001/cid", "r");
    if (NULL == cid_ptr)
    {
        HARDWARE_TEST("Open '/sys/class/mmc_host/mmc0/mmc0\\:0001/cid' Failed.\r\n");
        return 0;
    }

    /*Read location 0*/
    fseek(cid_ptr, 0, SEEK_SET);
    length = fread(cid_buf, sizeof(unsigned char), 32, cid_ptr);
    if (0 > length)
    {
        HARDWARE_TEST("Read '/sys/class/mmc_host/mmc0/mmc0\\:0001/cid' eMCP Card identification Failed, return %d.\r\n", length);
        goto err;
    }
    else if (31 == length)
    {
        /* cid号只有31位，低15位取出并在前头补0，用作申请签名*/
        strcpy(cid, "0");
        memcpy((void *)cid + 1, (void *)cid_buf + 16, 15*sizeof(unsigned char));
    }
    else if (32 == length)
    {
        /* cid号有32位，低16位取出，用作申请签名*/
        memcpy((void *)cid, (void *)cid_buf + 16, 16*sizeof(unsigned char));
    }
    else
    {
        HARDWARE_TEST("CID number not good, less than 31 bits. cid length:%d.\r\n", length);
        length = 0;
        goto err;
    }
    HARDWARE_TEST("Read eMCP Card identification:%s.\r\n", cid_buf);

    length = 16;

err:
    fclose(cid_ptr);
    return length;
}

/************************************************************************
* 函数名: adc_convert_test
* 功能描述:lc1160的ADC通道0~通道5的convert值
*
* 输入参数:
* 
* 输出参数:
*  
* 返回值: 无 
* 备注: ADC Channel4 为硬件版本通道
*
*************************************************************************/
void adc_convert_test(void)
{
    int channel = 0;
    char rxbuf[ADC_TEST_BYTES] = {0};
    int num = 0;
    float value = 0;
    unsigned short adc = 0;

    HARDWARE_TEST("Begin ADC convert Test!\n\r");

    /* 阻塞读取用户输入的ADC待读取通道*/
    while(1)
    {
        HARDWARE_TEST("Please input ADC Channel :0~4.\n\rQ(q): Return adc convert test.\n\r");
        /* 获取ADC通道号，如果输入是Q(q)则退出*/
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, ADC_TEST_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出GPIO测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit ADC convert test!\n\r");
                break;
            }
            /* 数字字符'0~4'*/
            else if ((rxbuf[0] >= 0x30) && (rxbuf[0] <= 0x34))
            {
                /* 控制台读取到的，空格字符前的字符为GPIO number值*/
                ASCToInt(rxbuf, &channel, num);
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input ADC Channel ERROR!Please input again!\n\r");
                continue;
            }
        }
        /* 输入字符错误*/
        else
        {
            HARDWARE_TEST("Your input ADC Channel ERROR!Please input again!\n\r");
            continue;
        }

        value = pmu_lc1160_adc_read(channel, &adc);
        break;
    }

     switch(channel)
    {
        case ADC_VBAT:
        {
            HARDWARE_TEST("ADC_VBAT(ADC%d) convert value = %f\n\r", channel, value);
            break;
        }
        case BATINSN:
        {
            HARDWARE_TEST("BATINSN(ADC%d) convert value = %f\n\r", channel, value);
            break;
        }
        case ADCI0:
        {
            HARDWARE_TEST("ADCI0(ADC%d) convert value = %f\n\r", channel, value);
            break;
        }
        case ADCI1:
        {
            HARDWARE_TEST("ADCI1(ADC%d) convert value = %f\n\r", channel, value);
            return;
        }
        case ADCI2:
        {
            HARDWARE_TEST("ADCI2(ADC%d) convert value = %f\n\r", channel, value);
            break;
        }
        default:
        {
            HARDWARE_TEST("Input ADC Channel ERROR!\n\r");
        }
    }
    HARDWARE_TEST("ADC convert Test Success!\n\r");

    return;
}

/************************************************************************
* 函数名: tcard_test 
* 功能描述:测试往TF卡中写字符，判断写入和读出的是否一致
*
* 输入参数:None
* 
* 输出参数:None
*  
* 返回值: 无 
* 备注: 
*
*************************************************************************/
void tcard_test(void)
{
    FILE *tf_fp = NULL;
    char w_buf[TCARD_TEST_BYTES] = {0};
    char r_buf[TCARD_TEST_BYTES] = {0};
    char path_buf[MNT_PATH_BYTES] = {0};
    char name_buf[20] = "/tcardtest.log";
    int ret = 0;
    int num = 0;
    int i = 0;

    HARDWARE_TEST("Begin TCard_test Test!\n\r");
#if 0
    strncp(pathbuf, TCARD_TEST_PATH, sizeof(TCARD_TEST_PATH));
#else
    HARDWARE_TEST("Please input TCard mount path!\n\rExample:/mnt/sdcard");

    /* 读取从控制台console输入的TF卡挂载目录*/
    while (1)
    {
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, path_buf, (MNT_PATH_BYTES - sizeof(name_buf)));
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /* 判断控制台输入字符是否正确*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if((path_buf[i] >= 0x30) && (path_buf[i] <= 0x39))
            {
                continue;
            }
            /* 小写字母字符'a~z'*/
            else if ((path_buf[i] >= 0x61) && (path_buf[i] <= 0x7A))
            {
                continue;
            }
            /* 大写字母字符'A~Z'*/
            else if ((path_buf[i] >= 0x41) && (path_buf[i] <= 0x5A))
            {
                continue;
            }
            /* 斜杠字符'/'*/
            else if (path_buf[i] == 0x2F)
            {
                continue;
            }
            else
            {
                HARDWARE_TEST("Console input error.\n\r");
                goto err1;
            }
        }

        if (num == i)
        {
            break;
        }
    }
    /* 删除路径缓存的多余字节*/
    path_buf[num] = '\0';
    strcat(path_buf, name_buf);
#endif

    tf_fp = fopen(path_buf, "w+");
    if (NULL == tf_fp)
    {
        HARDWARE_TEST("Can't open %s!\n\r", path_buf);
        goto err1;
    }

    strncpy(w_buf, tcard_str, strlen(tcard_str));
    fwrite(w_buf, strlen(w_buf), 1, tf_fp);
    fflush(tf_fp);
    /* 定位到文件开始位置*/
    fseek(tf_fp, 0, SEEK_SET);
    fread(r_buf, strlen(w_buf), 1, tf_fp);
    /* 删除读缓存的多余字节*/
    r_buf[strlen(w_buf)] = '\0';

    /* 比较写入的和读取的是否一致*/
   ret = strcmp(w_buf, r_buf);
    /* 写入的和读出的不一致*/
    if (0 != ret)
    {
        HARDWARE_TEST("Write and Read string are different!\n\r");
        goto err2;
    }

    fclose(tf_fp);
    HARDWARE_TEST("TCard_test Success!\n\r");
    return;

err2:
    fclose(tf_fp);
err1:
    HARDWARE_TEST("TCard_test Failed!\n\r");

    return;
}

/************************************************************************
* 函数名: EMMC_TEST 
* 功能描述:在EMMC的log分区建立文件，并判断写如何读出的字符是否一致
* 输入参数: None
*           
* 输出参数:None
* 返回值: 0: failed   1: success
*   
*
* 备注
*
*
*************************************************************************/
int emmc_test(void)
{
    FILE *emmc_fp = NULL;
    char w_buf[EMMC_TEST_BYTES] = {0};
    char r_buf[EMMC_TEST_BYTES] = {0};
    int ret = 0;

    HARDWARE_TEST("Begin EMMC test!\n\r");

    emmc_fp = fopen(EMMC_TEST_PATH, "w+");
    if (NULL == emmc_fp)
    {
        HARDWARE_TEST("Can't open %s!\n\r", EMMC_TEST_PATH);
        goto err1;
    }

    strncpy(w_buf, emmc_str, strlen(emmc_str));
    fwrite(w_buf, strlen(w_buf), 1, emmc_fp);
    fflush(emmc_fp);
    /* 定位到文件开始位置*/
    fseek(emmc_fp, 0, SEEK_SET);
    fread(r_buf, strlen(w_buf), 1, emmc_fp);
    /* 删除读缓存的多余字节*/
    r_buf[strlen(w_buf)] = '\0';

    /* 比较写入的和读取的是否一致*/
   ret = strcmp(w_buf, r_buf);
    /* 写入的和读出的不一致*/
    if (0 != ret)
    {
        HARDWARE_TEST("Write and Read string are different!\n\r");
        goto err2;
    }

    fclose(emmc_fp);
    HARDWARE_TEST("Begin EMMC test Success!\n\r");
    return 1;

err2:
    fclose(emmc_fp);
err1:
    HARDWARE_TEST("Begin EMMC test Failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: gpio_write_test 
* 功能描述:测试写GPIO管脚，手动拉高拉低控制
* 输入参数: None
*           
* 输出参数:None
* 返回值: None
*   
*
* 备注:测试时，输入参数格式为:gpio+空格+1(0)。
*              表示将对应GPIO管脚电压拉高或者拉低
*          输入参数格式可以为:gpio+空格+空格+空格+1(0)，
*              空格只能在1(0)之前，1(0)之后不能再有任何字符
*************************************************************************/
void gpio_write_test(void)
{
    int input = 0;  /* 记录GPIO的ACSII码转换成int型之后的值*/
    int val = 0;    /* 记录GPIO设置值的ACSII码转换成int型之后的值*/
    char rxbuf[GPIO_TEST_BYTES] = {0};
    char gpio_buf[GPIO_TEST_BYTES] = {0};
    int cunt = 0;
    int num = 0;
    int i = 0;
    int ret = 0;
    int space = 0;  /* 记录空格数量*/
    int right = 0;  /* 参数正确分支*/

    HARDWARE_TEST("Begin gpio_write_test Test!\n\r");

    /* 阻塞读取用户输入gpio管脚值*/
    while(1)
    {
        space = 0;
        right = 0;
        HARDWARE_TEST("Please input GPIO number and :0~255.\n\rQ(q): Return gpio test.\n\r");
        /* 获取GPIO number，如果输入是Q(q)则退出*/
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, GPIO_TEST_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出GPIO测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit GPIO write test!\n\r");
                break;
            }
        }

        /* 先处理输入的GPIO管脚号*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 空格字符*/
            else if ( rxbuf[i] == 0x20)
            {
                space ++;
                break;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input GPIO number ERROR!Please input again!\n\r");
                break;
            }
        }
        /* 处理前i个表示GPIO number的字符*/
        if (0 != space)
        {
            strncpy(gpio_buf, rxbuf, i);
        }
        else
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        cunt = i;

        i++;    /* 跳过输入的第一个空格键*/
        /* 处理剩下num-i个字符，仅表示GPIO拉高、拉低的参数1(0)*/
        for (; i < num; i++)
        {
            /* GPIO拉高、拉低状态只能是0或者1*/
            if ((rxbuf[i] == 0x30) || (rxbuf[i] == 0x31))
            {
                right++;
                break;
            }
            /* 空格字符*/
            else if ( rxbuf[i] == 0x20)
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                break;
            }
        }
        /* 判断GPIO拉高拉低的参数*/
        if ((i != (num-1)) || (0 == right))
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }

        /* 控制台读取到的，空格字符前的字符为GPIO number值*/
        ASCToInt(gpio_buf, &input, cunt);
        /* 控制台读取到的最后一个字符为GPIO拉高、拉低参数值*/
        ASCToInt(&rxbuf[num-1], &val, 1);

        HW_LOG("gpio_buf = %s, input = %d, rxbuf = %s, val = %d.\n\r", gpio_buf, input, rxbuf, val);
        if(input > AllGPIONUM)
        {
            HARDWARE_TEST("The GPIO number input: %d, out of range!\n\r", input);
            break;
        }
        /* 设置GPIO管脚为GPIO模式*/
        PIN_Mux_set(input, 2);
        /* 设置GPIO为输出*/
        ret = gpio_direction(input, GPIO_DIR_OUTPUT);
        if (FALSE == ret)
        {
            HARDWARE_TEST("parameter err!\n\r");
            continue;
        }

        if (0 == val)
        {
            ret = gpio_output_set(input, LOW_LEVEL);
            if (FALSE == ret)
            {
                HARDWARE_TEST("parameter err!\n\r");
                continue;
            }
            HARDWARE_TEST("GPIO%d output set Low level.\n\r", input);
        }
        else
        {
            ret = gpio_output_set(input, HIGH_LEVEL);
            if (FALSE == ret)
            {
                HARDWARE_TEST("parameter err!\n\r");
                continue;
            }
            HARDWARE_TEST("GPIO%d output set High level.\n\r", input);
        }
    }

    return;
}

/************************************************************************
* 函数名: gpio_read_test 
* 功能描述:测试读GPIO管脚
* 输入参数: None
*           
* 输出参数:None
* 返回值: None
*
* 备注:
*
*************************************************************************/
void gpio_read_test(void)
{
    int input = 0;
    char rxbuf[GPIO_TEST_BYTES] = {0};
    int num = 0;
    int val = 0;
    int i = 0;
    int ret = 0;

    HARDWARE_TEST("Begin gpio_read_test Test!\n\r");

    /* 阻塞读取用户输入gpio管脚值*/
    while(1)
    {
        HARDWARE_TEST("Please input GPIO number and :0~255.\n\rQ(q): Return gpio test.\n\r");
        /* 获取GPIO number，如果输入是Q(q)则退出*/
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, GPIO_TEST_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出GPIO测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit GPIO read test!\n\r");
                break;
            }
        }

        /* 判断输入的GPIO管脚号是否合法*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input GPIO number ERROR!Please input again!\n\r");
                break;
            }
        }

        if (i != num)
        {
            HARDWARE_TEST("Input the parameter Error!Please input again!\n\r");
            continue;
        }

        /* 控制台读取到的，空格字符前的字符为GPIO number值*/
        ASCToInt(rxbuf, &input, num);

        if(input > AllGPIONUM)
        {
            HARDWARE_TEST("The GPIO number input: %d, out of range!\n\r", input);
            break;
        }
        /* 设置GPIO管脚为GPIO模式*/
        PIN_Mux_set(input, 2);
        /* 设置GPIO为输入*/
        ret = gpio_direction(input, GPIO_DIR_INPUT);
        if (FALSE == ret)
        {
            HARDWARE_TEST("parameter err!\n\r");
            continue;
        }

        val = gpio_input_value(input);
        if (LOW_LEVEL == val)
        {
            HARDWARE_TEST("GPIO%d input Low level.\n\r", input);
        }
        else if (HIGH_LEVEL == val)
        {
            HARDWARE_TEST("GPIO%d input High level.\n\r", input);
        }
        else
        {
            HARDWARE_TEST("parameter err!\n\r");
            continue;
        }
    }

    return;
}

/************************************************************************
* 函数名: gpio_loopback_test 
* 功能描述:判断GPIO的连通性，看看是否为虚焊
* 输入参数: gpio_input:GPIO管脚号,gpio_output:GPIO管脚号
*           
* 输出参数:None
* 返回值: 0:success;   -1:failed;
*
* 备注:
*
*************************************************************************/
void gpio_loopback_test(void)
{
    int ret = -1;
    int flag1 = 0;
    int flag2 = 0;
    int gpio1 = UART2_CTS;
    int gpio2 = UART2_RTS;

    HARDWARE_TEST("Begin GPIO loopback Test!\n\r");

    ret = gpio_level_judge(gpio1, gpio2);
    if (0 == ret)
    {
        flag1 = 1;
        HARDWARE_TEST("GPIO%d and GPIO%d loopback Test Success!\n\r", gpio1, gpio2);
    }
    else
    {
        HARDWARE_TEST("GPIO%d and GPIO%d loopback Test Failed!\n\r", gpio1, gpio2);
    }

    ret = gpio_level_judge(gpio2, gpio1);
    if (0 == ret)
    {
        flag2 = 1;
        HARDWARE_TEST("GPIO%d and GPIO%d loopback Test Success!\n\r", gpio2, gpio1);
    }
    else
    {
        HARDWARE_TEST("GPIO%d and GPIO%d loopback Test Failed!\n\r", gpio2, gpio1);
    }

    if (flag1 && flag2)
    {
        HARDWARE_TEST("GPIO loopback Test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("GPIO loopback Test Failed!\n\r");
    }
    
    return;
}

/************************************************************************
* 函数名: i2c0_e2prom_init 
* 功能描述: 初始化i2c0，准备后续读写E2PROM
* 输入参数: 
* 输出参数: 无
* 返回值: 无 
* 备注: 设置i2c0管脚、时钟.
*
*************************************************************************/
void i2c0_e2prom_init(void)
{
    /* set gpio i2c0 function*/
    /* 0: i2c0_scl; 1:  保留; 2: gpio[213]*/
    PIN_Mux_set(I2C0_SCL, 0);
    PIN_Mux_set(I2C0_SDA, 0);

    i2cx_init(I2C_E2PROM_NUM, I2C_SS_SPEED);

    /* 设置并使能I2Cx工作时钟及速率*/
    get_i2cx_parent_rate();
    i2cx_clock_init_set_rate(I2C_E2PROM_NUM);
    i2cx_SetDevAddr(I2C_E2PROM_NUM, E2PROM_ADDR);
    usleep(10000);

    return;
}

/************************************************************************
* 函数名: i2c0_e2prom_test 
* 功能描述: 测试i2c0读写E2PROM的正确性
* 输入参数: 
* 输出参数: 无
* 返回值: 无 
* 备注:             E2PROM型号为CAT24C08
*
*************************************************************************/
void i2c0_e2prom_test(void)
{
    int ret = 0;
    int i = 0;
    char i2c_data;
    char page0_reg = 0xd;
    char page17_reg = 0x10;
    unsigned char val_e2prom[1024] = {0};

    HARDWARE_TEST("Begin i2c0_e2prom_test!\n\r");
    /* set gpio i2c0 function*/
    /* 0: i2c0_scl; 1:  保留; 2: gpio[213]*/
    PIN_Mux_set(I2C0_SCL, 0);
    PIN_Mux_set(I2C0_SDA, 0);

    i2cx_init(I2C_E2PROM_NUM, I2C_SS_SPEED);

    /* 设置并使能I2Cx工作时钟及速率*/
    get_i2cx_parent_rate();
    ret = i2cx_clock_init_set_rate(I2C_E2PROM_NUM);
    if (-1 == ret)
    {
        HARDWARE_TEST("I2C0 clock set failed.\n\r");
        goto err;
    }
    i2cx_SetDevAddr(I2C_E2PROM_NUM, E2PROM_ADDR);
    usleep(10000);

    e2prom_page_write(0, 0x55);
    e2prom_slct_read(0, 0xf, &i2c_data);
    HW_LOG("E2PROM. Read page0 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x55 != i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page0 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_page_write(16, 0xAA);
    e2prom_slct_read(16, 0xf, &i2c_data);
    HW_LOG("E2PROM. Read page16 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0xAA != i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page16 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_page_write(32, 0x99);
    e2prom_slct_read(32, 0xf, &i2c_data);
    HW_LOG("E2PROM. Read page32 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x99 != i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page32 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_page_write(48, 0x66);
    e2prom_slct_read(48, 0xf, &i2c_data);
    HW_LOG("E2PROM. Read page48 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x66 != i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page48 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

#if 0/* 这段代码没问题，完全正确，只是读写E2PROM有点多，运行时间长*/
    /* 往 地址reg写数据*/
    i2cx_WriteReg(I2C_E2PROM_NUM, page0_reg, 0x11);

    HW_LOG("Byte Write completed, Now read!\n\r");
    i2c_data = *I2Cx_CLR_INTR;

    /* 从地址reg读数据*/
    i2c_data = i2cx_ReadReg(I2C_E2PROM_NUM, page0_reg);
    HARDWARE_TEST("read page0 address 0x%x = 0x%x!\n\r", page0_reg, i2c_data);
    if (0x11 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read address:0x%x, value: 0x%x!\n\r", page0_reg, i2c_data);
        goto err;
    }

    e2prom_page_write(0, 0x55);
    e2prom_page_write(1, 0xAA);
    e2prom_page_write(10, 0x33);
    e2prom_page_write(11, 0xCC);
    e2prom_page_write(17, 0x11);
    e2prom_page_write(33, 0x22);
    e2prom_page_write(49, 0x33);
    HW_LOG("Page Write completed, Now read!\n\r");

    /* 该段代码为连续读E2PROM所有1024bytes大小内容，时间稍长，不便于测试*
      * 因此注释掉*/
    /* 页写过程中改变了I2C的从设备地址，需要设置回来*/
    i2cx_SetDevAddr(I2C_E2PROM_NUM, E2PROM_ADDR);
    e2prom_seq_read(val_e2prom);
    for (i=0; i<1024;i++)
    {
        i2c_data = val_e2prom[i];
        HW_LOG("E2PROM Read(I2C0) test, Read address:0x%x, value: 0x%x!\n\r", i, i2c_data);
    }

    i2c_data = *I2Cx_CLR_INTR;
    /* 页写过程中改变了I2C的从设备地址，需要设置回来*/
    i2cx_SetDevAddr(I2C_E2PROM_NUM, E2PROM_ADDR);
    i2c_data = i2cx_ReadReg(I2C_E2PROM_NUM, page0_reg);
    HARDWARE_TEST("E2PROM. Read page0 address:0x%x, value: 0x%x!\n\r", page0_reg, i2c_data);
    if (0x55 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read address:0x%x, value: 0x%x!\n\r", page0_reg, i2c_data);
        goto err;
    }

    e2prom_slct_read(0, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page0 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x55 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page0 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(1, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page1 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0xAA == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page1 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(10, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page10 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x33 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page10 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(11, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page11 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0xCC == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page11 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(17, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page17 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x11 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page17 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(33, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page33 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x22 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page33 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }

    e2prom_slct_read(49, 0xf, &i2c_data);
    HARDWARE_TEST("E2PROM. Read page49 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
    if (0x33 == i2c_data)
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("E2PROM Read(I2C0) test, Read page49 address:0x%x, value: 0x%x!\n\r", 0xf, i2c_data);
        goto err;
    }
#endif

    HARDWARE_TEST("e2prom i2c0 test Success!\n\r");

    return;

err:
    HARDWARE_TEST("e2prom i2c0 test Failed!\n\r");

    return;
}

/************************************************************************
* 函数名: sensor_test 
* 功能描述:I2C1总线上3个sensor芯片读写测试
* 输入参数: 
*           
* 输出参数:
* 返回值: 
*   
*
* 备注
*
*
*************************************************************************/
int i2c1_sensor_test(void)
{
    int ret = 0;
    unsigned char device_id = 0;

    HARDWARE_TEST("Begin i2c1 test!\n\r");

    /* SENSOR 上电  默认D1V8A D2V85A开机上电*/
    /* DLDO1 2.85V*/
//    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3e, 0xd6);
    /* BUCK6(DCDC6) 1.8v*/
//    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x0f, 0x5a);

    /* set i2c1 mode*/
    /* 0: i2c1_scl; 1:  保留; 2: gpio[207]*/
    PIN_Mux_set(I2C1_SCL, 0);
    PIN_Mux_set(I2C1_SDA, 0);

    /* 睡眠100ms，防止连续测试时I2C1总线还未准备好*/
    usleep(100000);

    /* 设置I2C1工作时钟及速率*/
    get_i2cx_parent_rate();
    ret = i2cx_clock_init_set_rate(I2C_SENSOR_NUM);
    if (-1 == ret)
    {
        HARDWARE_TEST("I2C1 clock set fail.\n\r");
        goto err;
    }

    /* GPIO默认就是I2C管脚*/
    i2cx_init(I2C_SENSOR_NUM, I2C_SS_SPEED);

    HARDWARE_TEST("Begin sensor MMC34160PJ test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, MMC34160PJ_address);
    usleep(1000);
    /* 读Device ID 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x20);
    
    /* 读取的数据相同测试成功*/
    if (0x06 == device_id)
    {
        HARDWARE_TEST("sensor MMC34160PJ test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor MMC34160PJ test Failed!\n\rRead sensor device ID:0x%x(real ID:0x06)", device_id);
        goto err;
    }
    usleep(1000);

    HARDWARE_TEST("Begin sensor L3GD20 test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, L3GD20_address);
    /* 读WHO_AM_I 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x0f);
    usleep(1000);
    
    //相同测试成功
    if((0xD4 == device_id) || (0xD7 == device_id))
    {
        HARDWARE_TEST("sensor L3GD20 test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor L3GD20 test Failed!\n\rRead sensor device ID:0x%x(real ID:0xD4 or 0xD7)", device_id);
        goto err;
    }
    usleep(1000);

    HARDWARE_TEST("Begin sensor MMA8653FCR1 test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, MMA8653FCR1_address);
    /* 读Device ID 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x0D);
    
    //相同测试成功
    if (0x5A == device_id)
    {
        HARDWARE_TEST("sensor MMA8653FCR1 test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor MMA8653FCR1 test Failed!\n\rRead sensor device ID:0x%x(real ID:0x5A)", device_id);
        goto err;
    }

    return 1;

err:
    HARDWARE_TEST("i2c1 test error!\n\r");

    return 0;
}

/************************************************************************
* 函数名: sensor_test 
* 功能描述:I2C1总线上3个sensor芯片读写测试
* 输入参数: 
*           
* 输出参数:
* 返回值: 
*   
*
* 备注
*
*
*************************************************************************/
void sensor_test(void)
{
    int input = 0;
    int choice = 0;
    int i = 0;
    int ret = 0;
    int num = 0;
    char rxbuf[MAX_KEY_BYTES];
    int sensor_item = 0;

    HARDWARE_TEST("Begin sensor test!\n\r");

    /* SENSOR 上电  默认D1V8A D2V85A开机上电*/
    /* DLDO1 2.85V*/
//    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3e, 0xd6);
    /* BUCK6(DCDC6) 1.8v*/
//    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x0f, 0x5a);

    /* set i2c1 mode*/
    /* 0: i2c1_scl; 1:  保留; 2: gpio[207]*/
    PIN_Mux_set(I2C1_SCL, 0);
    PIN_Mux_set(I2C1_SDA, 0);

    /* 设置I2C1工作时钟及速率*/
    get_i2cx_parent_rate();
    ret = i2cx_clock_init_set_rate(I2C_SENSOR_NUM);
    if (-1 == ret)
    {
        HARDWARE_TEST("I2C1 clock set fail.\n\r");
        goto err;
    }

    /* GPIO默认就是I2C管脚*/
    i2cx_init(I2C_SENSOR_NUM, I2C_SS_SPEED);
    sensor_item = GetListNum(SensorList);

    while(1)
    {
        DisplayList(SensorList);
        /* 等待控制台console输入*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出sensor测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit sensor test!\n\r");
                break;
            }
        }

        /* 其他字符键*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input sensor test number ERROR!Please input again!\n\r");
                break;
            }
        }

        /* 判断输入的sensor item参数*/
        if (i != num)
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        ASCToInt(rxbuf, &input, num);
        if(input > sensor_item)
        {
            HARDWARE_TEST("The sensor item number input: %d, out of range!Please input again!\n\r", i);
            continue;
        }
        
        /*按下正确测试项，进行测试*/
        choice = input - 1;
        HARDWARE_TEST("===========Item %d  %s : test begin-----Q(q):quit===========\n\r", input, SensorList[choice].pname);
        SensorList[choice].func();
        HARDWARE_TEST("=========================Item %d: test end=========================\n\r", input);
    }

    usleep(10);
err:
    HARDWARE_TEST("End sensor test!\n\r");

    return;
}

/************************************************************************
* 函数名: v2x_1pps_test 
* 功能描述:GPIO33(AP侧)、GPIO34(CP侧)两个管脚GPS信号输入，
*                           管脚电平变化表示有1pps信号输入。
*                           1s时间内，高电平100ms、低电平900ms，
*                           默认状态下GPIO33和GPIO34是低电平。
* 输入参数: 
*           
* 输出参数:
* 返回值: 
*   
*
* 备注
*
*
*************************************************************************/
int v2x_1pps_test(void)
{
    int count = 0;
    int ap_1pps = 33;
    int cp_1pps = 34;
    int val = 0;
    int high_flag = 0;
    int low_flag = 0;
    int ret = 0;
    int ap_flag = FALSE;
    int cp_flag = FALSE;

    HARDWARE_TEST("Begin v2x 1pps test!\n\r");

    /* GPIO33、GPIO34设置为GPIO输入模式*/
    PIN_Mux_set(ap_1pps, 2);
    PIN_Mux_set(cp_1pps, 2);
    gpio_direction(ap_1pps, GPIO_DIR_INPUT);
    gpio_direction(cp_1pps, GPIO_DIR_INPUT);

    count = 0;
    low_flag = 0;
    high_flag = 0;
    /* 判断GPIO33是否有1pps，默认GPIO33是低电平*/
    while (1)
    {
        val = gpio_input_value(ap_1pps);
        if (0 == val)
        {
            low_flag++;
            if (0 == low_flag % 50)
            {
                HARDWARE_TEST("GPIO%d input Low level, %d times.\n\r", ap_1pps, low_flag);
            }
        }
        else if (1 == val)
        {
            high_flag++;
            if (0 == high_flag % 50)
            {
                HARDWARE_TEST("GPIO%d input High level, %d times.\n\r", ap_1pps, high_flag);
            }
        }
        else
        {
            HARDWARE_TEST("parameter err!\n\r");
        }

        /* 3个以上高电平输出表示有1pps输入*/
        if (3 <= high_flag)
        {
            HARDWARE_TEST("GPIO%d have 1pps signal!test Success!\n\r", ap_1pps);
            ap_flag = TRUE;
            break;
        }
        /* 60ms读一次，一个pp1s脉冲内最多读2个高电平
             如果需要读到3次高电平，则至少有2次以上的pp1s脉冲信号*/
        usleep(60000);
        count++;
        /* 30s检测时间*/
        if ((500 <= count) && (3 > high_flag))
        {
            HARDWARE_TEST("GPIO%d have not 1pps signal!test Failed!\n\r", ap_1pps);
            ap_flag = FALSE;
            break;
        }
    }

    count = 0;
    low_flag = 0;
    high_flag = 0;
    /* 判断GPIO34是否有1pps，默认GPIO34是低电平*/
    while (1)
    {
        val = gpio_input_value(cp_1pps);
        if (0 == val)
        {
            low_flag++;
            if (0 == low_flag % 50)
            {
                HARDWARE_TEST("GPIO%d input Low level, %d times.\n\r", cp_1pps, low_flag);
            }
        }
        else if (1 == val)
        {
            high_flag++;
            if (0 == high_flag % 50)
            {
                HARDWARE_TEST("GPIO%d input High level, %d times.\n\r", cp_1pps, high_flag);
            }
        }
        else
        {
            HARDWARE_TEST("parameter err!\n\r");
        }

        /* 3个以上高电平输出表示有1pps输入*/
        if (3 <= high_flag)
        {
            HARDWARE_TEST("GPIO%d have 1pps signal!test Success!\n\r", cp_1pps);
            cp_flag = TRUE;
            break;
        }
        /* 60ms读一次，一个pp1s脉冲内最多读2个高电平
             如果需要读到3次高电平，则至少有2次以上的pp1s脉冲信号*/
        usleep(60000);
        count++;
        /* 30s检测时间*/
        if ((500 <= count) && (3 > high_flag))
        {
            HARDWARE_TEST("GPIO%d have not 1pps signal!test Failed!\n\r", cp_1pps);
            cp_flag = FALSE;
            break;
        }
    }

    if ((TRUE == ap_flag) && (TRUE == cp_flag))
    {
        ret = PPS_OK;
    }
    else if ((FALSE == ap_flag) && (TRUE == cp_flag))
    {
        /* GPIO33无PPS信号*/
        ret = PPS_GPIO33;
    }
    else if ((TRUE == ap_flag) && (FALSE == cp_flag))
    {
        /* GPIO34无PPS信号*/
        ret = PPS_GPIO34;
    }
    else
    {
        ret = PPS_ERR;
    }

    HARDWARE_TEST("End v2x 1pps test!\n\r");

    return ret;
}

/************************************************************************
* 函数名: usb_hsic_test 
* 功能描述:测试HSIC USB的联通性，通过lsusb判断是否有4604的设备
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int usb_hsic_test(void)
{
    int num = 0;
    char buf[200] = {0};
    char cmp_buf[] = "ID 0424:4604";
    FILE *fd_usb = NULL;
    int ret = 0;
    char *ptr = NULL;

    HARDWARE_TEST("Begin usb hsic test!\n\r");

    /* 执行lsusb命令，并将结果保存到文件中，查看是否有4604设备存在*/
    system("lsusb > lsusb.log");

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./lsusb.log", F_OK))
    {
        fd_usb = fopen("./lsusb.log", "r");
        if (NULL == fd_usb)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open lsusb.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_usb, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, 200, fd_usb);

        /* 从log文件中查找是否有4604设备*/
        ptr = strstr(buf, cmp_buf);
        if (NULL == ptr)
        {
            /* 未找到4604设备*/
            HARDWARE_TEST("Can't find usb hub device on lsusb.log file!\n\r");
            goto err1;
        }
        HW_LOG("lsusb:\n\r%s", buf);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("usb hsic test Success!\n\r");
    fclose(fd_usb);
    system("rm -rf ./lsusb.log");
    return 1;

err1:
    fclose(fd_usb);
    system("rm -rf ./lsusb.log");
err2:
    HARDWARE_TEST("usb hsic test failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: usb_host_test 
* 功能描述:测试USB做HOST的联通性，通过lsusb判断是否有0bda:8152的设备
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int usb_host_test(void)
{
    int  num = 0;
    char buf[200] = {0};
    char cmp_buf[] = "ID 0424:2530";    /*"Bus 002 Device 005: ID 0bda:8152"*/
    FILE *fd_usb = NULL;
    int  ret = 0;
    char *ptr = NULL;
	
	char buf_vendor[LSUSB_INFO_SIZE] = {0};
	char buf_product[LSUSB_INFO_SIZE] = {0};
	char *cmp_product = "2530";
	char *cmp_vendor = "0424";
	char *id_product = "/sys/bus/usb/drivers/usb/2-1.5/idProduct";
	char *id_vendor = "/sys/bus/usb/drivers/usb/2-1.5/idVendor";
    FILE *fd_product = NULL;
	FILE *fd_vendor = NULL;
	int ret_product = 0;
	int ret_vendor = 0;

    HARDWARE_TEST("Begin usb host test!\n");
#if ZELAI_TEST
    /* 执行lsusb命令，并将结果保存到文件中，查看是否有0bda:8152设备存在*/
    system("lsusb > lsusb.log");

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./lsusb.log", F_OK))
    {
        fd_usb = fopen("./lsusb.log", "r");
        if (NULL == fd_usb)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open lsusb.log file!\n");
            goto err2;
        }

        ret = fseek(fd_usb, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, 200, fd_usb);

        /* 从log文件中查找是否有0bda:8152设备*/
        ptr = strstr(buf, cmp_buf);
        if (NULL == ptr)
        {
            /* 未找到0bda:8152设备*/
            HARDWARE_TEST("Can't find usb host device!\n");
            goto err1;
        }
        HW_LOG("lsusb:\n\r%s\n", buf);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("usb host test Success!\n");
    fclose(fd_usb);
    system("rm -rf ./lsusb.log");
    return 1;

err1:
	fclose(fd_usb);
    system("rm -rf ./lsusb.log");
err2:
    HARDWARE_TEST("usb host test failed!\n\r");
    return 0;
#endif
#if CTAO_TEST
	/* at the moment, from hardware design, identifify the usb port from hub4604 be 2-1.5
		see /sys/bus/usb/drivers/usb/2-1.5.
		but, i think this is not the most practical approach,
		I will improve.
	*/
	if ((0==access(id_product, F_OK))&&(0==access(id_vendor, F_OK)))
	{
		fd_product = fopen(id_product, "r");
		fd_vendor = fopen(id_vendor, "r");
		if ((NULL == fd_product) || (NULL == fd_vendor))
        {
            /* 不能打开log文件*/
            HARDWARE_TEST(" Can't open id_product or id_vendor!\nplease check /sys/bus/usb/drivers/usb/2-1.5 have idProduct and idProduct\n");
            goto fopen_err;
        }
#if 0
		ret_product = fseek(fd_product, 0, SEEK_SET);
		ret_vendor = fseek(fd_vendor, 0, SEEK_SET);
        if ((0 != ret_product) || (0 != ret_vendor))
        {
            /* 移动指针位置失败*/
            goto operate_err;
        }
#endif
		fread(buf_product, 1, LSUSB_INFO_SIZE, fd_product);
		fread(buf_vendor, 1, LSUSB_INFO_SIZE, fd_vendor);
		HARDWARE_TEST("product: %s vendor: %s", buf_product, buf_vendor);
		/* 从log文件中查找是否有0bda:8152设备*/
        if((0 == strcmp(buf_product, cmp_product))||(0 == strcmp(buf_vendor, cmp_product)))
        {
            /* 未找到0bda:8152设备*/
            HARDWARE_TEST("Can't find usb product!\n");
            goto operate_err;
        }else
        	HARDWARE_TEST("we got a usb host device!\nproduct: %s vendor: %s", buf_product, buf_vendor);
		return 1;
	}else{
		goto fopen_err;
	}
	HARDWARE_TEST("usb host test Success!\n");
    fclose(buf_product);
	fclose(fd_vendor);
    return 1;

operate_err:
	fclose(fd_product);
	fclose(fd_vendor);
fopen_err:
	HARDWARE_TEST("usb host test failed!\n\r");
	return 0;
#endif
}

/************************************************************************
* 函数名: usb_lan9500_test 
* 功能描述:测试LAN9e00的联通性，通过lsusb判断是否有LAN9e00的设备
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int usb_LAN_test(char *buffer)
{
    int num = 0;
    char buf[200] = {0};
    char cmp_buf[] = "ID 0424:9e00";
    FILE *fd_usb = NULL;
    int ret = 0;
    char *ptr = NULL;

    HARDWARE_TEST("Begin LAN port test!\n\r");

    /* 执行lsusb命令，并将结果保存到文件中，查看是否有lan9e00设备存在*/
    system("lsusb > lsusb.log");

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./lsusb.log", F_OK))
    {
        fd_usb = fopen("./lsusb.log", "r");
        if (NULL == fd_usb)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open lsusb.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_usb, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, 200, fd_usb);

        /* 从log文件中查找是否有lan9e00设备*/
        ptr = strstr(buf, buffer);
        if (NULL == ptr)
        {
            /* 未找到lan9e00设备*/
            HARDWARE_TEST("Can't find lan9500 device on lsusb.log file!\n\r");
            goto err1;
        }
        HW_LOG("lsusb:\n\r%s", buf);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("lan9500 test Success!\n\r");
    fclose(fd_usb);
    system("rm -rf ./lsusb.log");
    return 1;

err1:
    fclose(fd_usb);
    system("rm -rf ./lsusb.log");
err2:
    HARDWARE_TEST("lan9500 test failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: usb_otg_test 
* 功能描述:测试USB OTG接口的联通性，通过USB转网口ping判断OTG是否正确
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int usb_otg_test(void)
{
#if ZELAI_TEST
    int num = 0;
    char buf[BUFF_LEN] = {0};
    char cmp_buf[] = "64 bytes from 192.168.62.224";
    FILE *fd_ping = NULL;
    int ret = 0;
    char *ptr = NULL;

    HARDWARE_TEST("Begin usb otg test!\n\r");

    /* 执行ping命令，发送3个包结束，通过192.168.62.199网卡
      * 并将结果保存到文件中，查看是否有ping回包*/
    system("ping -c 3 -I 192.168.62.199 192.168.62.224 > ping.log");
    sleep(3);

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./ping.log", F_OK))
    {
        fd_ping = fopen("./ping.log", "r");
        if (NULL == fd_ping)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open ping.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_ping, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, BUFF_LEN, fd_ping);

        /* 从log文件中查找是否有回包*/
        ptr = strstr(buf, cmp_buf);
        if (NULL == ptr)
        {
            HARDWARE_TEST("Can't find result of ping 192.168.62.224 command!\n\r");
            goto err1;
        }
        HW_LOG("ping:\n\r%s", buf);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("usb otg test Success!\n\r");
    fclose(fd_ping);
    system("rm -rf ./ping.log");
    return 1;

err1:
    fclose(fd_ping);
    system("rm -rf ./ping.log");
err2:
    HARDWARE_TEST("usb otg test failed!\n\r");

    return 0;
#endif

#if CTAO_TEST
	#define PING_TIMES 4

	HARDWARE_TEST("Begin usb otg test!\n\r");
	int i;
	int ping_resault[PING_TIMES];
	for(i=0; i< PING_TIMES; i++)
	{
		ping_resault[i] = system("ping -c 1 -I 192.168.62.199 192.168.62.224");
		HARDWARE_TEST("times %d, ping resault: %d\n", i, ping_resault[i]);
		if(0 == ping_resault[i]){
			HARDWARE_TEST("usb otg test Success!\n");
			return 1;
			}else{
			HARDWARE_TEST("ping test: times %d!, %d times failed\n\r", PING_TIMES, i);
			}
		HARDWARE_TEST("usb otg test failed!\n\r");
		return 0;
	}
#endif
return 0;
}

/************************************************************************
* 函数名: usb_network_eth_test 
* 功能描述:测试设备以太接插件联通性，通过ping外部主机地址判断ETH口连通性
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int usb_network_eth_test(void)
{
    int num = 0;
    char buf[BUFF_LEN] = {0};
    char cmp_buf[] = "64 bytes from 192.168.20.224";
    FILE *fd_ping = NULL;
    int ret = 0;
    char *ptr = NULL;

    HARDWARE_TEST("Begin network ETH test!\n\r");

    /* 执行ping命令，发送3个包结束，通过192.168.20.199网卡
      * 并将ping结果保存到文件中，查看是否有ping回包*/
    system("ping -c 3 -I 192.168.20.199 192.168.20.224 > ping.log");
    sleep(3);

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./ping.log", F_OK))
    {
        fd_ping = fopen("./ping.log", "r");
        if (NULL == fd_ping)
        {
            /* 不能打开log文件*/
            HARDWARE_TEST("Can't open ping.log file!\n\r");
            goto err2;
        }

        ret = fseek(fd_ping, 0, SEEK_SET);
        if (0 != ret)
        {
            /* 移动指针位置失败*/
            goto err1;
        }

        fread(buf, 1, BUFF_LEN, fd_ping);

        /* 从log文件中查找是否有回包*/
        ptr = strstr(buf, cmp_buf);
        if (NULL == ptr)
        {
            HARDWARE_TEST("Can't find result of ping 192.168.20.224 command!\n\r");
            goto err1;
        }
        HW_LOG("ping:\n\r%s", buf);
    }
    else
    {
        goto err2;
    }
    HARDWARE_TEST("network ETH test Success!\n\r");
    fclose(fd_ping);
    system("rm -rf ./ping.log");
    return 1;

err1:
    fclose(fd_ping);
    system("rm -rf ./ping.log");
err2:
    HARDWARE_TEST("network ETH test failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: module_wifi_test 
* 功能描述:测试DMD31模组上的BCM4343S芯片的WIFI模块数字接口的连通性
*                           通过设置WIFI模块连接外部的热点，并ping通网关判断
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注
*
*
*************************************************************************/
int module_wifi_test_wlan0(void)
{
    int num = 0;
    char buf[BUFF_LEN] = {0};
    char wlan0_buf[3072] = {0};
    char tmp_buf[100] = {0};
    char pid_buf[100] = {0};
    char cmp_buf[] = "64 bytes from 220.181.38.149";
    FILE *fd_read = NULL;
    FILE *fd_ping = NULL;
    int ret = 0;
    char *ptr = NULL;
    char *Begb = NULL;
    char *Endb = NULL;

    HARDWARE_TEST("%d:Begin module wifi test!\n\r", getpid());

    /*************************start juage network state*****************************/
    /* judge wifi up or not */
    system("ip addr show > module_wifi.log");
    if (0 == access("./module_wifi.log", F_OK))
    {
        fd_read = fopen("./module_wifi.log", "r");
        if (NULL == fd_read)
        {
            HARDWARE_TEST("%d:Can't open module_wifi.log file!\n\r", getpid());
            goto err1;
        }

        ret = fseek(fd_read, 0, SEEK_SET);
        if (0 != ret)
        {
            goto err1;
        }

        fread(wlan0_buf, 1, 3072, fd_read);
        fclose(fd_read);
        system("rm -rf ./module_wifi.log");

        /* find network up info from log */
        Begb = wlan0_buf;
        Endb = strstr(Begb, "wlan0:");
        if (Endb == NULL)
        {
            HARDWARE_TEST("%d:Can't find wlan0 device!\n\r", getpid());
            goto err1;
        }
        /* look up wlan0 info */
        Begb = Endb;
        Endb = strchr(Begb, '>');
        num = Endb+1-Begb;
        strncpy(tmp_buf, Begb, num);
        HARDWARE_TEST("%d:%s\n\r", getpid(), tmp_buf);

        ptr = strstr(tmp_buf, "UP");
        if (NULL == ptr)
        {
            /* wlan0 is up */
            HARDWARE_TEST("%d:wlan0 not up!\n\r", getpid());
            wifi_flag = 0;
        }
        else
        {
            HARDWARE_TEST("%d:wlan0 have up!\n\r", getpid());
            wifi_flag = 1;
        }
        HARDWARE_TEST("%d:wifi_flag:%d\n\r", getpid(), wifi_flag);
    }
    else
    {
        goto err1;
    }
    /*************************end juage network state*****************************/

    HARDWARE_TEST("%d:module wifi test 111...\n\r", getpid());
    /* setup wifi ap ssid(YFtest) and psk(new.1234) which we will link*/
    system("sed -i -r \'s\/(ssid=)\".*\".*\/\\1\"\'\"YFtest\"\'\"\/g\' /etc/wpa_supplicant.conf");
    system("sed -i -r \'s\/(psk=)\".*\".*\/\\1\"\'\"new.1234\"\'\"\/g\' /etc/wpa_supplicant.conf");

	/* Default: wifi in AP mode;
	 * not matter what mode, will down wlan and flush for ip info;
	 * kill the AP and STATION process;
	 * start STATION process;
	 * after test, replace AP process;
	 * */
    //if ((1!=wifi_flag))
    {
        HARDWARE_TEST("%d:module wifi test 222...\n\r", getpid());
        system("echo 0 > /sys/module/bcmdhd/parameters/op_mode");
        usleep(1000);
        system("ip link set wlan0 down");
        system("ip addr flush dev wlan0");
        usleep(1000);
        system("echo 'kill -9 $(pidof hostapd)' > /system/kill_ap.sh");
        system("echo 'kill -9 $(pidof udhcpd)' >> /system/kill_ap.sh");
        system("echo 'kill -9 $(pidof wpa_supplicant)' >> /system/kill_ap.sh");
        system("chmod +x /system/kill_ap.sh");
        system("sh /system/kill_name.sh");
        /*
          * fork, child used for setup wpa_supplicant sh; 
          */
        pid_t fpid = fork();
        if (fpid < 0)
        {
            HARDWARE_TEST("Can't fork new pid!\n\r");
        }
        else if (fpid == 0)
        {
            HARDWARE_TEST("%d:module wifi test 333...\n\r", getpid());
            system("/system/bin/cfg_wifi_mac.sh wlan0");
            HARDWARE_TEST("%d:module wifi test 444...\n\r", getpid());
            usleep(1000);
            HARDWARE_TEST("%d:module wifi test 555...\n\r", getpid());
            system("wpa_supplicant -iwlan0 -c /etc/wpa_supplicant.conf &");
            usleep(1000);
            HARDWARE_TEST("%d:module wifi test 666...\n\r", getpid());
            system("udhcpc -i wlan0 -q");
            usleep(1000);
            HARDWARE_TEST("%d:child process done.\n\r", getpid());

            /* child exit
              */
            exit(0);
            HARDWARE_TEST("%d:kill child process.\n\r", getpid());
        }
        else
        {
            HARDWARE_TEST("%d:module wifi test 777...\n\r", getpid());
            sleep(1);
            waitpid(-1, NULL, 0);
            HARDWARE_TEST("%d:parent process continue.\n\r", getpid());
        }
    }

    int i;
    int ping_resault[4];
    for(i=0; i< 4; i++)
    {
        ping_resault[i] = system("ping -I wlan0 -c 3 220.181.38.149");
        HARDWARE_TEST("times %d, ping resault: %d\n", i, ping_resault[i]);
        if(0 == ping_resault[i])
        {
            HARDWARE_TEST("module wifi test Success!\n");
            /* replace ap mode */
            HARDWARE_TEST("module wifi switch ap_mode 111!\n");
            system("echo 2 > /sys/module/bcmdhd/parameters/op_mode");
            system("ip link set wlan0 down");
            system("ip addr flush dev wlan0");

            HARDWARE_TEST("module wifi switch ap_mode 222!\n");
            usleep(1000);
            system("echo 'kill -9 $(pidof hostapd)' > /system/kill_station.sh");
            system("echo 'kill -9 $(pidof udhcpd)' >> /system/kill_station.sh");
            system("echo 'kill -9 $(pidof wpa_supplicant)' >> /system/kill_station.sh");
            system("chmod +x /system/kill_station.sh");
            system("sh /system/kill_station.sh");

            HARDWARE_TEST("module wifi switch ap_mode 333!\n");
            system("ip addr add dev wlan0 192.168.1.2/24");

            HARDWARE_TEST("module wifi switch ap_mode 444!\n");
            system("/system/bin/cfg_wifi_mac.sh wlan0");

            HARDWARE_TEST("module wifi switch ap_mode 555!\n");
            usleep(1000);
            system("hostapd -B /etc/hostapd.conf");

            HARDWARE_TEST("module wifi switch ap_mode 666!\n");
            usleep(1000);
            system("udhcpd /etc/udhcpd.conf &");
            return 1;
        }
        else
        {
            HARDWARE_TEST("ping test: times %d!, %d times failed\n\r", 4, i);
            HARDWARE_TEST("module wifi test failed!\n\r");
            return 0;
        }
    }

err2:
    fclose(fd_ping);
    system("rm -rf ./module_wifi.log");
err1:
    HARDWARE_TEST("%d:module wifi test failed!\n\r", getpid());
    return 0;
}

/************************************************************************
* º¯ÊýÃû: module_wifi_test 
* ¹¦ÄÜÃèÊö:²âÊÔDMD31Ä£×éÉÏµÄBCM4343SÐ¾Æ¬µÄWIFIÄ£¿éÊý×Ö½Ó¿ÚµÄÁ¬Í¨ÐÔ
*                           Í¨¹ýÉèÖÃWIFIÄ£¿éÁ¬½ÓÍâ²¿µÄÈÈµã£¬²¢pingÍ¨Íø¹ØÅÐ¶Ï
* ÊäÈë²ÎÊý: 
*           
* Êä³ö²ÎÊý:None
* ·µ»ØÖµ: 0: false   1: success
*   
*
* ±¸×¢
*
*
*************************************************************************/
int module_wifi_test_mlan0(void)
{
    int num = 0;
    char buf[BUFF_LEN] = {0};
    char mlan0_buf[3072] = {0};
    char tmp_buf[100] = {0};
    char pid_buf[100] = {0};
    char cmp_buf[] = "64 bytes from 220.181.38.149";        /* YFtest�ȵ�IP��ַ192.168.0.1*/
    FILE *fd_read = NULL;
    FILE *fd_ping = NULL;
    int ret = 0;
    char *ptr = NULL;
    char *Begb = NULL;
    char *Endb = NULL;

    HARDWARE_TEST("%d:Begin module wifi test!\n\r", getpid());

    /*************************start juage network state*****************************/
    /* �ж�mlan0�Ƿ��Ѿ�up���*/
    system("ip addr show > module_wifi.log");
    /* �ж��ļ��Ƿ���ڣ�����ȡ�ļ�����*/
    if (0 == access("./module_wifi.log", F_OK))
    {
        fd_read = fopen("./module_wifi.log", "r");
        if (NULL == fd_read)
        {
            /* ���ܴ�log�ļ�*/
            HARDWARE_TEST("%d:Can't open module_wifi.log file!\n\r", getpid());
            goto err1;
        }

        ret = fseek(fd_read, 0, SEEK_SET);
        if (0 != ret)
        {
            /* �ƶ�ָ��λ��ʧ��*/
            goto err1;
        }

        fread(mlan0_buf, 1, 3072, fd_read);
        fclose(fd_read);
        system("rm -rf ./module_wifi.log");

        /* ��log�ļ��в���mlan0�����Ƿ��Ѿ�up*/
        Begb = mlan0_buf;
        Endb = strstr(Begb, "mlan0:");
        if (Endb == NULL)
        {
            HARDWARE_TEST("%d:Can't find mlan0 device!\n\r", getpid());
            goto err1;
        }
        /* �ҵ�waln0������λ��*/
        Begb = Endb;
        Endb = strchr(Begb, '>');
        num = Endb+1-Begb;
        strncpy(tmp_buf, Begb, num);
        HARDWARE_TEST("%d:%s\n\r", getpid(), tmp_buf);

        ptr = strstr(tmp_buf, "UP");
        if (NULL == ptr)
        {
            /* mlan0������δup*/
            HARDWARE_TEST("%d:mlan0 not up!\n\r", getpid());
            wifi_flag = 0;
        }
        else
        {
            /* mlan0�����Ѿ�up*/
            HARDWARE_TEST("%d:mlan0 have up!\n\r", getpid());
            wifi_flag = 1;
        }
        HARDWARE_TEST("%d:wifi_flag:%d\n\r", getpid(), wifi_flag);
    }
    else
    {
        goto err1;
    }
    /*************************end juage network state*****************************/

    HARDWARE_TEST("%d:module wifi test 111...\n\r", getpid());
    /* ������Ҫ���ӵ��ȵ���:YFtest ������:new.1234 */
    system("sed -i -r \'s\/(ssid=)\".*\".*\/\\1\"\'\"YFtest\"\'\"\/g\' /etc/wpa_supplicant.conf");
    system("sed -i -r \'s\/(psk=)\".*\".*\/\\1\"\'\"new.1234\"\'\"\/g\' /etc/wpa_supplicant.conf");

    /* ѭ������ʱ���ж�mlan0�Ƿ��Ѿ�up������up����ִ��mlan0��ʼ������*/
    //if ((1!=wifi_flag))
    {
        HARDWARE_TEST("%d:module wifi test 222...\n\r", getpid());
        system("echo 0 > /sys/module/bcmdhd/parameters/op_mode");
        usleep(1000);
        system("ip link set mlan0 down");
        system("ip addr flush dev mlan0");
        usleep(1000);
        system("echo 'kill -9 $(pidof hostapd)' > /system/kill_ap.sh");
        system("echo 'kill -9 $(pidof udhcpd)' >> /system/kill_ap.sh");
        system("echo 'kill -9 $(pidof wpa_supplicant)' >> /system/kill_ap.sh");
        system("chmod +x /system/kill_ap.sh");
        system("sh /system/kill_name.sh");
        /*
          * ��fork����ִ����Ϻ���������½��̳ɹ���������������̣�
          * һ�����ӽ��̣�һ���Ǹ����̡����ӽ����У�fork��������0��
          * �ڸ������У�fork�����´����ӽ��̵Ľ���ID��
          */
        pid_t fpid = fork();
        if (fpid < 0)
        {
            HARDWARE_TEST("Can't fork new pid!\n\r");
        }
        /* fork�����ӽ��̣��ӽ��̽���ִ��wpa_supplicant��ִ�����kill�˳�*/
        else if (fpid == 0)
        {
            HARDWARE_TEST("%d:module wifi test 333...\n\r", getpid());
            system("/system/bin/cfg_wifi_mac.sh mlan0");
            HARDWARE_TEST("%d:module wifi test 444...\n\r", getpid());
            usleep(1000);
            HARDWARE_TEST("%d:module wifi test 555...\n\r", getpid());
            system("wpa_supplicant -imlan0 -c /etc/wpa_supplicant.conf &");
            usleep(1000);
            HARDWARE_TEST("%d:module wifi test 666...\n\r", getpid());
            system("udhcpc -i mlan0 -q");
            usleep(1000);
            HARDWARE_TEST("%d:child process done.\n\r", getpid());

            /* ����system�����������������ӽ���������wifi�ű�����kill���ӽ��̣�
              * ����������dtvl-pltest���̡�
              */
            #if 0
            sprintf(pid_buf, "kill -9 %d", getpid());
            system(pid_buf);
            #endif
            exit(0);
            /* �þ䲻ִ�У��ӽ����Ѿ���kill���Ժ���򶼲�ִ��*/
            HARDWARE_TEST("%d:kill child process.\n\r", getpid());
        }
        /* �����̴��ӽ���ִ��������wifi�ȵ�󣬼�������ִ��*/
        else
        {
            HARDWARE_TEST("%d:module wifi test 777...\n\r", getpid());
            /* sleep���ø������ͷ�CPU���Ӷ���fork�������ӽ��̻�ȡcpuʱ�䣬ִ���ӽ���*/
            sleep(1);
            waitpid(-1, NULL, 0);
            HARDWARE_TEST("%d:parent process continue.\n\r", getpid());
        }
    }

    int i;
    int ping_resault[4];
    for(i=0; i< 4; i++)
    {
        ping_resault[i] = system("ping -I mlan0 -c 3 220.181.38.149");
        HARDWARE_TEST("times %d, ping resault: %d\n", i, ping_resault[i]);
        if(0 == ping_resault[i])
        {
            HARDWARE_TEST("module wifi test Success!\n");
            /* replace ap mode */
            HARDWARE_TEST("module wifi switch ap_mode 111!\n");
            system("echo 2 > /sys/module/bcmdhd/parameters/op_mode");
            system("ip link set mlan0 down");
            system("ip addr flush dev mlan0");

            HARDWARE_TEST("module wifi switch ap_mode 222!\n");
            usleep(1000);
            system("echo 'kill -9 $(pidof hostapd)' > /system/kill_station.sh");
            system("echo 'kill -9 $(pidof udhcpd)' >> /system/kill_station.sh");
            system("echo 'kill -9 $(pidof wpa_supplicant)' >> /system/kill_station.sh");
            system("chmod +x /system/kill_station.sh");
            system("sh /system/kill_station.sh");

            HARDWARE_TEST("module wifi switch ap_mode 333!\n");
            system("ip addr add dev mlan0 192.168.1.2/24");

            HARDWARE_TEST("module wifi switch ap_mode 444!\n");
            system("/system/bin/cfg_wifi_mac.sh mlan0");

            HARDWARE_TEST("module wifi switch ap_mode 555!\n");
            usleep(1000);
            system("hostapd -B /etc/hostapd.conf");

            HARDWARE_TEST("module wifi switch ap_mode 666!\n");
            usleep(1000);
            system("udhcpd /etc/udhcpd.conf &");
            return 1;
        }
        else
        {
            HARDWARE_TEST("ping test: times %d!, %d times failed\n\r", 4, i);
            HARDWARE_TEST("module wifi test failed!\n\r");
            return 0;
        }
    }

err2:
    fclose(fd_ping);
    system("rm -rf ./module_wifi.log");
err1:
    HARDWARE_TEST("%d:module wifi test failed!\n\r", getpid());
    return 0;
}


/************************************************************************
* 函数名: public_network_test 
* 功能描述:测试公网联通性，通过ping百度地址判断是否有回包
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注: 使用有方的公网模组，"ip addr show"会显示'ppp0'的一个网卡名称
*
*
*************************************************************************/
int public_network_test(void)
{
    int num = 0;
    char buf[BUFF_LEN] = {0};
    char cmp_buf[] = "64 bytes from 220.181.38.149";        /* 百度IP地址*/
    FILE *fd_ppp0 = NULL;
    FILE *fd_usb1 = NULL;
    int ret = 0;
    char *ptr = NULL;

    HARDWARE_TEST("Begin public network test!\n\r");
    /* 删除保存ping结果的log文件*/
    system("rm -rf ./ppp0.log");
    system("rm -rf ./usb1.log");
    memset(buf, 0, sizeof(buf));

    if (0xff == hwver)
    {
        /* ADC连续多次不间断读取会出现读取不到数据的情况*/
        hwver = io_board_hardware_ver_test();
    }
    usleep(3000);

    /* 执行ping外网命令，并将结果保存到文件中*
      * RSU3110有方的4G公网模组N720，网卡名称:'ppp0'*
      * VU3004有方的4G公网模组N720，网卡名称:'ppp0'*
      * VU3005广和通的5G公网模组FM150，网卡名称:'usb1'*
      * NR-V2X广和通的5G公网模组FM150，网卡名称:'usb1'*
      * DTVL5000-OBU中兴的5G公网模组ZM9000，网卡名称:'usb1'*
      */
    switch (hwver)
    {
        case DTVL3110:
        {
            system("ping -I ppp0 -c 3 220.181.38.149 > ppp0.log");
            system("ping -I usb1 -c 3 220.181.38.149 > usb1.log");
            sleep(3);
            break;
        }
        case VU300X:
        {
            system("ping -I ppp0 -c 3 220.181.38.149 > ppp0.log");
            system("ping -I usb1 -c 3 220.181.38.149 > usb1.log");
            sleep(3);
            break;
        }
#if 0
        case NR-V2X:
        {
            break;
        }
        case DTVL5000-OBU:
        {
            break;
        }
#endif
        case UNKNOWN:
        {
            system("ping -I ppp0 -c 3 220.181.38.149 > ppp0.log");
            system("ping -I usb1 -c 3 220.181.38.149 > usb1.log");
            sleep(3);
            break;
        }
        default:
        {
            system("ping -I ppp0 -c 3 220.181.38.149 > ppp0.log");
            system("ping -I usb1 -c 3 220.181.38.149 > usb1.log");
            sleep(3);
            HARDWARE_TEST("Device type error!\n");
            break;
        }
    }

    /* 判断文件是否存在，并读取文件内容*/
    if (0 == access("./ppp0.log", F_OK))
    {
        fd_ppp0 = fopen("./ppp0.log", "r");
        if (NULL != fd_ppp0)
        {
            /* 打开log文件*/
            ret = fseek(fd_ppp0, 0, SEEK_SET);
            if (0 == ret)
            {
                /* 移动指针到指定位置*/
                fread(buf, 1, BUFF_LEN, fd_ppp0);

                /* 从log文件中查找是否有百度回包*/
                ptr = strstr(buf, cmp_buf);
                if (NULL == ptr)
                {
                    /* 未找到百度回包字段*/
                    HARDWARE_TEST("Can't find result of ppp0 ping 220.181.38.149(baidu) command!\n\r");
                }
                else
                {
                    /* 找到百度回包字段，测试成功*/
                    HW_LOG("ppp0 ping:\n\r%s", buf);
                    goto ok;
                }
            }
        }
    }

    memset(buf, 0, sizeof(buf));
    if (0 == access("./usb1.log", F_OK))
    {
        fd_usb1 = fopen("./usb1.log", "r");
        if (NULL != fd_usb1)
        {
            /* 打开log文件*/
            ret = fseek(fd_usb1, 0, SEEK_SET);
            if (0 == ret)
            {
                /* 移动指针到指定位置*/
                fread(buf, 1, BUFF_LEN, fd_usb1);

                /* 从log文件中查找是否有百度回包*/
                ptr = strstr(buf, cmp_buf);
                if (NULL == ptr)
                {
                    /* 未找到百度回包字段*/
                    HARDWARE_TEST("Can't find result of usb1 ping 220.181.38.149(baidu) command!\n\r");
                }
                else
                {
                    /* 找到百度回包字段，测试成功*/
                    HW_LOG("usb1 ping:\n\r%s", buf);
                    goto ok;
                }
            }
        }
    }

    if ((0 != access("./ppp0.log", F_OK)) && (0 != access("./usb1.log", F_OK)))
    {
        /* 没找到ppp0.log及usb1.log文件*/
        goto err2;
    }
    else
    {
        /* 上网卡不能上网*/
        goto err1;
    }

ok:
    if (NULL != fd_ppp0)
    {
        fclose(fd_ppp0);
    }

    if (NULL != fd_usb1)
    {
        fclose(fd_usb1);
    }

    if (0 == access("./ppp0.log", F_OK))
    {
        system("rm -rf ./ppp0.log");
    }

    if (0 == access("./usb1.log", F_OK))
    {
        system("rm -rf ./usb1.log");
    }

    HARDWARE_TEST("public network test Success!\n\r");
    return 1;

err1:
    if (NULL != fd_ppp0)
    {
        fclose(fd_ppp0);
    }

    if (NULL != fd_usb1)
    {
        fclose(fd_usb1);
    }

    if (0 == access("./ppp0.log", F_OK))
    {
        system("rm -rf ./ppp0.log");
    }

    if (0 == access("./usb1.log", F_OK))
    {
        system("rm -rf ./usb1.log");
    }
err2:
    HARDWARE_TEST("public network test failed!\n\r");

    return 0;
}

#define CTAO 0
#define AT_FLAGR_ESET 1
#define ATCPIN 1

int   fd_tty = -1, port = 0;
static int flag = 0, sign = 0, at_result = 0;
static pthread_mutex_t mutex;
FILE *at_fp = NULL;
#define LOG_PATH "/var/log/ecm.log"

/*
 * atcmd_log_open: record log og atcmd;
 * hongfei's method;
 */
int atcmd_log_open()
{
    unsigned int size = 0;

    if((at_fp = fopen(LOG_PATH, "a+")) == NULL){
        return -1;
    };
    fseek(at_fp, 0L, SEEK_END);
    size = ftell(at_fp);

    if(size >= 50*1024){
        fclose(at_fp);
        if((at_fp = fopen(LOG_PATH, "w+")) == NULL){
            return -1;
        };
    }
    return 0;
}
/*
 * atcmd_log_write: record log og atcmd;
 * hongfei's method;
 */
int atcmd_log_write(char *buf, const char *op)
{
   struct timeval tv;
   char tmp[32] = {};
   char *p = NULL;

   gettimeofday(&tv, NULL);
   tv.tv_sec += 3600*8;
   p = ctime(&tv.tv_sec);
   strncpy(tmp, p, strlen(p)-1);
   fprintf(at_fp, "[%s] %s: %s", tmp, op, buf);
   return 0;
}

/*
 * flag for at cmd
 * sign for at result
 * hongfei's method;
 */
void *atcmd_pth_fun()
{
    char buf[1024] = {};
    int n;
    while(1){
        memset(buf, 0, 1024);
        n = read(fd_tty, buf, 1024);
        buf[n] = '\0';
        atcmd_log_write(buf, "r");

        if(strstr(buf, "AT+CPIN?") != NULL){
            flag=1;
		}
        else if(strstr(buf, "AT+GTRNDIS=1,1") != NULL)
		{
            flag=2;
		}
        else if(strstr(buf, "AT+GTRNDIS?") != NULL)
		{
            flag=3;
		}

        if(flag == 1  && strstr(buf, "OK")){
            flag = 0;
            sign = 1;
			at_result = 1;
        }else if(flag == 1  && strstr(buf, "ERROR")){
            flag = 0;
            sign = 1;
			at_result = 0;
        }
		else if(flag == 2  && strstr(buf, "OK")){
            flag = 0;
            sign = 2;
        }else if(flag == 3 && strstr(buf, "GTRNDIS: 1,1")){
            flag = 0;
            sign = 3;
        }
    }
    return ;
}
/*
 * hongfei's method;
 */
int atcmd_cmd_check(char *cmd, int flag)
{
	int i;
	write(fd_tty, cmd, strlen(cmd));
	atcmd_log_write(cmd, "w");
	sleep(1);
	if(at_result == flag){
		sign = 0;
		return 1;
	}
	return 0;
}
/*
 * hongfei's method;
 */
void atcmd_cmd_send(char *cmd, int flag)
{
    while(1){
        write(fd_tty, cmd, strlen(cmd));
        atcmd_log_write(cmd, "w");
        sleep(1);
        if(sign == flag){
            sign = 0;
            break;
        }
    }
    return;
}
/*
 * hongfei's method;
 */
void atcmd_sig_alarm(int signum)
{
    exit(0);
}
/*
 * hongfei's method;
 */
int atcmd_read_tty(char *name)
{
    struct sigaction act;
    pthread_t pthid;
    act.sa_handler = SIG_IGN;
    int recvlen = 0, ret = 0, return_value = 0;

    sigaction(SIGPIPE, &act, NULL);
    signal(SIGALRM, atcmd_sig_alarm);
    alarm(5);

    if ((fd_tty = open(name, O_RDWR)) < 0){
        perror("open");
        return -1;
    }

    if (atcmd_log_open() < 0){
        perror("fopen");
        return -1;
    }
    pthread_mutex_init(&mutex, NULL);
	pthread_create(&pthid, NULL, atcmd_pth_fun, NULL);
	pthread_detach(pthid);

	return_value = atcmd_cmd_check("AT+CPIN?\n", 1);

	alarm(0);
	close(fd_tty);
	//exit(0);
	return at_result;
}
/*
 * used for AT command, read data from serial.
 * */
int read_serial(char *filename, char *cmd, char *rbuff)
{
	int fd, res;
	int i;
	int sr;
	char temp[64];
	int read_times = 10;

	HARDWARE_TEST("enter read_serial\n");
	fd = open(filename, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		HARDWARE_TEST("open %s fail\n", filename);
		return -1;//ERR_READ_SERIAL;
	}

	HARDWARE_TEST("\nwrite data:(%s) to %s\n\n", cmd, filename);
	write(fd, cmd, strlen(cmd));
    usleep(10000);
    sr = 0;
    while(read_times--) {
        if(temp[0] == 'O' && temp[1] == 'K')
        {
            break;
		}
        HARDWARE_TEST("read data from %s\n", filename);
		res = read(fd, temp,64);
		for(i=0;i<res;i++)
		{
            rbuff[sr+i] = temp[i];
		}
		sr = sr + res;
        if(res==0)
        {
            continue;
        }
    }
    rbuff[sr] = '\0';
	/* too often used AT command will caused a serial port error,
	  * let serial sleep 1, put it here for now.
	 */
	sleep(1);
	HARDWARE_TEST("\nread_atbuf:\n %s\n\n", rbuff);
	tcflush(fd, TCIOFLUSH);
	close(fd);
	return 0;//READ_SERIAL_SUCCESS;
}

int simcard_state_test(char *filename, char *cmd, char *msg)
{
#if 0
	/*used hongfei's method*/
	int read_value;
	read_value = atcmd_read_tty(filename);
	printf("%d\n", read_value);
	return read_value;
	/*end of used hongfei's method*/
#endif
	if(0 == read_serial(filename, cmd, msg))
	{
		HARDWARE_TEST("read_atbuf seccess\n");
		return 1;
	}
	else
	{
		HARDWARE_TEST("read_atbuf fail\n");
		return -1;
	}
}

/************************************************************************
* 函数名: can_interface_test 
* 功能描述: CAN接口测试，硬件外部连CAN收发器
* 输入参数: 
*           
* 输出参数:None
* 返回值: 0: false   1: success
*   
*
* 备注: 查看CAN接口是否有数据
*
*************************************************************************/
int can_interface_test(void)
{
    int can_fd = -1;
    int ret = 0;
    /* fd_set*/
    fd_set ser_fdset;
    int max_fd = 1;
    int bytes = 0;
    char buf[CAN_BUF_SIZ];
    int n = 0;
    int i = 0;
    struct timeval mytime;
    struct sockaddr_can addr;
    struct ifreq ifr;
    /*为了能够接收CAN报文，我们需要定义一个CAN数据格式的结构体变量*/
    struct can_frame frame[2];
    struct can_frame *ptr_frame;
    /* 将CAN0波特率设置为125kbps*/
    char cmd_down[] = "ifconfig can0 down";
    char cmd_rate[] = "ip link set can0 type can bitrate 125000";
    char cmd_up[] = "ifconfig can0 up";

    mytime.tv_sec = 20;    /* 接收超时时间，设定20s*/
    mytime.tv_usec = 0;

    /*关闭CAN设备，设置波特率125kbps，再重新打开CAN设备*/
    system(cmd_down);
    system(cmd_rate);
    system(cmd_up);

    /*建立套接字，设置为原始套接字，原始CAN协议 */
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0)
    {
        HARDWARE_TEST("create can socket failed.");
        return 0;
    }

     /*对CAN接口进行初始化，如设置CAN接口名，为用ifconfig命令时显示的名字 */
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_fd, SIOCGIFINDEX, &ifr);
    HARDWARE_TEST("can0 can_ifindex = %x\n", ifr.ifr_ifindex);

    /*设置CAN协议 */
    addr.can_family = PF_CAN;
    //addr.can_ifindex = 0;
    addr.can_ifindex = ifr.ifr_ifindex;
    /*将刚生成的套接字与网络地址进行绑定*/
    ret = bind(can_fd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0)
    {
        HARDWARE_TEST("can socket bind failed.\n");
        goto err2;
    }
    HARDWARE_TEST("interface = %s, family = %d, type = %d, proto = %d\n", ifr.ifr_name, PF_CAN, SOCK_RAW, CAN_RAW);

    FD_ZERO(&ser_fdset);
    /* 增加can socket select检测*/
    FD_SET(can_fd, &ser_fdset);
    max_fd = can_fd;

    frame[0].can_id = 1;
    frame[0].can_dlc = 1;
    frame[0].data[0] = 0x40;

    while (1)
    {
        /* 由夹具上的V-Box板卡收到被测板卡发送的CAN数据后，再发送一次CAN消息，
             被测OBU或者V-Box收到CAN消息表示CAN总线测试通过*/
        bytes = write(can_fd, &frame[0], sizeof(struct can_frame));
        if(bytes != sizeof(frame[0]))
        {
            HARDWARE_TEST("[can socket]: Send Error frame.\n");
        }

        ret = select(max_fd + 1, &ser_fdset, NULL, NULL, &mytime);
        if(ret < 0)    
        {    
            HARDWARE_TEST("select failure\n");    
            continue;    
        }
        else if(ret == 0)
        {
            HARDWARE_TEST("time out!\n");
            goto err2;
        }
        else
        {
            if (FD_ISSET(can_fd, &ser_fdset))
            {
                /* 读取到can0口上的can数据*/
                bytes = read(can_fd, &frame[1], sizeof(struct can_frame));
                if(bytes < 0)
                {
                    HARDWARE_TEST("[can socket]: Receive failed.\n");
                    goto err2;
                }

                if (frame[1].can_id & CAN_EFF_FLAG)
                {
                    n = snprintf(buf, CAN_BUF_SIZ, "<0x%08x> ", frame[1].can_id & CAN_EFF_MASK);
                }
                else
                {
                    n = snprintf(buf, CAN_BUF_SIZ, "<0x%03x> ", frame[1].can_id & CAN_SFF_MASK);
                }

                n += snprintf(buf + n, CAN_BUF_SIZ - n, "[%d] ", frame[1].can_dlc);
                for (i = 0; i < frame[1].can_dlc; i++)
                {
                    n += snprintf(buf + n, CAN_BUF_SIZ - n, "%02x ", frame[1].data[i]);
                }
                if (frame[1].can_id & CAN_RTR_FLAG)
                {
                    n += snprintf(buf + n, CAN_BUF_SIZ - n, "remote request");
                }
                HARDWARE_TEST("%s\n", buf);
                HARDWARE_TEST("[can]: ID=0x%x DLC=0x%x data0=0x%x data1=0x%x data2=0x%x data3=0x%x data4=0x%x data5=0x%x data6=0x%x data7=0x%x!\n", 
                                                            frame[1].can_id, frame[1].can_dlc, frame[1].data[0], frame[1].data[1], frame[1].data[2], 
                                                            frame[1].data[3], frame[1].data[4], frame[1].data[5], frame[1].data[6], frame[1].data[7]);
                break;
            }/*end if (FD_ISSET(can_fd, &ser_fdset))*/
        }
    }

    HARDWARE_TEST("can interface test Success!\n\r");
    close(can_fd);
    system(cmd_down);

    return 1;

err2:
    close(can_fd);
    system(cmd_down);
err1:
    HARDWARE_TEST("can interface test failed!\n\r");

    return 0;
}

/************************************************************************
* 函数名: modem_rf_test 
* 功能描述:RF驱动打桩发固定频点与功率的单音
* 输入参数: 
*           
* 输出参数:None
* 返回值: None
*   
*
* 备注:AT指令已经支持该测试项，需要建立一个tcp client与XDS通信
*              发送'AT + TMCMTX' 指令，等待返回结果'OK'或者'CME ERROR:<err>'
*
*************************************************************************/
void modem_rf_test(void)
{
    int client_fd = -1;
    char sendbuf[TCP_BUFFER_SIZE] = {0};
    char recvbuf[TCP_BUFFER_SIZE] = {0};
    char rxbuf[TCP_BUFFER_SIZE - 2] = {0};
    char close_echo[] = "ATE0\r\n";
    int num = 0;
    int ret = -1;

    /* 定义IPV4的TCP连接套接字描述符*/
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 == client_fd)
    {
        HARDWARE_TEST("Can't create socket.\n\r");
        goto err1;
    }

    /* 定义sockaddr_in*/
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(SERVER_IP_ADDR);
    servaddr.sin_port = htons(SERVER_PORT_NUM);

    /* 连接服务器，成功返回0，错误返回-1*/
    if (connect(client_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        HARDWARE_TEST("Can't connect server.\n\r");
        goto err2;
    }
    HARDWARE_TEST("Connect server(IP:%s; PORT:%d)\n\r", SERVER_IP_ADDR, SERVER_PORT_NUM);

    /* 关闭AT指令回显*/
    num = send(client_fd, close_echo, strlen(close_echo), 0);
    if (0 >= num)
    {
        HARDWARE_TEST("Send AT Command(ATE0) Failed!\n\r");
        goto err2;
    }
    /* 读取ATE0指令返回的消息*/
    recv(client_fd, recvbuf, sizeof(recvbuf), 0);

    HARDWARE_TEST("Please input AT command(test RF interface).\n\r");
    /* 阻塞读取控制台console输入的AT指令*/
    num = read(STDIN_FILENO, rxbuf, TCP_BUFFER_SIZE - 2);
    if(num < 0)
    {
        HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
        goto err1;
    }
    /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
    num = num-1;
    memcpy(sendbuf, rxbuf, num);
    /* XDS端以'\r\n'表示字符输入结束*/
    strcat(sendbuf, "\r\n");

    /* 将控制台输入的信息发送给服务端，再接收服务端返回结果*/
    num = send(client_fd, sendbuf, strlen(sendbuf), 0);
    if (num < 0)
    {
        goto err2;
    }
    HW_LOG("Send %d Bytes.\nAT command: %s\n\r", num, sendbuf);
    usleep(1000);

    num = recv(client_fd, recvbuf, sizeof(recvbuf), 0);
    HW_LOG("Reveive %d Bytes.\nAT command Response: %s\n\r", num, recvbuf);
    /* 接收到XDS的回复内容为'\r\nOK\r\n'具体为'<CR><LF>OK<CR><LF>'*/
    ret = strncmp(recvbuf, "\r\nOK\r\n", 6);
    if (0 != ret)
    {
        HARDWARE_TEST("AT command: %s Response: %s\n\r", sendbuf, recvbuf);
        goto err2;
    }

    HARDWARE_TEST("modem rf test Success!\n\r");
    close(client_fd);

    return;

err2:
    close(client_fd);
err1:
    HARDWARE_TEST("modem rf test Failed!\n\r");

    return;
}

/************************************************************************
* 函数名: spi2_hsm_detect 
* 功能描述:查询ssi2总线上挂载的hsm硬件安全模块的连通性
*
* 输入参数:None
* 
* 输出参数:None
*  
* 返回值: 1: success(查询到HSM); 0: failed(未查询到HSM)
* 备注: 
*************************************************************************/
int spi_hsm_detect(int num)
{
    int length = 0;
    int i = 0;
    int times = 5;
	int ret = 0;
    XKF_HANDLE handle;
    XDJA_DEVINFO devInfo;

	char devName[64];
	snprintf(devName, sizeof(devName), "/dev/spidev%d.0", num); 
	if(num==0){
		HARDWARE_TEST("power on for hsm in spi0\n");
		system("echo 219 > /sys/class/gpio/export");
		system("echo out > /sys/class/gpio/gpio219/direction");
		system("echo 1 > /sys/class/gpio/gpio219/value");
	}

    HARDWARE_TEST("Begin ssi%d HSM test!\n", num);
    ret = XKF_OpenDevByName(devName, &handle);
    if (XKR_OK != ret){
        HARDWARE_TEST("Open %s[HSM Module] failed, ret[%d] \n", devName, ret);
        goto err1;
    }

    HARDWARE_TEST("\n------XKF_GetDevInfo--------\n");
    for (i=0; i<=times; i++)
    {
        ret = XKF_GetDevInfo(handle, &devInfo);
        if (XKR_OK != ret){
            HARDWARE_TEST("Can't Read HSM Info(%d).\nid=%s,\ncosver=%s,\ntype=0x%x\n", i, devInfo.cardid, devInfo.cosver, devInfo.cardtype);
        }else{
            HARDWARE_TEST("Read HSM Info(%d).\n", i);
            break;
        }

        if (times <= i){
            XKF_CloseDev(handle);
            goto err1;
        }
    }
    HARDWARE_TEST("id=%s,\ncosver=%s,\ntype=0x%x\n", devInfo.cardid, devInfo.cosver, devInfo.cardtype);

    XKF_CloseDev(handle);

    HARDWARE_TEST("ssi%d HSM test Success!\n", num);

    return 1;

err1:
    HARDWARE_TEST("ssi%d HSM test Failed!\n", num);

    return 0;
}

/************************************************************************
* 函数名: system_opt 
* 功能描述:进行关机或者重启等系统操作
* 输入参数: 
*           
* 输出参数:
* 返回值: 
*   
*
* 备注
*
*
*************************************************************************/
void system_opt(void)
{
    int input = 0;
    int choice = 0;
    int i = 0;
    int num = 0;
    char rxbuf[MAX_KEY_BYTES];
    int opt_item = 0;

    HARDWARE_TEST("Begin system_option Test!\n\r");

    opt_item = GetListNum(OptList);

    while(1)
    {
        DisplayList(OptList);
        /* 等待控制台console输入*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出sensor测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit sensor test!\n\r");
                break;
            }
        }

        /* 其他字符键*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input sensor test number ERROR!Please input again!\n\r");
                break;
            }
        }

        /* 判断输入的option item参数*/
        if (i != num)
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        ASCToInt(rxbuf, &input, num);
        if(input > opt_item)
        {
            HARDWARE_TEST("The sensor item number input: %d, out of range!Please input again!\n\r", i);
            continue;
        }
        
        /*按下正确测试项，进行测试*/
        choice = input - 1;
        HARDWARE_TEST("===========Item %d  %s : test begin-----Q(q):quit===========\n\r", input, OptList[choice].pname);
        OptList[choice].func();
        HARDWARE_TEST("=========================Item %d: test end=========================\n\r", input);
    }

    return;
}

/************************************************************************
* 函数名: loop_back_test 
* 功能描述:
* 输入参数: 
*           
* 输出参数:
* 返回值: 
*   
*
* 备注
*
*
*************************************************************************/
void loop_back_test(void)
{
    HARDWARE_TEST("Begin loop_back_test Test!\n\r");
    return;
}

/************************************************************************
* 函数名: ssi0_can_test 
* 功能描述:spi0转CAN测试
* 输入参数: 
*           
* 输出参数:
* 返回值: 0: failed   1: success
*   
*
* 备注: 能读取到SPI0上挂载的转CAN设备MCP25625的寄存器值，则测试成功
*
*
*************************************************************************/
int ssi0_can_test(void)
{
    int val = 0;
    int save = 0;
    int i = 0;
    int can_en = 154;
    int reset = 156;
    int en_input = 0;
    int select_input = 0;
    int can_power_en = 237;         /* 0:enable; 1:disable*/
    int can_power_select = 167;     /* 0:1.8V; 1:3.0V*/
    int ret = 0;

    HARDWARE_TEST("Begin ssi0 can Test!\n\r");

    /* DBB2CAN_EN pin, output high level */
    gpio_direction(can_en, GPIO_DIR_OUTPUT);
    gpio_output_set(can_en, HIGH_LEVEL);
    /* RESET pin, STBY output low-level, MCP25625 work in NORMAL mode*/
    gpio_direction(reset, GPIO_DIR_OUTPUT);
    gpio_output_set(reset, LOW_LEVEL);

#if 0
    /* POWER set*/
    gpio_direction(can_power_en, GPIO_DIR_INPUT);
    en_input = gpio_input_value(can_power_en);
    if (HIGH_LEVEL == en_input)
    {
        /* GPIO237输入高电平，去使能ALDO14电压，表示MCP25625芯片不使用*/
        i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3A, 0x5F);
    }
    else
    {
        /* GPIO237输入低电平，使能ALDO14电压，MCP25625芯片使用*/
        gpio_direction(can_power_select, GPIO_DIR_INPUT);
        can_power_select = gpio_input_value(can_power_select);
        if (HIGH_LEVEL == can_power_select)
        {
            /* GPIO167输入高电平，ALDO4打开3.0V*/
            i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3A, 0xD9);
        }
        else
        {
            /* GPIO167输入低电平，ALDO4打开1.8V*/
            i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3A, 0xD1);
        }
    }
#else
    /* ALDO14 enable 3.0V*/
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3A, 0xD9);
#endif

    ssix_reg_init(SSI_0_NUM);
    ssix_gpio_config(SSI_0_NUM);
    ssix_clock_enable(SSI_0_NUM);
    //spix_dump_regs(SSI_0_NUM);

    ssix_WriteReg(SSIx_EN, 0);
    ssix_WriteReg(SSIx_SE, 1);
    /* start trans. enable ssix. */
    ssix_WriteReg(SSIx_EN, 1);

    for (i =0; i < 30; i++)
    {
        save = read_reg_can(SSI_0_NUM, CANCTRL);
        if (0 != save)
        {
            HARDWARE_TEST("Read register CANCTRL = 0x%x, read %d times.\n\r", save, i);
            break;
        }
        /* 30ms读一次*/
        usleep(30000);
    }
    /* 由于在spi-can初始化的时候，已经配置过了，
      * 所以只要能读取到CANCTRL寄存器数据，就表明spi0通路没问题*/
    if (30 == i)
    {
        ret = 0;
        HARDWARE_TEST("ssi0 can test Failed!\n\r");
    }
    else
    {
        ret = 1;
        HARDWARE_TEST("ssi0 can test Success!\n\r");
    }
#if 0
    /* 调用reset can之后，会打乱内核初始化SPI-CAN的流程*/
    reset_can(SSI_0_NUM);
    for (i =0; i < 10; i++)
    {
        val = read_reg_can(SSI_0_NUM, CANCTRL);
        if (0 != val)
        {
            HARDWARE_TEST("Read register CANCTRL = 0x%x, read %d times.\n\r", save, i);
            break;
        }
    }

    for (i =0; i < 10; i++)
    {
        /* 设置MCP25625为正常模式*/
        val = save & 0x1F;
        write_reg_can(SSI_0_NUM, CANCTRL, val);
        val = read_reg_can(SSI_0_NUM, CANCTRL);
        if (0x80 == (val & 0x80))
        {
            HARDWARE_TEST("ssi0 can test Failed!\n\r");
            break;
        }
        else
        {
            HARDWARE_TEST("ssi0 can test Success!\n\r");
            break;
        }
    }

    /* 将原配置写入CANCTRL寄存器*/
    write_reg_can(SSI_0_NUM, CANCTRL, save);
#endif

    /*end of trans disable ssix. */
    ssix_WriteReg(SSIx_EN, 0);

    return ret;
}

/************************************************************************
* 函数名: gpio_direction 
* 功能描述:设置GPIO的fuction和输入输出状态
* 输入参数: gpio_id GPIO号,gpio_dir=1输出，=0为输入
*           
* 输出参数:None
* 返回值: 1正确，0错误
*   
* 备注
*
*************************************************************************/
int gpio_direction(int gpio_id, int gpio_dir)
{
    volatile unsigned int * GPIO_DDR_reg;
    int i;
    
    if ((gpio_id > (AllGPIONUM-1)) || (gpio_dir > 1))
    {
        /* gpio_id输入超出范围,或者gpio_dir不是0/1*/
        return FALSE;
    }
    /* 设置GPIO function   gpio[gpio_id]端口连接到外部管脚GPIO[gpio_id]*/
    PIN_Mux_set(gpio_id, 2);

    GPIO_DDR_reg = (volatile unsigned *)(GPIO_MODEULE_BASE + GPIO_PORT_DDR0) + ((gpio_id/16) & 0xf);
    i = gpio_id % 16;

    if(gpio_dir == GPIO_DIR_OUTPUT)
    {
        /* 把方向寄存器i位置1.设置成输出*/
        SETBIT_ATOM(GPIO_DDR_reg, i, 16);
    }
    else
    {
        /* 把方向寄存器i位置0.设置成输入*/
        CLRBIT_ATOM(GPIO_DDR_reg, i, 16);
    }

    return TRUE;
}

/************************************************************************
* 函数名: gpio_input_value 
* 功能描述:读取GPIO输入的高低值
* 输入参数: gpio_id: GPIO号
*           
* 输出参数:None
* 返回值: 1高电平，0低电平
*   
* 备注
*
*************************************************************************/
int gpio_input_value(int gpio_id)
{
    volatile unsigned int * GPIO_EXT_reg;
    int i;
    if ((gpio_id > (AllGPIONUM-1)))
    {
        return -1;//gpio_id输入超出范围,gpio_dir不是0/1
    }

    GPIO_EXT_reg = (volatile unsigned *)(GPIO_MODEULE_BASE + GPIO_EXT_PORT0) + ((gpio_id/32) & 0x7);
    i = gpio_id % 32;

    return ((ReadReg(GPIO_EXT_reg) & (1<<i)) >> i);
}

/************************************************************************
* 函数名: gpio_output_set 
* 功能描述:设置GPIO输出高低值
* 输入参数: gpio_id: GPIO号，level=1为输出高，level=0为输出低
*           
* 输出参数:None
* 返回值: 1正确，0错误
*   
* 备注
*
*************************************************************************/
int gpio_output_set(int gpio_id, int level)
{
    volatile unsigned int * GPIO_DR_reg;
    int i;
    
    if ((gpio_id>(AllGPIONUM-1)) || (level>1))
    {
        /* gpio_id输入超出范围,或者gpio_dir不是0/1*/
        return FALSE;
    }

    GPIO_DR_reg = (volatile unsigned *)(GPIO_MODEULE_BASE + GPIO_PORT_DR0) + ((gpio_id/16) & 0xf);
    i = gpio_id % 16;

    if(level == HIGH_LEVEL)
    {
        /* 把数据寄存器i位置1.设置成高电平*/
        SETBIT_ATOM(GPIO_DR_reg, i, 16);
    }
    else
    {
        /* 把数据寄存器i位置0.设置成低电平*/
        CLRBIT_ATOM(GPIO_DR_reg, i, 16);
    }
    
    return TRUE;
}

/************************************************************************
* 函数名: PIN_Muxsel_set 
* 功能描述:设置GPIO输出高低值
* 输入参数: Mux_reg_num=0-15,Mux寄存器number，bit要设置的寄存器位，
*                           由于手册中2bit一个功能，所以bit为设置功能的低位，
*                           value为设置的值0/1/2/3。
* 输出参数:None
* 返回值: 1正确，0错误
*   
* 备注
*
*************************************************************************/
void PIN_Mux_set(int Mux_reg_num, int value)
{
    volatile unsigned int * Mux_reg;
    int i;

    Mux_reg = (volatile unsigned *)(MUX_PIN_MODEULE_BASE + MUXPIN_CTRL_DIGRFEN + 4*Mux_reg_num);
    i = ~(0x3 << 0);
    i = i & (*Mux_reg);//先把相应位清0
    i = i | (value << 0);//把要写的值
    *Mux_reg = i;//写寄存器

    return;
}

/************************************************************************
* 函数名: gpio_level_judge 
* 功能描述:判断GPIO的连通性，看看是否为虚焊
* 输入参数: gpio_input:GPIO管脚号,gpio_output:GPIO管脚号
*           
* 输出参数:None
* 返回值: 0:success;   -1:failed;
*
* 备注:
*
*************************************************************************/
int gpio_level_judge(int gpio_input, int gpio_output)
{
    int value = 0;
    int ret = -1;

    /* set GPIO1 GPIO2 gpio function*/
    PIN_Mux_set(gpio_input, 2);
    PIN_Mux_set(gpio_output, 2);
    /*set gpio direction*/
    gpio_direction(gpio_output, GPIO_DIR_OUTPUT);
    gpio_direction(gpio_input, GPIO_DIR_INPUT);
    
    /* set gpio output value*/
    gpio_output_set(gpio_output, HIGH_LEVEL);
    /* read input gpio level*/
    value = gpio_input_value(gpio_input);
    if (HIGH_LEVEL == value)
    {
        gpio_output_set(gpio_output, LOW_LEVEL);
        value = gpio_input_value(gpio_input);
        if (LOW_LEVEL == value)
        {
            ret = 0;
            HW_LOG("GPIO%d INPUT and GPIO%d OUTPUT loopback test success!\n\r", gpio_input, gpio_output);
        }
        else
        {
            ret = -1;
            HW_LOG("GPIO%d is output low level, but GPIO%d read high level.\n\r", gpio_output, gpio_input);
        }
    }
    else
    {
        ret = -1;
        HW_LOG("GPIO%d is output high level, but GPIO%d read low level.\n\r", gpio_output, gpio_input);
    }
    
    return ret;
}

/************************************************************************
* 函数名: i2cx_init 
* 功能描述: 初始化，设置7bit地址，没有设置I2C地址，
*                            需要和I2Cx_SetDevAddr配合使用
* 输入参数: i2c number=0,=1, speed=0为100K，=1为400K
* 输出参数: 无
* 返回值: 1正确，0错误
* 备注:    
*
*************************************************************************/
int i2cx_init(int i2cx, int speed)
{
    int flag = 0;

    i2cx_reg_init(i2cx);

    /* I2C状态寄存器I2Cx STATUS bit0如为0进行下一步，1则等待。0:不激活, 1:激活*/
    while ((*I2Cx_STATUS) & (1 << I2C_STATUS_ACTIVITY))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("I2C%d init failed!\n\r", i2cx);
            return 0;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }

    /* 去使能，配置寄存器*/
    *I2Cx_ENABLE = 0;
    usleep(1000);
    
    //set as fast mode, master mode , restart enable,10bit address mode 
    if (speed == 0)//100k
    {
        /*在写入控制寄存器时必须将此位置1.*/
        *I2Cx_CON = (1 << I2C_CON_NOMEANING) |
        /*使能master模式下的restart功能*/
        (1 << I2C_CON_I2C_RESTART_EN) |
        /*7位地址*/
        (0 << I2C_CON_I2C_10BITADDR_MASTER) |
        /*标准工作模式*/
        (I2C_CON_SPEED << I2C_CON_SPEED) |
        /*master使能*/
        (1 << I2C_CON_MASTER_MODE);

        /*************intial I2C Pulse width *****************/
        /* >4us,I2c_clk频率固定为13MHz=76.9ns 4000/76.9=52.01，所以HCNT=53*/
        *I2Cx_SS_SCL_HCNT = 0x3c;// 0x35;
        /* >4.7us,4700/76.9=61.1,所以LCNT=62*/
        *I2Cx_SS_SCL_LCNT = 0x41;// 0x3E;
    }
    else if (speed == 1)//400k
    {
        /*在写入控制寄存器时必须将此位置1.*/
        *I2Cx_CON = (1 << I2C_CON_NOMEANING) |
        /*使能master模式下的restart功能*/
        (1 << I2C_CON_I2C_RESTART_EN) |
        /*7位地址*/
        (0 << I2C_CON_I2C_10BITADDR_MASTER) |
        /*快速工作模式*/
        (I2C_CON_SPEED_FAST << I2C_CON_SPEED) |
        /*master使能*/
        (1 << I2C_CON_MASTER_MODE);

        /*************intial I2C Pulse width *****************/	
        /* >0.6us,I2c_clk频率固定为13MHz=76.9ns 600/76.9=7.8，所以HCNT=8*/
        *I2Cx_FS_SCL_HCNT = 0xE;//0x08;
        /* >1.3us,1300/76.9=16.9,所以LCNT=17*/
        *I2Cx_FS_SCL_LCNT = 0x12;//0x11;
    }
    else
    {
        HARDWARE_TEST("Set the speed of I2C bus error!\n\r");
        return 0;//错误
    }

    /* 设置7bit地址模式*/
    *I2Cx_TAR = (0<<I2C_TAR_10BITADDR) |
    /* 设置忽略GC_OR_START*/
    (0<<I2C_TAR_SPECIAL) |
    /* 设置通用寻址*/
    (0<<I2C_TAR_GCORSTART);
//    *I2Cx_RX_TL = 0;
//    *I2Cx_TX_TL = 0xc;
    /* 关闭所有中断*/
    *I2Cx_INTR_EN = 0;

    *I2Cx_SDA_HOLD = 3;

    /* Enable I2C*/
    *I2Cx_ENABLE = 1;
    usleep(1000);
    
    return 1;
}

/************************************************************************
* 函数名: i2cx_reg_init 
* 功能描述: 初始化寄存器
* 输入参数: I2C端口号
* 输出参数: None
* 返回值: None
* 备注:    
*
*************************************************************************/
void i2cx_reg_init(int I2Cx)
{
    if (I2Cx == 0)
    {
        I2Cx_CON = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CON);
        I2Cx_TAR = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_TAR);
        I2Cx_HS_MADDR = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_HS_MADDR);
        I2Cx_DATA_CMD = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_DATA_CMD);
        I2Cx_SS_SCL_HCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_SS_SCL_HCNT);
        I2Cx_SS_SCL_LCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_SS_SCL_LCNT);
        I2Cx_FS_SCL_HCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_FS_SCL_HCNT);
        I2Cx_FS_SCL_LCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_FS_SCL_LCNT);
        I2Cx_HS_SCL_HCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_HS_SCL_HCNT);
        I2Cx_HS_SCL_LCNT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_HS_SCL_LCNT);
        I2Cx_INTR_STAT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_INTR_STAT);
        I2Cx_INTR_EN = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_INTR_EN);
        I2Cx_RAW_INTR_STAT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_RAW_INTR_STAT);
        I2Cx_RX_TL = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_RX_TL);
        I2Cx_TX_TL = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_TX_TL);
        I2Cx_CLR_INTR = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_INTR);
        I2Cx_CLR_RX_UNDER = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_RX_UNDER);
        I2Cx_CLR_RX_OVER = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_RX_OVER);
        I2Cx_CLR_TX_OVER = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_TX_OVER);
        I2Cx_CLR_TX_ABRT = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_TX_ABRT);
        I2Cx_CLR_ACTIVITY = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_ACTIVITY);
        I2Cx_CLR_STOP_DET = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_STOP_DET);
        I2Cx_CLR_START_DET = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_START_DET);
        I2Cx_CLR_GEN_CALL = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_CLR_GEN_CALL);
        I2Cx_ENABLE = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_ENABLE);
        I2Cx_STATUS = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_STATUS);
        I2Cx_TXFLR = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_TXFLR);
        I2Cx_RXFLR = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_RXFLR);
        I2Cx_SDA_HOLD = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_SDA_HOLD);
        I2Cx_TX_ABRT_SOURCE = (volatile unsigned *)(I2C0_MODEULE_BASE + I2C0_TX_ABRT_SOURCE);
    }
    else if (I2Cx == 1)
    {
        I2Cx_CON = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CON);
        I2Cx_TAR = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_TAR);
        I2Cx_HS_MADDR = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_HS_MADDR);
        I2Cx_DATA_CMD = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_DATA_CMD);
        I2Cx_SS_SCL_HCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_SS_SCL_HCNT);
        I2Cx_SS_SCL_LCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_SS_SCL_LCNT);
        I2Cx_FS_SCL_HCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_FS_SCL_HCNT);
        I2Cx_FS_SCL_LCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_FS_SCL_LCNT);
        I2Cx_HS_SCL_HCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_HS_SCL_HCNT);
        I2Cx_HS_SCL_LCNT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_HS_SCL_LCNT);
        I2Cx_INTR_STAT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_INTR_STAT);
        I2Cx_INTR_EN = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_INTR_EN);
        I2Cx_RAW_INTR_STAT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_RAW_INTR_STAT);
        I2Cx_RX_TL = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_RX_TL);
        I2Cx_TX_TL = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_TX_TL);
        I2Cx_CLR_INTR = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_INTR);
        I2Cx_CLR_RX_UNDER = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_RX_UNDER);
        I2Cx_CLR_RX_OVER = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_RX_OVER);
        I2Cx_CLR_TX_OVER = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_TX_OVER);
        I2Cx_CLR_TX_ABRT = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_TX_ABRT);
        I2Cx_CLR_ACTIVITY = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_ACTIVITY);
        I2Cx_CLR_STOP_DET = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_STOP_DET);
        I2Cx_CLR_START_DET = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_START_DET);
        I2Cx_CLR_GEN_CALL = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_CLR_GEN_CALL);
        I2Cx_ENABLE = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_ENABLE);
        I2Cx_STATUS = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_STATUS);
        I2Cx_TXFLR = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_TXFLR);
        I2Cx_RXFLR = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_RXFLR);
        I2Cx_SDA_HOLD = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_SDA_HOLD);
        I2Cx_TX_ABRT_SOURCE = (volatile unsigned *)(I2C1_MODEULE_BASE + I2C1_TX_ABRT_SOURCE);
    }
    else if (I2Cx == 2)
    {
        I2Cx_CON = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CON);
        I2Cx_TAR  = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_TAR);
        I2Cx_HS_MADDR = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_HS_MADDR);
        I2Cx_DATA_CMD = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_DATA_CMD);
        I2Cx_SS_SCL_HCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_SS_SCL_HCNT);
        I2Cx_SS_SCL_LCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_SS_SCL_LCNT);
        I2Cx_FS_SCL_HCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_FS_SCL_HCNT);
        I2Cx_FS_SCL_LCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_FS_SCL_LCNT);
        I2Cx_HS_SCL_HCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_HS_SCL_HCNT);
        I2Cx_HS_SCL_LCNT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_HS_SCL_LCNT);
        I2Cx_INTR_STAT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_INTR_STAT);
        I2Cx_INTR_EN = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_INTR_EN);
        I2Cx_RAW_INTR_STAT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_RAW_INTR_STAT);
        I2Cx_RX_TL = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_RX_TL);
        I2Cx_TX_TL = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_TX_TL);
        I2Cx_CLR_INTR = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_INTR);
        I2Cx_CLR_RX_UNDER = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_RX_UNDER);
        I2Cx_CLR_RX_OVER = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_RX_OVER);
        I2Cx_CLR_TX_OVER = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_TX_OVER);
        I2Cx_CLR_TX_ABRT = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_TX_ABRT);
        I2Cx_CLR_ACTIVITY = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_ACTIVITY);
        I2Cx_CLR_STOP_DET = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_STOP_DET);
        I2Cx_CLR_START_DET = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_START_DET);
        I2Cx_CLR_GEN_CALL = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_CLR_GEN_CALL);
        I2Cx_ENABLE = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_ENABLE);
        I2Cx_STATUS = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_STATUS);
        I2Cx_TXFLR = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_TXFLR);
        I2Cx_RXFLR = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_RXFLR);
        I2Cx_SDA_HOLD = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_SDA_HOLD);
        I2Cx_TX_ABRT_SOURCE = (volatile unsigned *)(I2C2_MODEULE_BASE + I2C2_TX_ABRT_SOURCE);
    }
    else if (I2Cx == 3)
    {
        I2Cx_CON = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CON);
        I2Cx_TAR = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_TAR);
        I2Cx_HS_MADDR = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_HS_MADDR);
        I2Cx_DATA_CMD = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_DATA_CMD);
        I2Cx_SS_SCL_HCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_SS_SCL_HCNT);
        I2Cx_SS_SCL_LCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_SS_SCL_LCNT);
        I2Cx_FS_SCL_HCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_FS_SCL_HCNT);
        I2Cx_FS_SCL_LCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_FS_SCL_LCNT);
        I2Cx_HS_SCL_HCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_HS_SCL_HCNT);
        I2Cx_HS_SCL_LCNT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_HS_SCL_LCNT);
        I2Cx_INTR_STAT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_INTR_STAT);
        I2Cx_INTR_EN = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_INTR_EN);
        I2Cx_RAW_INTR_STAT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_RAW_INTR_STAT);
        I2Cx_RX_TL = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_RX_TL);
        I2Cx_TX_TL = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_TX_TL);
        I2Cx_CLR_INTR = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_INTR);
        I2Cx_CLR_RX_UNDER = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_RX_UNDER);
        I2Cx_CLR_RX_OVER = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_RX_OVER);
        I2Cx_CLR_TX_OVER = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_TX_OVER);
        I2Cx_CLR_TX_ABRT = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_TX_ABRT);
        I2Cx_CLR_ACTIVITY = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_ACTIVITY);
        I2Cx_CLR_STOP_DET = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_STOP_DET);
        I2Cx_CLR_START_DET = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_START_DET);
        I2Cx_CLR_GEN_CALL = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_CLR_GEN_CALL);
        I2Cx_ENABLE = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_ENABLE);
        I2Cx_STATUS = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_STATUS);
        I2Cx_TXFLR = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_TXFLR);
        I2Cx_RXFLR = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_RXFLR);
        I2Cx_SDA_HOLD = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_SDA_HOLD);
        I2Cx_TX_ABRT_SOURCE = (volatile unsigned *)(I2C3_MODEULE_BASE + I2C3_TX_ABRT_SOURCE);
    }
    else if (I2Cx == 4)
    {
        I2Cx_CON = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CON); 
        I2Cx_TAR = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_TAR); 
        I2Cx_HS_MADDR = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_HS_MADDR); 
        I2Cx_DATA_CMD = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_DATA_CMD); 
        I2Cx_SS_SCL_HCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_SS_SCL_HCNT); 
        I2Cx_SS_SCL_LCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_SS_SCL_LCNT); 
        I2Cx_FS_SCL_HCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_FS_SCL_HCNT); 
        I2Cx_FS_SCL_LCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_FS_SCL_LCNT); 
        I2Cx_HS_SCL_HCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_HS_SCL_HCNT); 
        I2Cx_HS_SCL_LCNT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_HS_SCL_LCNT); 
        I2Cx_INTR_STAT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_INTR_STAT); 
        I2Cx_INTR_EN = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_INTR_EN); 
        I2Cx_RAW_INTR_STAT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_RAW_INTR_STAT); 
        I2Cx_RX_TL = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_RX_TL); 
        I2Cx_TX_TL = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_TX_TL); 
        I2Cx_CLR_INTR = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_INTR); 
        I2Cx_CLR_RX_UNDER = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_RX_UNDER); 
        I2Cx_CLR_RX_OVER = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_RX_OVER); 
        I2Cx_CLR_TX_OVER = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_TX_OVER); 
        I2Cx_CLR_TX_ABRT = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_TX_ABRT); 
        I2Cx_CLR_ACTIVITY = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_ACTIVITY); 
        I2Cx_CLR_STOP_DET = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_STOP_DET); 
        I2Cx_CLR_START_DET = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_START_DET); 
        I2Cx_CLR_GEN_CALL = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_CLR_GEN_CALL); 
        I2Cx_ENABLE = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_ENABLE); 
        I2Cx_STATUS = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_STATUS); 
        I2Cx_TXFLR = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_TXFLR); 
        I2Cx_RXFLR = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_RXFLR); 
        I2Cx_SDA_HOLD  = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_SDA_HOLD); 
        I2Cx_TX_ABRT_SOURCE = (volatile unsigned *)(COM_I2C_MODEULE_BASE + COM_I2C_TX_ABRT_SOURCE); 
    }
    else
    {
        HARDWARE_TEST("i2cx_Reg_init: I2C NUM %d out of range!\n\r", I2Cx);
    }

    return;
}

/************************************************************************
* 函数名: get_i2cx_parent_rate 
* 功能描述:  获取I2Cx端口上级时钟
* 输入参数:
* 输出参数: None
* 返回值: None
* 备注:    
*
*************************************************************************/
void get_i2cx_parent_rate(void)
{
    unsigned long  pll1_out_rate = 0;
    unsigned long  pll1_mclk_rate = 0;
    unsigned int   val;
    unsigned int   refdiv;
    unsigned int   fbdiv;
    unsigned int   postdiv1;
    unsigned int   postdiv2;

    val = *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_PLL1CFG_CTL);
    refdiv = val & 0x3f;
    fbdiv = (val >> 8) & 0xfff;
    postdiv1 = (val >> 20) & 0x7;
    postdiv2 = (val >> 24) & 0x7;

    pll1_out_rate = osc_clk_rate / refdiv / postdiv1 / postdiv2 * fbdiv;
    //HARDWARE_TEST("pll1_out_rate:%ld.\n\r", pll1_out_rate);

    pll1_mclk_rate = pll1_out_rate / 4;
    //HARDWARE_TEST("pll1_mclk_rate:%ld.\n\r", pll1_mclk_rate);

    i2cx_parent_rate = pll1_mclk_rate;

    return;    
}

/************************************************************************
* 函数名: i2cx_clock_init_set_rate 
* 功能描述:  设置i2cx时钟去使能
* 输入参数:  需要去使能的i2c端口号
* 输出参数: None
* 返回值: None
* 备注:    I2Cx时钟配置参照驱动文件clock-lc1860.h & clock-lc1860.c
*                  时钟速率由父到子一级一级传递分频，时钟父到子的关系如下
*                  osc_clk->pll1_out->pll1_mclk->i2cx_clk, 其中osc_clk时钟为26M
*************************************************************************/
int i2cx_clock_init_set_rate(int I2Cx)
{
    unsigned long parent_rate = i2cx_parent_rate;
    unsigned long cal_rate;
    unsigned long rate, last_rate;
    unsigned int  div, last_div, divclk_val;
    unsigned char divclk_bit, divclk_mask, mclk_bit;
    unsigned int  val, val_mast;
    unsigned int  en = 0;

    if (I2Cx == 0)
    {
        divclk_bit = 0;
        divclk_mask = 0x1F;
        mclk_bit = 0;
        val_mast = 0xFFFFFF00;
    }
    else if (I2Cx == 1)
    {
        divclk_bit = 8;
        divclk_mask = 0x1F;
        mclk_bit = 1;
        val_mast = 0xFFFF00FF;
    }
    else if (I2Cx == 2)
    {
        divclk_bit = 16;
        divclk_mask = 0x1F;
        mclk_bit = 1;
        val_mast = 0xFF00FFFF;
    }
    else if (I2Cx == 3)
    {
        divclk_bit = 24;
        divclk_mask = 0x1F;
        mclk_bit = 2;
        val_mast = 0xFFFFFF;
    }
    else
    {
        HARDWARE_TEST("i2cx_clock_init_set_rate: Parameter err, I2C NUM %d out of range!\n\r", I2Cx);
        return -1;
    }

    val = *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_I2CCLK_CTL);
    divclk_val = (val >> divclk_bit) & divclk_mask;
    rate = parent_rate / (divclk_val + 1);

    if (2 != I2Cx)
    {
        /*for sdiv disable clk first*/
        if((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN))
        {
            val = *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN);
            en = (val >> mclk_bit) & 0x1;
            if(en)
            {
                i2cx_clock_disable(I2Cx);
                usleep(1000);
            }
        }
    }
    else
    {
        /* I2C2 MCLK 使能寄存器bit1*/
        if((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN))
        {
            val = *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN);
            en = (val >> mclk_bit) & 0x1;
            if(en)
            {
                i2cx_clock_disable(I2Cx);
                usleep(1);
            }
        }
    }

    div = parent_rate / rate - 1;

    if (div < 1 || div > divclk_mask)
    {
        return -1;
    }

    last_rate = parent_rate / (div + 1);
    last_div = div;

    if (rate != last_rate)
    {
        div++;
        cal_rate = parent_rate / (div + 1);
        if ((rate - cal_rate) < (rate - last_rate))
        {
            divclk_val = div;
            rate = cal_rate;
        }
        else
        {
            divclk_val = last_div;
            rate = last_rate;
        }
        //HARDWARE_TEST("set I2C%d rate %ld.\n\r", I2Cx, rate);
    }
    else
    {
        divclk_val = last_div;
        rate = last_rate;
    }

    /* AP_PWR_I2CCLK_CTL寄存器每个I2Cx总线分频比占8bit(I2C0~I2C3)*/
    val = (*(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_I2CCLK_CTL) & val_mast) | (divclk_val << divclk_bit);

    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_I2CCLK_CTL) = val;

    if (2 != I2Cx)
    {
        /*restore enable status*/
        if((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN))
        {
            if(en)
            {
                i2cx_clock_enable(I2Cx);
            }
        }
    }
    else
    {
        /* I2C2 MCLK 使能寄存器bit1*/
        if((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN))
        {
            if(en)
            {
                i2cx_clock_enable(I2Cx);
            }
        }
    }
    //HARDWARE_TEST("I2C%d_clock_rate rate:%ld, last_rate:%ld, clk_div:%ld.\n\r", I2Cx, rate, last_rate, divclk_val);

    i2cx_clock_enable(I2Cx);

    return 0;
}

/************************************************************************
* 函数名: i2cx_clock_enable 
* 功能描述:  设置i2cx时钟使能
* 输入参数:  需要使能的i2c端口号
* 输出参数: None
* 返回值: None
* 备注:    I2Cx时钟配置参照驱动文件clock-lc1860.h & clock-lc1860.c
*                  时钟速率由父到子一级一级传递分频，时钟父到子的关系如下
*                  osc_clk->pll1_out->pll1_mclk->i2cx_clk, 其中osc_clk时钟为26M
*************************************************************************/
void i2cx_clock_enable(int I2Cx)
{
    unsigned int val;
    unsigned char mclk_bit, mclk_we_bit, ifclk_bit, ifclk_we_bit;

    if (I2Cx == 0)
    {
        mclk_bit = 0;
        mclk_we_bit = 16;
        ifclk_bit = 5;
        ifclk_we_bit = 21;
    }
    else if (I2Cx == 1)
    {
        mclk_bit = 1;
        mclk_we_bit = 17;
        ifclk_bit = 6;
        ifclk_we_bit = 22;
    }
    else if (I2Cx == 2)
    {
        mclk_bit = 1;
        mclk_we_bit = 17;
        ifclk_bit = 0;
        ifclk_we_bit = 16;
    }
    else if (I2Cx == 3)
    {
        mclk_bit = 2;
        mclk_we_bit = 18;
        ifclk_bit = 7;
        ifclk_we_bit = 23;
    }
    else
    {
        HARDWARE_TEST("I2Cx_clock_enable: Parameter err, I2C NUM %d out of range!\n\r", I2Cx);
        return;
    }

    if (2 != I2Cx)
    {
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN) != 0)
        {
            val = (1 << mclk_bit) | (1 << mclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN) = val;
        }

        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLPCLK_EN) != 0)
        {
            val = (1 << ifclk_bit) | (1 << ifclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLPCLK_EN) = val;
        }
        //HARDWARE_TEST("I2C%d_clock_rate mclk_en:0x%lx, clk_en:0x%lx.\n\r", I2Cx, *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN), *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLPCLK_EN));
    }
    else
    {
        /* I2C2 mclk及pclk使能寄存器与I2C0、I2C1、I2C3不一样*/
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN) != 0)
        {
            val = (1 << mclk_bit) | (1 << mclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN) = val;
        }

        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN) != 0)
        {
            val = (1 << ifclk_bit) | (1 << ifclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN) = val;
        }
        HARDWARE_TEST("I2C%d_clock_rate mclk_en:0x%lx, clk_en:0x%lx.\n\r", I2Cx, *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN), *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN));
    }

    return;
}

/************************************************************************
* 函数名: i2cx_clock_disable 
* 功能描述:  设置i2cx时钟去使能
* 输入参数:  需要去使能的i2c端口号
* 输出参数: None
* 返回值: None
* 备注:    I2Cx时钟配置参照驱动文件clock-lc1860.h & clock-lc1860.c
*                  时钟速率由父到子一级一级传递分频，时钟父到子的关系如下
*                  osc_clk->pll1_out->pll1_mclk->i2cx_clk, 其中osc_clk时钟为26M
*************************************************************************/
void i2cx_clock_disable(int I2Cx)
{
    unsigned int val;
    unsigned char mclk_we_bit, ifclk_we_bit;
    
    if (I2Cx == 0)
    {
        mclk_we_bit = 16;
        ifclk_we_bit = 21;
    }
    else if (I2Cx == 1)
    {
        mclk_we_bit = 17;
        ifclk_we_bit = 22;
    }
    else if (I2Cx == 2)
    {
        mclk_we_bit = 17;
        ifclk_we_bit = 16;
    }
    else if (I2Cx == 3)
    {
        mclk_we_bit = 18;
        ifclk_we_bit = 23;
    }
    else
    {
        HARDWARE_TEST("I2Cx_clock_disable: Parameter err, I2C NUM %d out of range!\n\r", I2Cx);
        return;
    }

    if (2 != I2Cx)
    {
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN) != 0)
        {
            val = (1 << mclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLAPBMCLK_EN) = val;
        }

        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLPCLK_EN) != 0)
        {
            val = (1 << ifclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CTLPCLK_EN) = val;
        }
    }
    else
    {
        /* I2C2 mclk及pclk使能寄存器与I2C0、I2C1、I2C3不一样*/
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN) != 0)
        {
            val = (1 << mclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECAPBMCLK_EN) = val;
        }

        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN) != 0)
        {
            val = (1 << ifclk_we_bit);
            *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN) = val;
        }
    }
    //HARDWARE_TEST("Disable I2C%d clock.\n\r", I2Cx);

    return;
}


/************************************************************************
* 函数名: i2cx_SetDevAddr 
* 功能描述:  设置i2cx总线设备从地址
* 输入参数:  Slave地址
* 输出参数: None
* 返回值: None
* 备注:    写入要访问的slave地址
* 
*************************************************************************/
void i2cx_SetDevAddr(int i2cx, unsigned short addr)
{
    i2cx_reg_init(i2cx);

    /* 禁止I2C*/
    *I2Cx_ENABLE = 0;
    usleep(2000);

    /* 把TAR地址写出去, 只有在I2C禁止时写入才有效, 写入TAR, 高位保留*/
    *I2Cx_TAR = (*I2Cx_TAR & (~I2C_TAR_TAR)) | (addr & I2C_TAR_TAR);
    /* 使能I2C*/
    *I2Cx_ENABLE = 1;
    usleep(2000);

    return;
}

/************************************************************************
* 函数名: i2cx_WriteReg 
* 功能描述:  往从设备发送数据
* 输入参数:  reg, data, data
* 输出参数: None
* 返回值: None
* 备注:    先写出reg，再写Data
* 
*************************************************************************/
void i2cx_WriteReg(int i2cx, unsigned char reg, unsigned char data)
{
    int flag = 0;

    i2cx_reg_init(i2cx);

    /* I2C_STATUS_TFE=0,TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS) & (1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. I2C%d WriteReg failed!\n\r", i2cx);
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    /* CMD bit8=0表示写，=1表示读；7:0发送或接收的数据*/
    *I2Cx_DATA_CMD = reg;
    *I2Cx_DATA_CMD = data | (1<<9);
    usleep(10000);

    return;
}

/************************************************************************
* 函数名: i2cx_ReadReg 
* 功能描述:  往从设备发送数据
* 输入参数:  reg
* 输出参数: None
* 返回值: None
* 备注:    先写出reg，再读Data，返回1个Byte数据
* 
*************************************************************************/
unsigned char i2cx_ReadReg(int i2cx, unsigned char reg)
{
    int flag = 0;

    i2cx_reg_init(i2cx);

    /* I2C_STATUS_TFE=0, TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. I2C%d ReadReg failed!\n\r", i2cx);
            return 0;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    *I2Cx_DATA_CMD = reg;
    *I2Cx_DATA_CMD = 0x100 | (1<<9) | (1<<10);
    usleep(10);

    flag = 0;
    /* I2C_STATUS_RFNE=0, RX FIFO空，循环，直到RX FIFO不空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_RFNE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("RX FIFO empty. I2C%d ReadReg failed!\n\r", i2cx);
            return 0;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    usleep(20);

    /* 读取I2C接收的数据*/
    return  *I2Cx_DATA_CMD;
}

/************************************************************************
* 函数名: e2prom_byte_write 
* 功能描述:  E2PROM型号:CAT24C08
*                     对 E2PROM进行字节写操作
*
* 输入参数:  addr:待写的地址
*                           data:待写的数据
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void e2prom_byte_write(char addr, unsigned int data)
{
    unsigned short slave_addr = E2PROM_ADDR;
    int flag = 0;

    /*根据E2PROM CAT24C08芯片字节写规定:
      *主机 开始发送一个START信号，后跟slave设备地址(写地址)，
      *待slave从设备回复后，再发送待写内存地址，
      *收到slave从设备回复后，再发送待写数据，
      *收到slave从设备回复后，最后发送STOP信号。
      *注:CAT24C08芯片共1024Bytes，因此，待写地址不能超过1023*/
    if ((addr < 0) || (addr > 1023))
    {
        HARDWARE_TEST("E2PROM. Write out of address range. Address range: 0~1023. Now Write address:%d!\n\r", addr);
        return;
    }

    i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);

    /* I2C_STATUS_TFE=0,TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. can't send addr, e2prom byte write failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    /* CMD bit8=0表示写，=1表示读；7:0发送或接收的数据*/
    *I2Cx_DATA_CMD = addr;
    usleep(20000);

    flag = 0;
    /* I2C_STATUS_TFE=0,TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. can't send data, e2prom byte write failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    *I2Cx_DATA_CMD = data | (1<<9);
    usleep(20000);
    
    return;
}

/************************************************************************
* 函数名: e2prom_page_write 
* 功能描述:  E2PROM型号:CAT24C08
*                     对 E2PROM进行页写操作
*
* 输入参数:  page:待写的页，总共1024Bytes，page范围:0~63;
*                           data:待写的数据
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void e2prom_page_write(char page, unsigned int data)
{
    int i;
    int flag = 0;
    unsigned char addr = 0x0;
    unsigned short slave_addr = E2PROM_ADDR;

    /*根据E2PROM CAT24C08芯片页写规定，一次最多连续写16bytes，一页。
      *主机 就必须发送一个stop信号，不然会覆盖之前写入的地址。
      *在写的过程成，低4位address地址会自动增加，高8位仍然不变，需手动操作。
      *注:CAT24C08芯片1024Bytes，需10位address地址位表示。发送数据前的8位地址，再加
      *芯片I2C硬件地址的a9、a10两位，总共10位地址。
      *10位地址，高6位表示页，低4位表示页内，因此每页16Bytes大小。*/
    if ((page < 0) || (page > 63))
    {
        HARDWARE_TEST("E2PROM. Write out of page range. Page range: 0~63. Now Write Page address:%d!\n\r", page);
        return;
    }

    /*8 bit address, addr:0~255 Bytes, page:0~15*/
    if ((page >= 0) && (page < 16))
    {
        i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
        addr = page * E2PROM_PAGE_SIZE;
    }
    /*超过256Bytes的写，需要把I2C从地址的a8、a9算上。slave address: (1) (0) (1) (0) (A2) (a9) (a8) (R/W)*/
    /*addr:256~511 Bytes, page:16~31*/
    else if ((page >= 16) && (page < 32))
    {
        slave_addr |= 0x1;
        i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
        addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE;
    }
    /*addr:512~767 Bytes, page:32~47*/
    else if ((page >= 32) && (page < 48))
    {
        slave_addr |= 0x2;
        i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
        addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE;
    }
    /*addr:768~1023 Bytes, page:48~63*/
    else
    {
        slave_addr |= 0x3;
        i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
        addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE;
    }

    /* I2C_STATUS_TFE=0,TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. can't send addr, e2prom page write failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    /* CMD bit8=0表示写，=1表示读；7:0发送或接收的数据*/
    *I2Cx_DATA_CMD = addr;
    usleep(20000);
    for(i=0; i<E2PROM_PAGE_SIZE-1; i++)
    {
        flag = 0;
        /* I2C_STATUS_TFE=0,TX FIFO不空，循环，直到TX FIFO空*/
        while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
        {
            /* 等待3s仍未读到数据返回失败*/
            if (300 <= flag)
            {
                HARDWARE_TEST("TX FIFO not empty. can't send data(%d), e2prom page write failed!\n\r", i);
                return;
            }
            flag++;
            /* 睡眠10ms*/
            usleep(10000);
        }
        *I2Cx_DATA_CMD = data;
        usleep(20000);
    }

    /* 连续写完第16个字节后，发送STOP信号*/
    *I2Cx_DATA_CMD = data | (1<<9);
    usleep(100000);
    
    return;
}

/************************************************************************
* 函数名: e2prom_seq_read 
* 功能描述:  E2PROM型号:CAT24C08
*                     E2PROM连续读操作，从第一个字节读到芯片最后一个字节
*
* 输入参数: data:返回数据
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void e2prom_seq_read(unsigned char *data)
{
    unsigned char buf[1024] = {0};
    int i = 0;
    int flag = 0;

    /*根据E2PROM CAT24C08芯片连续读规定，一次可以从第一个字节读到E2PROM的结尾，而不是页的尾部，该E2PROM为1024Bytes大小。
      *在读的过程，10位address地址会自动增加，不需要手动操作。
      *注:CAT24C08芯片1024Bytes，需10位address地址位表示。接收数据前的8位地址，再加
      *芯片I2C硬件地址的a9、a10两位，总共10位地址。
      *10位地址，高6位表示页，低4位表示页内，因此每页16Bytes大小。*/

    /* I2C_STATUS_TFE=0, TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS) & (1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. e2prom seq read failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    *I2Cx_DATA_CMD = 0;
    *I2Cx_DATA_CMD = 0x100;
    usleep(10000);

    flag = 0;
    /* I2C_STATUS_RFNE=0, RX FIFO空，循环，直到RX FIFO不空*/
    while(!((*I2Cx_STATUS) & (1 << I2C_STATUS_RFNE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("RX FIFO empty. e2prom seq read failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    usleep(20000);

    for (i=0; i<1022; i++)
    {
        flag = 0;
        /* 读取I2C返回的数据*/
        buf[i] = *I2Cx_DATA_CMD;
        /*每读取一个字节，需要再次发送一次读信号，以回应从设备，这是Master ACK信号*/
        *I2Cx_DATA_CMD = 0x100;

        /* I2C_STATUS_RFNE=0, RX FIFO空，循环，直到RX FIFO不空*/
        while(!((*I2Cx_STATUS) & (1 << I2C_STATUS_RFNE)))
        {
            /* 等待3s仍未读到数据返回失败*/
            if (300 <= flag)
            {
                HARDWARE_TEST("RX FIFO empty. can't read data(%d), e2prom seq read failed!\n\r", i);
                return;
            }
            flag++;
            /* 睡眠10ms*/
            usleep(10000);
        }
        usleep(20000);
    }
    /* 读取I2C接收的E2PROM第1023字节数据*/
    buf[1022] = *I2Cx_DATA_CMD;

    flag = 0;
    /* 读取到E2PROM最后一个字节，需要发送一个STOP信号，告诉从设备，读取结束*/
    *I2Cx_DATA_CMD = 0x100 | (1<9);
    /* I2C_STATUS_RFNE=0,RX FIFO空，循环，直到RX FIFO不空*/
    while(!((*I2Cx_STATUS) & (1 << I2C_STATUS_RFNE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("RX FIFO empty. e2prom seq read failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    usleep(20000);
    buf[1023] = *I2Cx_DATA_CMD;

    memcpy(data, buf, 1024);
    
    return;
}

/************************************************************************
* 函数名: e2prom_slct_read 
* 功能描述:  E2PROM型号:CAT24C08
*                     E2PROM SelectiveRead Sequence 读操作
*
* 输入参数: page: 待读取页; addr: 页中的哪个地址; data: 返回数据
*
* 输出参数: None
* 返回值: None
* 备注:  先发送dummy假写操作(假写操作包括先发送从设备地址，
*                  再发送希望读的那一页中的某个地址addr)，
*                接着再发送一次从设备地址，就能读取到对应地址内容。
* 
*************************************************************************/
void e2prom_slct_read(char page, char addr, unsigned char *data)
{
    int i;
    int flag = 0;
    unsigned char val = 0xff;
    unsigned char val_addr = 0xff;
    unsigned short slave_addr = E2PROM_ADDR;

    /* CAT24C08芯片共1024Bytes，需10位address地址位表示。发送数据前的8位地址，
      * 再加芯片I2C硬件地址的a9、a10两位，总共10位地址。
      * 10位地址，高6位表示页，低4位表示页内，因此每页16Bytes大小。*/
    if ((page < 0) || (page > 63))
    {
        HARDWARE_TEST("E2PROM. Write out of page range. Page range: 0~63. Now Write Page address:%d!\n\r", page);
        return;
    }

    if ((addr < 0) || (addr > 15))
    {
        HARDWARE_TEST("E2PROM. Write out of addr at every page range. addr range: 0~0xf. Now Write addr address:0x%x!\n\r", addr);
        return;
    }

    /*8 bit address, addr:0~255 Bytes, page:0~15*/
    if ((page >= 0) && (page < 16))
    {
        val_addr = page * E2PROM_PAGE_SIZE + addr;
    }
    /*超过256Bytes的写，需要把I2C从地址的a8、a9算上。slave address: (1) (0) (1) (0) (A2) (a9) (a8) (R/W)*/
    /*addr:256~511 Bytes, page:16~31*/
    else if ((page >= 16) && (page < 32))
    {
        slave_addr |= 0x1;
        val_addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE + addr;
    }
    /*addr:512~767 Bytes, page:32~47*/
    else if ((page >= 32) && (page < 48))
    {
        slave_addr |= 0x2;
        val_addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE + addr;
    }
    /*addr:768~1023 Bytes, page:48~63*/
    else
    {
        slave_addr |= 0x3;
        val_addr = (page % E2PROM_PAGE_SIZE) * E2PROM_PAGE_SIZE + addr;
    }

    /* 读数据时，需重新设置I2C的待读地址(该地址通过设置从设备地址实现)*/
    i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
    /* I2C_STATUS_TFE=0, TX FIFO不空，循环，直到TX FIFO空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_TFE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("TX FIFO not empty. can't send addr, e2prom select read failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    *I2Cx_DATA_CMD = val_addr;
    *I2Cx_DATA_CMD = (1<<10);
    usleep(10);

    i2cx_SetDevAddr(I2C_E2PROM_NUM, slave_addr);
    *I2Cx_DATA_CMD = 0x100 | (1<<9);

    flag = 0;
    /* I2C_STATUS_RFNE=0, RX FIFO空，循环，直到RX FIFO不空*/
    while(!((*I2Cx_STATUS)&(1 << I2C_STATUS_RFNE)))
    {
        /* 等待3s仍未读到数据返回失败*/
        if (300 <= flag)
        {
            HARDWARE_TEST("RX FIFO empty. can't read data, e2prom select read failed!\n\r");
            return;
        }
        flag++;
        /* 睡眠10ms*/
        usleep(10000);
    }
    usleep(20);

    /* 读取I2C接收的数据*/
    val = *I2Cx_DATA_CMD;
    strncpy(data, &val, 1);

    return;
}

int read_sn(int fd, int eeprom_addr, int read_len, char *data)
{
	int ret;
	struct i2c_rdwr_ioctl_data e2prom_data;
	e2prom_data.nmsgs=2; 
	e2prom_data.msgs=(struct i2c_msg*)malloc(e2prom_data.nmsgs*sizeof(struct i2c_msg));
	if(!e2prom_data.msgs){
		perror("malloc error");
		exit(1);
	}

	/***write data to e2prom**/
	e2prom_data.nmsgs=1;
	(e2prom_data.msgs[0]).len=1; 
	(e2prom_data.msgs[0]).addr=E2PROM_ADDR;
	(e2prom_data.msgs[0]).flags=0; //write
	(e2prom_data.msgs[0]).buf=(unsigned char*)malloc(1);
	(e2prom_data.msgs[0]).buf[0]=0x00;
	//(e2prom_data.msgs[0]).buf[1]=0x56;//the data to write 
	ret=ioctl(fd,I2C_RDWR,(unsigned long)&e2prom_data);
	if(ret<0){
		perror("ioctl error1");
	}
	usleep(5000);
	/******read data from e2prom*******/
	e2prom_data.nmsgs=2;
	(e2prom_data.msgs[0]).len=1; 
	(e2prom_data.msgs[0]).addr=E2PROM_ADDR; 
	(e2prom_data.msgs[0]).flags=0;//write
	(e2prom_data.msgs[0]).buf[0]=eeprom_addr;
	(e2prom_data.msgs[1]).len=read_len;
	(e2prom_data.msgs[1]).addr=E2PROM_ADDR;
	(e2prom_data.msgs[1]).flags=I2C_M_RD;
	(e2prom_data.msgs[1]).buf=(unsigned char*)malloc(read_len);
	memset((e2prom_data.msgs[1]).buf, 0, sizeof(unsigned char*));
	ret=ioctl(fd,I2C_RDWR, (unsigned long)&e2prom_data);
	if(ret<0){
		perror("ioctl error2");
	}
	memcpy(data, (e2prom_data.msgs[1]).buf, read_len);
	return 0;
}

int write_sn(int fd, int eeprom_addr, int write_len, char* data)
{
	int ret;
	struct i2c_rdwr_ioctl_data e2prom_data;
	e2prom_data.nmsgs=2; 
	e2prom_data.msgs=(struct i2c_msg*)malloc(e2prom_data.nmsgs*sizeof(struct i2c_msg));
	if(!e2prom_data.msgs){
		perror("malloc error");
		exit(1);
	}
	/***write data to e2prom**/
	e2prom_data.nmsgs=1;
	(e2prom_data.msgs[0]).len=write_len+1; 
	(e2prom_data.msgs[0]).addr=E2PROM_ADDR;
	(e2prom_data.msgs[0]).flags=0; //write
	(e2prom_data.msgs[0]).buf=(unsigned char*)malloc(write_len+1);
	(e2prom_data.msgs[0]).buf[0]=eeprom_addr;
	memcpy((e2prom_data.msgs[0]).buf+1, data, write_len);
	ret=ioctl(fd,I2C_RDWR,(unsigned long)&e2prom_data);
	if(ret<0){
		perror("ioctl error1");
	}
	usleep(5000);
	return 0;
}

int write_test_result(int fd, int eeprom_addr, int data)
{
	int ret;
	struct i2c_rdwr_ioctl_data e2prom_data;
	e2prom_data.nmsgs=2;
	e2prom_data.msgs=(struct i2c_msg*)malloc(e2prom_data.nmsgs*sizeof(struct i2c_msg));
	if(!e2prom_data.msgs){
		perror("malloc error");
		exit(1);
	}
	/***write data to e2prom**/
	e2prom_data.nmsgs=1;
	(e2prom_data.msgs[0]).len=2;
	(e2prom_data.msgs[0]).addr=E2PROM_ADDR;
	(e2prom_data.msgs[0]).flags=0; //write
	(e2prom_data.msgs[0]).buf=(unsigned char*)malloc(2);
	(e2prom_data.msgs[0]).buf[0]=eeprom_addr;
	(e2prom_data.msgs[0]).buf[1]=data;
	ret=ioctl(fd,I2C_RDWR,(unsigned long)&e2prom_data);
	if(ret<0){
		perror("ioctl error1");
	}
	usleep(5000);
	return 0;
}

void init_i2c_ioctrl(int fd)
{
	/* 100k */
	int speed = 1;
	ioctl(fd, I2C_SLAVE, E2PROM_ADDR);
	ioctl(fd, I2C_XFERSPEED, speed);
	ioctl(fd, I2C_TENBIT,0);
	ioctl(fd, I2C_TIMEOUT,1);
	ioctl(fd, I2C_RETRIES,1);
}
/************************************************************************
* 函数名: sensor_mmc34160pj_test 
* 功能描述:  M-sensor
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void sensor_mmc34160pj_test(void)
{
    unsigned char device_id = 0;

    HARDWARE_TEST("Begin sensor MMC34160PJ test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, MMC34160PJ_address);
    usleep(10);
    /* 读Device ID 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x20);
    
    /* 读取的数据相同测试成功*/
    if (0x06 == device_id)
    {
        HARDWARE_TEST("sensor MMC34160PJ test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor MMC34160PJ test Failed!\n\rRead sensor device ID:0x%x(real ID:0x06)", device_id);
    }

    return;
}

/************************************************************************
* 函数名: sensor_l3gd20_test 
* 功能描述:  Gyroscope
*
* 输入参数: None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void sensor_l3gd20_test(void)
{
    unsigned char device_id = 0;
    
    HARDWARE_TEST("Begin sensor L3GD20 test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, L3GD20_address);
    /* 读WHO_AM_I 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x0f);
    usleep(10);
    
    //相同测试成功
    if((0xD4 == device_id) || (0xD7 == device_id))
    {
        HARDWARE_TEST("sensor L3GD20 test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor L3GD20 test Failed!\n\rRead sensor device ID:0x%x(real ID:0xD4 or 0xD7)", device_id);
    }

    return;
}

/************************************************************************
* 函数名: sensor_mma8653fcr1_test 
* 功能描述:  G-sensor
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void sensor_mma8653fcr1_test(void)
{
    unsigned char device_id = 0;

    HARDWARE_TEST("Begin sensor MMA8653FCR1 test!\n\r");
    i2cx_SetDevAddr(I2C_SENSOR_NUM, MMA8653FCR1_address);
    /* 读Device ID 地址的数据*/
    device_id = i2cx_ReadReg(I2C_SENSOR_NUM, 0x0D);
    
    //相同测试成功
    if (0x5A == device_id)
    {
        HARDWARE_TEST("sensor MMA8653FCR1 test Success!\n\r");
    }
    else
    {
        HARDWARE_TEST("sensor MMA8653FCR1 test Failed!\n\rRead sensor device ID:0x%x(real ID:0x5A)", device_id);
    }

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_init 
* 功能描述:  lc1160初始化
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_init(void)
{
    /*set com_i2c mode*/
    /* 0: com_i2c_scl; 1:  保留; 2: gpio[88]*/
    PIN_Mux_set(COM_I2C_SCL, 0);
    PIN_Mux_set(COM_I2C_SDA, 0);
    /* COM I2C 配置速率为100K*/
    i2cx_init(I2C_PCMCODEC_NUM, I2C_SS_SPEED);
    /* 设置访问1160的I2C地址*/
    i2cx_SetDevAddr(I2C_PCMCODEC_NUM, I2C_LC1160_addr);
    usleep(10);
    // i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x01, 0x64);
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x0b, 0x64);

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_test 
* 功能描述:  lc1160测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_test(void)
{
    float val = 0;
    int i =0;
    unsigned char data = 0;
    unsigned short adc = 0;

    /* set com_i2c mode*/
    /* 0: com_i2c_scl; 1:  保留; 2: gpio[88]*/
    PIN_Mux_set(COM_I2C_SCL, 0);
    PIN_Mux_set(COM_I2C_SDA, 0);
    /* 100K*/
    i2cx_init(I2C_PCMCODEC_NUM, I2C_SS_SPEED);
    /* 设置访问1160的I2C地址*/
    i2cx_SetDevAddr(I2C_PCMCODEC_NUM, I2C_LC1160_addr);
    usleep(10);

    /* 使能Sink1，PRE on*/
    pmu_lc1160_sink_set(1, 1);
    /* 使能Sink2，VIB on*/
    pmu_lc1160_sink_set(2, 1);
    
    data = i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x95);
    data |= (0x1<<6);
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x95, data);
    usleep(2000);

    /* 去使能Sink2，VIB off*/
    pmu_lc1160_sink_set(2, 0);
    /* 使能Sink3，FLASH on*/
    pmu_lc1160_sink_set(3, 1);
    usleep(1000);
    /* 去使能Sink3，FLASH off*/
    pmu_lc1160_sink_set(3, 0);
    
    val = pmu_lc1160_adc_read(ADC_VBAT, &adc);
    /* 打印ADC_VBAT电压值*/
    HARDWARE_TEST("ADC_VBAT = %.2fV;\n\r", val);

    /* 测试RTC*/
    HARDWARE_TEST("%dYear    %dMonth    %dDay    %dHour    %dMinutes\n\r",
                                    (pmu_lc1160_rtc_read(6)),(pmu_lc1160_rtc_read(5)),
                                    (pmu_lc1160_rtc_read(3)),(pmu_lc1160_rtc_read(2)),(pmu_lc1160_rtc_read(1)));

    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x49, 0x01);
    /* ALDO4=1.8V*/
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x30, 0xC1);

    for(i=1; i<16; i++)
    {
        pmu_lc1160_aldo_en(i, 1);
    }
    HARDWARE_TEST("All ALDOs have been opened.\n\r");

    for(i=1; i<12; i++)
    {
        pmu_lc1160_dldo_en(i, 1);
    }
    HARDWARE_TEST("All DLDOs have been opened.\n\r");

    for(i=7; i<10; i++)
    {
        pmu_lc1160_buck_en(i, 1);
    }
    HARDWARE_TEST("All BUCKs have been opened.\n\r");

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_aldo_en 
* 功能描述:  lc1160 aldo 测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_aldo_en(int ALDO_num, int enable)
{
    unsigned char data = 0;
    unsigned char reg = 0;

    reg = 0x29;
    data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
    data&= ~(1<<3);
    data &= ~(1<<4);
    usleep(100);
    i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);

    if (enable == 1)
    {
        /* ALDO1-2*/
        if ((0<ALDO_num) && (ALDO_num<3))
        {
            reg = 0x2b + ALDO_num - 1;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data |= (1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        /* ALDO3*/
        else if(ALDO_num == 3)
        {
            reg = 0x2e;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data |= (1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        /* ALDO15-4*/
        else if((3<ALDO_num) && (ALDO_num<16))
        {
            reg = 0x30 + ALDO_num - 4;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data |= (1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        else
        {
            HARDWARE_TEST("LC1160 ALDO num is 1~15, you set out of rang!\n\r");
        }
    }
    else if (enable ==0)
    {
        /* ALDO1-2*/
        if ((0<ALDO_num) && (ALDO_num <3))
        {
            reg = 0x2b + ALDO_num - 1;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data &= ~(1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        /* ALDO3*/
        else if( ALDO_num == 3)
        {
            reg = 0x2e;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data &= ~(1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        /* ALDO15-4*/
        else if((3<ALDO_num)&&(ALDO_num<16))
        {
            reg = 0x30 + ALDO_num - 4;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data &= ~(1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        else
        {
            HARDWARE_TEST("LC1160 ALDO num is 1~15, you set out of rang!\n\r");
        }
    }
    else
    {
        HARDWARE_TEST("Sorry! LC1160 ALDO parameter error!\n\r");
    }

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_dldo_en 
* 功能描述:  lc1160 dldo测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_dldo_en(int DLDO_num, int enable)
{
    unsigned char data = 0;
    unsigned char reg = 0;
    
    if (enable == 1)
    {
        if ((0<DLDO_num) && (DLDO_num <12))//ALDO1-11
        {
            reg = 0x3E + DLDO_num - 1;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data |= (1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        else
        {
            HARDWARE_TEST("Sorry! LC1160 DLDO num set available!\n\r");
        }
    }
    else if (enable == 0)
    {
        if ((0<DLDO_num) && (DLDO_num <12))//ALDO1-11
        {
            reg = 0x3E + DLDO_num - 1;
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data &= ~(1<<7);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        else
        {
            HARDWARE_TEST("Sorry! LC1160 DLDO num set available!\n\r");
        }
    }
    else
    {
        HARDWARE_TEST("Sorry! LC1160 DLDO parameter available!\n\r");
    }

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_buck_en 
* 功能描述:  lc1160 buck测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_buck_en(int BUCK_num, int enable)
{
    unsigned char data = 0;
    unsigned reg = 0;
    unsigned reg_buck7 = 0x11;

    if((BUCK_num<10) && (BUCK_num>6))
    {
        if (enable == 1)
        {
            reg = reg_buck7 + 2*(BUCK_num-7);
            data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);
            data |= (1 << 6);
            usleep(100);
            i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        }
        else if (enable == 0)
        {
            HARDWARE_TEST("Sorry! no disable function available.\n\r");
        }
        else
        {
            HARDWARE_TEST("Sorry! parameter available.\n\r");
        }
    }
    else
    {
        HARDWARE_TEST("Sorry! parameter available.\n\r");
    }

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_sink_set 
* 功能描述:  控制SINK使能
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_sink_set(int sink_num, int enable)
{
    unsigned char data = 0;

    /* 使能sink*/
    if (enable == 1)
    {
        data = i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x52);
        data |= (0x5<<4);// 4mA
        i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x52, data);//SINK1 current PRE

        data = i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x53);
        data |= (0x0a<<0) | (0x5<<4);// 40mA  50mA
        i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x53, data);//SINK2,3 current VIB FLASH

        data = i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x51);
        data |= (1 << (sink_num));
        i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x51, data);//SINK en
    }
    /* 关闭sink*/
    else if (enable == 0)
    {
        data = i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x51);
        data &= ~(1 << (sink_num));
        i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x51, data);//SINK off
    }
    else
    {
        HARDWARE_TEST("Sorry! parameter available.\n\r");
    }

    return;
}

/************************************************************************
* 函数名: pmu_lc1160_adc_read 
* 功能描述:  选择通道，读ADC值。2次读之间至少间隔80us，转换时间1us
*
* 输入参数:unsigned int channel: 待转换ADC通道号
*                           int* adc: 转换后的ADC值
*
* 输出参数: None
* 返回值: 转换后的模拟电压值
* 备注:  
* 
*************************************************************************/
float pmu_lc1160_adc_read(unsigned int channel, unsigned short *padc)
{
    float val = 0;
    int times = 0;
    unsigned short u16_data = NULL;
    unsigned char u8_data = 0;

    i2cx_init(I2C_PCMCODEC_NUM, I2C_SS_SPEED);
    i2cx_SetDevAddr(I2C_PCMCODEC_NUM, I2C_LC1160_addr);

    u16_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1);
    u16_data |= (1 << 0);
    /* adc LDO en*/
	/* 0x87  12.1.12.1 ADCCR1: A/D Converter Control Register1 */
    i2cx_WriteReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1, u16_data);
    usleep(1000);

    /* select adc channel*/
	/* 0x88  12.1.12.2 ADCCR2: A/D Converter Control Register2 */
    i2cx_WriteReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR2, channel);

    u16_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1);
    u16_data |= (1 << 1);
    /* enable adc*/
    i2cx_WriteReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1, u16_data);

    u16_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1);
    u16_data |= (1 << 3);
    /* start adc*/
    i2cx_WriteReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1, u16_data);

    do
    {
        u16_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCCR1);
        u16_data = u16_data & 0x10;
        /* 读10次，仍未读到数据返回失败*/
        if (10 <= times)
        {
            HARDWARE_TEST("ADC conversion failed!\n\r");
            return 0;
        }
        times++;
    }
    while(!u16_data);

    HARDWARE_TEST("ADC conversion is done!\n\r");

    u16_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCDAT1); //high 8 bits
    u16_data = u16_data << 8;
    u8_data = i2cx_ReadReg(I2C_PCMCODEC_NUM, LC1160_REG_ADCDAT0); //low 4bits
    u8_data = u8_data << 4;
    u16_data |= u8_data;
    u16_data = u16_data >> 4;

    HARDWARE_TEST("ADC data is %d.\n\r", u16_data);
    *padc = u16_data;

    if (ADC_VBAT == channel)
    {
        val = 2.8*4*u16_data/4095;
    }
    else
    {
        val =  2.8*u16_data/4095;
    }

    /* 等待80us后退出*/
    usleep(80);
    
    return val;
}

/************************************************************************
* 函数名: pmu_lc1160_test 
* 功能描述:  读取RTC寄存器，//i = 0-秒 1-分 2-时 3-日 4-月 5-年 6-星期
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
unsigned char pmu_lc1160_rtc_read(int input)
{
    int val = 0;
    if((input>=0) && (input <= 6))
    {
        val = 0x60 + input;
        return i2cx_ReadReg(I2C_PCMCODEC_NUM, val);
    }
    
    return ERROR;
}

/************************************************************************
* 函数名: pmu_lc1160_reg_set_test 
* 功能描述:  lc1160测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void pmu_lc1160_reg_set_test(void)
{
    char rxbuf[MAX_KEY_BYTES] = {0};
    int num = 0;
    int item_num = 0;

    /* Main Content*/
    HARDWARE_TEST("*****************************************************************\n\r");
    HARDWARE_TEST("*******************<<LC1160 PMU REG SET TEST>>********************\n\r");
    while(1)
    {
        HARDWARE_TEST("*****************************************************************\n\r");
        HARDWARE_TEST("*****************1-Read Reg   2-Write Reg   Q(q)-Quit*******************\n\r");

        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        if (1 == num)
        {
            /*输入Q(q)，退出lc1160 reg set测试*/
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit lc1160 reg set test!\n\r");
                break;
            }
            /* 不是数字字符'1'、'2'*/
            else if ((rxbuf[0] != 0x31) && (rxbuf[0] != 0x32))
            {
                HARDWARE_TEST("Input error!please again\n\r");
                continue;
            }
        }
        else
        {
            HARDWARE_TEST("Input error!please again\n\r");
            continue;
        }

        /* 将ASCII码转换为十六进制*/
        asctohex(rxbuf, &item_num, num);
        switch(item_num)
        {
            case 1:
            {
                lc1160_read_reg();
                break;
            }
            case 2:
            {
                lc1160_write_reg();
                break;
            }
            default:
            {
                HARDWARE_TEST("Input data is out of range! Please Re-Input:\n\r");
            }
        }
    }
    HARDWARE_TEST("************************END OF LC1160 PMU REG SET TEST************************\n\r");

    return;
}

void split_str(char *src, const char *separator, char **dest, int *num)
{
    /*
     *src: str to cut, from /proc/dtvl/hd_version
     *separator: symbol of cut
     *dest: store str after cut
     *num: num of str after cut
     */
    char *pnext;
    int count = 0;

    if ((src==NULL) || (strlen(src)==0))
    {
        return ;
    }

    if ((separator==NULL) || (strlen(separator)==0))
    {
        return ;
    }

    pnext = (char *)strtok(src, separator);
    while(pnext!=NULL)
    {
        *dest++ = pnext;
        ++count;
        pnext = (char *)strtok(NULL, separator);
    }
    *num = count;
}
int split_ttyUSB_num()
{
	int i;
	FILE *fp;
	char buffer[200];
	char *revbuf[8] = {0};
	char *sec_revbuf[50] = {0};

	int num = 0;
	int sec_num = 0;
	int nloop;
	int str_value;
	int value_adci1;

	system("ls /dev/ttyUSB* > ttyUSB.log");
	fp = fopen("ttyUSB.log", "r");
	fread(buffer, 200, 1, fp);
	HARDWARE_TEST("buffer:\n**************\n%s\n***************\n", buffer);

	split_str(buffer,"\n",revbuf,&num);

    for(i = 0;i < num; i ++) {
        if(strstr(revbuf[i], "ttyUSB1") != NULL)
        {
            str_value = i+1;
            HARDWARE_TEST("\nfind %s in num %d\n", revbuf[i], i);
            break;
        }
    }

	split_str(revbuf[str_value],"USB",sec_revbuf,&sec_num);
	for(i = 0;i < sec_num; i ++)
	{
        if(strstr(sec_revbuf[i], "tty") != NULL)
        {
            str_value = i+1;
        }
    }

	HARDWARE_TEST("find current tty value %s\n", sec_revbuf[str_value]);
	fclose(fp);
	return atoi(sec_revbuf[str_value]);
}

int get_adci_from_device(unsigned short *adci_get)
{
    int i;
    unsigned short adci_value = ~0;
    FILE *fp;
    char buffer[ADCI_BUFFER_SIZE];

    char *revbuf[8] = {0};
    char *sec_revbuf[8] = {0};
    int num = 0;
    int sec_num = 0;
    int str_adci1;

    if (0 == access("/proc/dtvl/hd_version", F_OK))
    {
        /* read adci value from proc_file */
        fp = fopen("/proc/dtvl/hd_version", "r");
        fread(buffer, ADCI_BUFFER_SIZE, 1, fp);
        fclose(fp);
		HARDWARE_TEST("read hd_version from file:\n%s\n", buffer);

		/* find adci value from read_buffer */
	    split_str(buffer, " ", revbuf, &num);
		for(i=0; i<num; i++)
		{
			if(strstr(revbuf[i], "adci1") != NULL)
			{
				str_adci1 = i;
				HARDWARE_TEST("find str_adci1 in str_%d\n", i);
			}
		}

	    split_str(revbuf[str_adci1],"\t",sec_revbuf,&sec_num);
		for(i=0; i<sec_num; i++)
		{
			if(strstr(sec_revbuf[i], "adci1") != NULL)
			{
				str_adci1 = i+1;
			}
			if(sec_revbuf[str_adci1] == NULL)
			{
				HARDWARE_TEST("find adci value NULL\n\n");
				return -1;
			}else{
				HARDWARE_TEST("find adci value %s\n\n", sec_revbuf[str_adci1]);
			}
		}

	    /*num3 == adci1_value, num4 == adci2_value*/
	    HARDWARE_TEST("adci1 value from %s: %d\n", __FUNCTION__, atoi(sec_revbuf[str_adci1]));
	    adci_value = atoi(sec_revbuf[str_adci1]);
	    *adci_get = adci_value*4096/2800;
	    HARDWARE_TEST("adci1 value %s: %d conver %d\n", __FUNCTION__, adci_value, *adci_get);
		return 0;
	}
	else
	{
		HARDWARE_TEST("open /proc/dtvl/hd_version fail\n");
		return -1;
	}
}
/************************************************************************
* 函数名: lc1160_read_reg 
* 功能描述:  lc1160读寄存器测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void lc1160_read_reg(void)
{
    char rxbuf[MAX_KEY_BYTES] = {0};
    int num = 0;
    int item_num = 0;
    int i =0;
    unsigned char reg = 0;
    unsigned char data = 0;

    while(1)
    {
        HARDWARE_TEST("input the register(hex) you want to read(the characters of input, will be regard as hexadecimal);press Q(q) to exit.\n\r");
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出lc1160 read测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit lc1160 reg read test!\n\r");
                break;
            }
        }

        /* 其他字符键*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 字符'A~F'*/
            else if ((rxbuf[i] >= 0x41) && (rxbuf[i] <= 0x46))
            {
                continue;
            }
            /* 字符'a~f'*/
            else if (((rxbuf[i] >= 0x61) && (rxbuf[i] <= 0x66)))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input lc1160 reg read test number ERROR!Please input again!\n\r");
                break;
            }
        }

        /* 判断输入的reg item参数*/
        if (i != num)
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        /* 将ASCII码转换为十六进制*/
        asctohex(rxbuf, &item_num, num);
        /* 超过LC1160寄存器最大范围*/
        if (item_num > 0xFF)
        {
            HARDWARE_TEST("Input register(0x%x) out of range!\n\r", item_num);
            continue;
        }
        reg = (unsigned char)item_num;

        data = i2cx_ReadReg(I2C_PCMCODEC_NUM, reg);

        HARDWARE_TEST("reg 0x%x=0x%x.\n\r", reg, data);
    }

    return;
}

/************************************************************************
* 函数名: lc1160_write_reg 
* 功能描述:  lc1160写寄存器测试
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void lc1160_write_reg(void)
{
    char rxbuf[MAX_KEY_BYTES] = {0};
    int num = 0;
    int item_num = 0;
    int i =0;
    unsigned char reg = 0;
    unsigned char data = 0;

    while(1)
    {
        /* 输入待写入的寄存器*/
        HARDWARE_TEST("Pls input reg address(hex);press Q(q) to exit.\n\r");
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出reg write测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit lc1160 reg write test!\n\r");
                break;
            }
        }

        /* 其他字符键*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 字符'A~F'*/
            else if ((rxbuf[i] >= 0x46) && (rxbuf[i] <= 0x41))
            {
                continue;
            }
            /* 字符'a~f'*/
            else if (((rxbuf[i] >= 0x66) && (rxbuf[i] <= 0x61)))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input lc1160 reg write test number ERROR!Please input again!\n\r");
                break;
            }
        }

        /* 判断输入的reg item参数*/
        if (i != num)
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        /* 将ASCII码转换为十六进制*/
        asctohex(rxbuf, &item_num, num);
        /* 超过LC1160寄存器最大范围*/
        if (item_num > 0xFF)
        {
            HARDWARE_TEST("Input register(0x%x) out of range!\n\r", item_num);
            continue;
        }
        reg=(unsigned char)item_num;



        /* 输入待写入寄存器的值*/
        HARDWARE_TEST("Pls input data (hex);press Q(q) to exit.\n\r");
        /* 阻塞读取控制台console输入的字符*/
        num = read(STDIN_FILENO, rxbuf, MAX_KEY_BYTES);
        if(num < 0)
        {
            HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
            continue;
        }
        /* 只取控制台读到的0~num-1个字符，从而删除回车键*/
        num = num-1;
        if(0 == num)
        {
            HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
            continue;
        }

        /*输入Q(q)，退出reg write测试*/
        if (1 == num)
        {
            if ((rxbuf[0] == 0x51) || (rxbuf[0] == 0x71))
            {
                HARDWARE_TEST("Quit lc1160 reg write test!\n\r");
                break;
            }
        }

        /* 其他字符键*/
        for (i = 0; i < num; i++)
        {
            /* 数字字符'0~9'*/
            if ((rxbuf[i] >= 0x30) && (rxbuf[i] <= 0x39))
            {
                continue;
            }
            /* 字符'A~F'*/
            else if ((rxbuf[i] >= 0x46) && (rxbuf[i] <= 0x41))
            {
                continue;
            }
            /* 字符'a~f'*/
            else if (((rxbuf[i] >= 0x66) && (rxbuf[i] <= 0x61)))
            {
                continue;
            }
            /* 输入字符错误*/
            else
            {
                HARDWARE_TEST("Your input lc1160 reg write test number ERROR!Please input again!\n\r");
                break;
            }
        }

        /* 判断输入的data item参数*/
        if (i != num)
        {
            HARDWARE_TEST("Input the format of parameter Error!Please input again!\n\r");
            continue;
        }
        /* 将ASCII码转换为十六进制*/
        asctohex(rxbuf, &item_num, num);
        /* 超过LC1160寄存器最大范围*/
        if (item_num > 0xFF)
        {
            HARDWARE_TEST("Input data(0x%x) out of range!\n\r", item_num);
            continue;
        }
        data = (unsigned char)item_num;

        i2cx_WriteReg(I2C_PCMCODEC_NUM, reg, data);
        HARDWARE_TEST("\nafter writing, reg 0x%x=0x%x.\n\r", reg, data);
    }

    return;
}

/************************************************************************
* 函数名: ssix_reg_init 
* 功能描述:  SSIx寄存器地址赋值
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void ssix_reg_init(int ssix)
{
    if (ssix == SSI_0_NUM)
    {
        SSIx_CTRL0 = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_CTRL0);
        SSIx_CTRL1 = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_CTRL1);
        SSIx_EN = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_EN);
        SSIx_SE = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_SE);
        SSIx_BAUD = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_BAUD);
        SSIx_TXFTL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_TXFTL);
        SSIx_RXFTL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_RXFTL);
        SSIx_TXFL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_TXFL);
        SSIx_RXFL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_RXFL);
        SSIx_STS = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_STS);
        SSIx_IE = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_IE);
        SSIx_IS = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_IS);
        SSIx_RIS = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_RIS);
        SSIx_TXOIC = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_TXOIC);
        SSIx_RXOIC = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_RXOIC);
        SSIx_RXUIC = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_RXUIC);
        SSIx_IC = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_IC);
        SSIx_DMAC = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_DMAC);
        SSIx_DMATDL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_DMATDL);
        SSIx_DMARDL = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_DMARDL);
        SSIx_DATA = (volatile unsigned *)(SSI0_MODEULE_BASE + SSI0_DATA);
        CTL_SSIx_PROTOCOL_CTRL = (volatile unsigned *)(AP_CTL_MODEULE_BASE + CTL_SSI0_PROTOCOL_CTRL);
    }
    else if (ssix == SSI_1_NUM)
    {
        SSIx_CTRL0 = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_CTRL0);
        SSIx_CTRL1 = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_CTRL1);
        SSIx_EN = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_EN);
        SSIx_SE = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_SE);
        SSIx_BAUD = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_BAUD);
        SSIx_TXFTL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_TXFTL);
        SSIx_RXFTL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_RXFTL);
        SSIx_TXFL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_TXFL);
        SSIx_RXFL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_RXFL);
        SSIx_STS = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_STS);
        SSIx_IE = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_IE);
        SSIx_IS = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_IS);
        SSIx_RIS = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_RIS);
        SSIx_TXOIC = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_TXOIC);
        SSIx_RXOIC = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_RXOIC);
        SSIx_RXUIC = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_RXUIC);
        SSIx_IC = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_IC);
        SSIx_DMAC = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_DMAC);
        SSIx_DMATDL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_DMATDL);
        SSIx_DMARDL = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_DMARDL);
        SSIx_DATA = (volatile unsigned *)(SSI1_MODEULE_BASE + SSI1_DATA);
        CTL_SSIx_PROTOCOL_CTRL = (volatile unsigned *)(AP_CTL_MODEULE_BASE + CTL_SSI1_PROTOCOL_CTRL);
    }
    else if (ssix == SSI_2_NUM)
    {
        SSIx_CTRL0 = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_CTRL0);
        SSIx_CTRL1 = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_CTRL1);
        SSIx_EN = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_EN);
        SSIx_SE = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_SE);
        SSIx_BAUD = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_BAUD);
        SSIx_TXFTL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_TXFTL);
        SSIx_RXFTL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_RXFTL);
        SSIx_TXFL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_TXFL);
        SSIx_RXFL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_RXFL);
        SSIx_STS = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_STS);
        SSIx_IE = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_IE);
        SSIx_IS = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_IS);
        SSIx_RIS = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_RIS);
        SSIx_TXOIC = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_TXOIC);
        SSIx_RXOIC = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_RXOIC);
        SSIx_RXUIC = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_RXUIC);
        SSIx_IC = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_IC);
        SSIx_DMAC = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_DMAC);
        SSIx_DMATDL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_DMATDL);
        SSIx_DMARDL = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_DMARDL);
        SSIx_DATA = (volatile unsigned *)(SSI2_MODEULE_BASE + SSI2_DATA);
        CTL_SSIx_PROTOCOL_CTRL = (volatile unsigned *)(AP_CTL_MODEULE_BASE + CTL_SSI2_PROTOCOL_CTRL);
    }
    else
    {
        HARDWARE_TEST("SSIx_Reg_init: SSI NUM out of range!\n\r");
    }

    return;
}

/************************************************************************
* 函数名: ssix_gpio_config
* 功能描述:  配置ssix管脚作ssi功能
*
* 输入参数:ssi端口号
*
* 输出参数: None
* 返回值: None
* 备注:  ssix_ssn: 帧同步信号
* 
*************************************************************************/
void ssix_gpio_config(int ssix)
{
    if (ssix == SSI_0_NUM)
    {
        /* GPIO157:ssi0_tx; GPIO158:ssi0_rx; GPIO159:ssi0_clk; GPIO160:ssi0_ssn(chip select)*/
        PIN_Mux_set(SSI0_TX, 0);
        PIN_Mux_set(SSI0_RX, 0);
        PIN_Mux_set(SSI0_CLK, 0);
        PIN_Mux_set(SSI0_SSN, 0);
    }
    else if (ssix == SSI_1_NUM)
    {
         /* GPIO219:ssi1_tx; GPIO220:ssi1_rx; GPIO221:ssi1_clk; GPIO222:ssi1_ssn(chip select)*/
        PIN_Mux_set(SSI1_TX, 0);
        PIN_Mux_set(SSI1_RX, 0);
        PIN_Mux_set(SSI1_CLK, 0);
        PIN_Mux_set(SSI1_SSN, 0);
    }
    else if (ssix == SSI_2_NUM)
    {
         /* GPIO161:ssi2_tx; GPIO162:ssi2_rx; GPIO163:ssi2_clk; GPIO164:ssi2_ssn(chip select)*/
        PIN_Mux_set(SSI2_TX, 0);
        PIN_Mux_set(SSI2_RX, 0);
        PIN_Mux_set(SSI2_CLK, 0);
        PIN_Mux_set(SSI2_SSN, 0);
    }

    return;
}

/************************************************************************
* 函数名: ssix_clock_enable
* 功能描述:  使能ssix时钟
*
* 输入参数:ssi端口号
*
* 输出参数: None
* 返回值: None
* 备注:
* 
*************************************************************************/
void ssix_clock_enable(int ssix)
{
    unsigned int val = 0;
    unsigned char mclk_bit, mclk_we_bit, ifclk_bit, ifclk_we_bit;

    if (ssix == SSI_0_NUM)
    {
        mclk_bit = 12;
        mclk_we_bit = 28;
        ifclk_bit = 0;
        ifclk_we_bit = 16;
    }
    else if (ssix == SSI_1_NUM)
    {
        mclk_bit = 13;
        mclk_we_bit = 29;
        ifclk_bit = 1;
        ifclk_we_bit = 17;
    }
    else if (ssix == SSI_2_NUM)
    {
        mclk_bit = 14;
        mclk_we_bit = 30;
        ifclk_bit = 2;
        ifclk_we_bit = 18;
    }
    else
    {
        HARDWARE_TEST("SSIx_clock_enable: Parameter err, SSI NUM out of range!\n\r");
        return;
    }

    /* enable SSIx clk*/
    if ((volatile unsigned *)(AP_PWR_MODEULE_BASE +AP_PWR_SSICLK_CTL0) != 0)
    {
         /* 使能ssi时钟ssix_mclk */
        val =  (1 << mclk_bit) | (1 << mclk_we_bit);
        ssix_WriteReg((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SSICLK_CTL0), val);
    }

    /* 使能ssix_pclk*/
    if (2 != ssix)
    {
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE +AP_PWR_DATAPCLK_EN) != 0)
        {
            val = (1 << ifclk_bit) | (1 << ifclk_we_bit);
            ssix_WriteReg((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_DATAPCLK_EN), val);
        }
    }
    else
    {
        if ((volatile unsigned *)(AP_PWR_MODEULE_BASE +AP_PWR_SECPCLK_EN) != 0)
        {
            val = (1 << ifclk_bit) | (1 << ifclk_we_bit);
            ssix_WriteReg((volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SECPCLK_EN), val);
        }
    }

    /* ssix_mclk 频 率 为Freq(pll1_mclk) * ((grp + 1)/8) / (dvi + 1) */
    HARDWARE_TEST("SSI%d enable clk.\n\r", ssix);

    return;
}

/************************************************************************
* 函数名: spix_dump_regs
* 功能描述:   待读取ssi号
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:
* 
*************************************************************************/
void spix_dump_regs(int ssix)
{
    HARDWARE_TEST("SSI%d CTRL0 : 0x%08x\n", ssix, ssix_ReadReg(SSIx_CTRL0));
    HARDWARE_TEST("SSI%d CTRL1 : 0x%08x\n", ssix, ssix_ReadReg(SSIx_CTRL1));
    HARDWARE_TEST("SSI%d EN : 0x%08x\n", ssix, ssix_ReadReg(SSIx_EN));
    HARDWARE_TEST("SSI%d SE : 0x%08x\n", ssix, ssix_ReadReg(SSIx_SE));
    HARDWARE_TEST("SSI%d BAUD : 0x%08x\n", ssix, ssix_ReadReg(SSIx_BAUD));
    HARDWARE_TEST("SSI%d TXFTL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_TXFTL));
    HARDWARE_TEST("SSI%d RXFTL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_RXFTL));
    HARDWARE_TEST("SSI%d TXFL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_TXFL));
    HARDWARE_TEST("SSI%d RXFL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_RXFL));
    HARDWARE_TEST("SSI%d STS : 0x%08x\n", ssix, ssix_ReadReg(SSIx_STS));
    HARDWARE_TEST("SSI%d IE : 0x%08x\n", ssix, ssix_ReadReg(SSIx_IE));
    HARDWARE_TEST("SSI%d IS : 0x%08x\n", ssix, ssix_ReadReg(SSIx_IS));
    HARDWARE_TEST("SSI%d RIS : 0x%08x\n", ssix, ssix_ReadReg(SSIx_RIS));
    HARDWARE_TEST("SSI%d TXOIC : 0x%08x\n", ssix, ssix_ReadReg(SSIx_TXOIC));
    HARDWARE_TEST("SSI%d RXOIC: 0x%08x\n", ssix, ssix_ReadReg(SSIx_RXOIC));
    HARDWARE_TEST("SSI%d RXUIC : 0x%08x\n", ssix, ssix_ReadReg(SSIx_RXUIC));
    HARDWARE_TEST("SSI%d IC : 0x%08x\n", ssix, ssix_ReadReg(SSIx_IC));
    HARDWARE_TEST("SSI%d DMAC : 0x%08x\n", ssix, ssix_ReadReg(SSIx_DMAC));
    HARDWARE_TEST("SSI%d DMATDL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_DMATDL));
    HARDWARE_TEST("SSI%d DMARDL : 0x%08x\n", ssix, ssix_ReadReg(SSIx_DMARDL));
    HARDWARE_TEST("CTL_SSI%d_PROTOCOL_CTRL: 0x%08x\n", ssix, ssix_ReadReg(CTL_SSIx_PROTOCOL_CTRL));

    return;
}

/************************************************************************
* 函数名: ssix_WriteReg
* 功能描述:   往SSI的register寄存器写值
*
* 输入参数:*p_reg:待写寄存器; data:待写数据
*
* 输出参数: None
* 返回值: None
* 备注:
* 
*************************************************************************/
void ssix_WriteReg(volatile unsigned *p_reg, int data)
{
    *p_reg = data;

    return;
}

/************************************************************************
* 函数名: ssix_ReadReg
* 功能描述:   读取register寄存器值
*
* 输入参数:*p_reg:待写寄存器
*
* 输出参数: None
* 返回值: 返回读取的数据
* 备注:
* 
*************************************************************************/
int ssix_ReadReg(volatile unsigned *p_reg)
{
    int val;
    val = *p_reg;
    
    return  val;
}

/************************************************************************
* 函数名: reset_can
* 功能描述:   给MCP25625芯片发送reset指令INSTRUCTION_RESET
*
* 输入参数:
*
* 输出参数: None
* 返回值: None
* 备注:
* 
*************************************************************************/
void reset_can(int ssix)
{
    int val = 0;

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送reset指令INSTRUCTION_RESET*/
    ssix_WriteReg(SSIx_DATA, INSTRUCTION_RESET);

    return;
}

/************************************************************************
* 函数名: read_reg_can
* 功能描述:   读取MCP25625芯片register寄存器值
*                             需先发送读寄存器的指令INSTRUCTION_READ，
*                             再发送读的寄存器地址reg
*                             最后再读取寄存器返回的值
*                             一次只能发送1个字节(8bit)数据，一次大于1字节，将发送失败
*
* 输入参数:ssix:待读ssi号; reg:待读取的寄存器地址;
*
* 输出参数: None
* 返回值: 返回读取的寄存器存储值
* 备注:
* 
*************************************************************************/
int read_reg_can(int ssix, int reg)
{
    int val = 0;

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送读指令INSTRUCTION_READ 和寄存器reg*/
    ssix_WriteReg(SSIx_DATA, INSTRUCTION_READ);

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送读指令INSTRUCTION_READ 和寄存器reg*/
    ssix_WriteReg(SSIx_DATA, (reg & 0xFF));

    /* 按照spi协议，随便发送一个字节，才有时钟，才能读取一个字节
        时钟在数据不发送时，无时钟信号*/
    val = *SSIx_STS;
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = *SSIx_STS;
    }
    ssix_WriteReg(SSIx_DATA, 0xFF);

    val = *SSIx_STS;
    while(!(SSI_RFNE & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = *SSIx_STS;
    }
    val = *SSIx_DATA;

    HW_LOG("Read MCP25625 register 0x%x = 0x%08x.\n\r", reg, val);

    return val;
}

/************************************************************************
* 函数名: write_reg_can
* 功能描述:   写MCP25625芯片register寄存器
*                             需先发送写寄存器的指令INSTRUCTION_WRITE，
*                             再发送待写的寄存器地址reg
*                             最后再发送待写入的寄存器的值
*
* 输入参数:ssix:待读ssi号; reg:待读取的寄存器地址; data:待写入寄存器的值
*
* 输出参数: None
* 返回值: None
* 备注:
* 
*************************************************************************/
void write_reg_can(int ssix, int reg, int data)
{
    int val = 0;

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送写指令INSTRUCTION_WRITE*/
    ssix_WriteReg(SSIx_DATA, INSTRUCTION_WRITE);

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送待写入的MCP25625寄存器*/
    ssix_WriteReg(SSIx_DATA, (reg & 0xFF));

    /* 判断发送FIFO是否可用*/
    val = ssix_ReadReg(SSIx_STS);
    while(!(SSI_TFNF & val))
    {
        HARDWARE_TEST("Read SSI%d_STS register = 0x%08x, tx fifo filled.\n\r", ssix, val);
        usleep(10);
        val = ssix_ReadReg(SSIx_STS);
    }
    /* 发送待写入的MCP25625寄存器值*/
    ssix_WriteReg(SSIx_DATA, data);
    
    return;
}

/************************************************************************
* 函数名: power_off 
* 功能描述:  关机,PWEN拉低
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void power_off(void)
{
    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_PWEN_CTL) = 0;
    usleep(50000);
}

/************************************************************************
* 函数名: Rst_On 
* 功能描述:  复位,RSTN输出拉低3ms
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void Rst_On(void)
{
    /*bit12=0，不屏蔽软复位;bit11-0设置复位时长,32K周期数*/
    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_CHIPRSTN_CTL) = (0<<12 | 0x44c<<0);
    /*写入ff启动软复位*/
    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SFRST_CTL) = 0xff;
    usleep(50000);
}

/************************************************************************
* 函数名: uart2_bt_test 
* 功能描述: 测试uart2的连通性，原模组设计uart2连接BCM4343S蓝牙设备
*                           BT BCM4343S测试，同步后发数，check返回的数据是否正确。
*
* 输入参数:None
*
* 输出参数: None
* 返回值: 0: failed   1: success
* 备注:  
* 
*************************************************************************/
int uart2_bt_test(void)
{
    char btReceiveValue[20];
    int ret = 0;
    int i = 0;
    struct termios old_tty_opt;
    struct termios new_tty_opt;
    int count = 0;
    int len = 0;
    int MaxFd = 0;
    fd_set fs_read; 
    struct timeval tv_timeout; 

    HARDWARE_TEST("uart2 bt test begin.\n\r");
    //bt_enable_reset();

    /* set gpio uart2 function*/
    //PIN_Mux_set(183, 0);    /* uart2_tx 端口连接到外部管脚 U1TXD(gpio183)*/
    //PIN_Mux_set(184, 0);    /* uart2_rx 端口连接到外部管脚 U1RXD(gpio184)*/

    g_ttyS2_fd = open(UART2_DEV_FILE, O_RDWR | O_NOCTTY |O_NDELAY);
    if(g_ttyS2_fd < 0)
    {
        HARDWARE_TEST("Can't open %s.\n", UART2_DEV_FILE);
        goto open_err;
    }
	
    HARDWARE_TEST("Open %s success.\n", UART2_DEV_FILE);

    /**1. tcgetattr函数用于获取与终端相关的参数。 
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中 
    */ 
    ret = tcgetattr(g_ttyS2_fd, &old_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't get %s attributes.\n", UART2_DEV_FILE);
        goto tcgetattr_err;
    }

    new_tty_opt = old_tty_opt;

    /**2. 修改所获得的参数*/ 
    new_tty_opt.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能 
    new_tty_opt.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位 
    new_tty_opt.c_cflag &= ~CRTSCTS;//无硬件流控 
    new_tty_opt.c_cflag |= CS8;//8位数据长度 
    new_tty_opt.c_cflag &= ~CSTOPB;//1位停止位 
    new_tty_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    new_tty_opt.c_iflag |= IGNPAR;//无奇偶检验位
//    new_tty_opt.c_oflag = 0; //输出模式
//    new_tty_opt.c_lflag = 0; //不激活终端模式

    new_tty_opt.c_oflag &= ~(ICANON | OPOST); //不需要回车就能read buffer中的数据
    
    new_tty_opt.c_cc[VTIME] = 0;    //读取超时时间
    new_tty_opt.c_cc[VMIN] = 0;    //读取的最小字符数
    cfsetospeed(&new_tty_opt, B115200);     //设置波特率 
    cfsetispeed(&new_tty_opt, B115200);	//设置输入波特率115200

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/ 
    tcflush(g_ttyS2_fd, TCIOFLUSH);//溢出数据可以接收，但不读 
    ret = tcsetattr(g_ttyS2_fd, TCSANOW, &new_tty_opt); 
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART2_DEV_FILE);
        goto tcsetattr_1_err;
    }

    /* 阻塞读取控制台console输入的字符*/
    while (1)
    {
        FD_ZERO(&fs_read); 
        FD_SET(g_ttyS2_fd, &fs_read);

        MaxFd = g_ttyS2_fd + 1;

        tv_timeout.tv_sec = 1;
        tv_timeout.tv_usec = SELECT_TIMEOUT_500MS;

        /* 同步UART*/
        for (i=0; i<4; i++)
        {
            len = write(g_ttyS2_fd, bcm4343S_baud+i, 1);
            if(len > 0)
            {
                HARDWARE_TEST("ttyS2 send data: 0x%x\n", bcm4343S_baud[i]); 
            }
            else
            {
                goto writeerr; 
            }
            usleep(20);
        }
        usleep(20);

        ret = select(MaxFd, &fs_read, NULL, NULL, &tv_timeout);
        if(ret < 0)
        {
            HARDWARE_TEST("select ttyS2 failed.(err: %d).\n", ret);
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select ttyS2 timeout.\n");
            goto readerr;
        }

        if(FD_ISSET(g_ttyS2_fd, &fs_read))
        {
            /* 判断BT串口返回参数是否为预设值*/
            len = read(g_ttyS2_fd, btReceiveValue, 20);
            if (len <= 0)
            {
                HARDWARE_TEST("Read ttyS2 failed.(err: %d).\n", len);
                goto readerr;
            }
            HARDWARE_TEST("Read ttyS2 length %d.\n", len);
        }
        HARDWARE_TEST("uart2 speed baud Resp: %s", btReceiveValue);

        ret = strcmp(bcm4343S_baudResp, btReceiveValue);
        if (0 != ret)
        {
            goto readerr;
        }

        usleep(1000);
        len = write(g_ttyS2_fd, bcm4343S_HCIrst, 4);
        if(len > 0)
        {
            HARDWARE_TEST("ttyS2 send HCIrst data length: %d\n", len); 
        }
        else
        {
            goto writeerr; 
        }
        usleep(20);

        ret = select(MaxFd, &fs_read, NULL, NULL, &tv_timeout);
        if(ret < 0)
        {
            HARDWARE_TEST("select ttyS2 failed.(err: %d).\n", ret);
            continue;
        }
        else if(0 == ret)
        {
            HARDWARE_TEST("select ttyS2 timeout.\n");
            goto readerr;
        }

        if(FD_ISSET(g_ttyS2_fd, &fs_read))
        {
            /* 判断BT串口返回参数是否为预设值*/
            len = read(g_ttyS2_fd, btReceiveValue, 10);
            if (len <= 0)
            {
                HARDWARE_TEST("Read ttyS2 failed.(err: %d).\n", len);
                goto readerr;
            }
            HARDWARE_TEST("Read ttyS2 length %d.\n", len);
        }
        HARDWARE_TEST("uart2 HCIrst Resp: %s", btReceiveValue);

        ret = strcmp(bcm4343S_HCIrstResp, btReceiveValue);
        if (0 != ret)
        {
            goto readerr;
        }
        break;
    }

    /* 回车和换行当成同一字符*/
    old_tty_opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    old_tty_opt.c_oflag &= ~(ONLCR | OCRNL);
    tcflush(g_ttyS2_fd, TCIOFLUSH);

    ret = tcsetattr(g_ttyS2_fd, TCSANOW, &old_tty_opt);
    if(ret < 0)
    {
        HARDWARE_TEST("Can't set %s attributes.\n", UART2_DEV_FILE);
        goto tcsetattr_2_err;
    }

    close(g_ttyS2_fd);
    g_ttyS2_fd = -1;
    HARDWARE_TEST("uart2 test success!\n");

    return 1;

tcsetattr_2_err:
readerr:
    tcsetattr(g_ttyS2_fd, TCSANOW, &old_tty_opt);
writeerr:
tcsetattr_1_err:
tcgetattr_err:

    close(g_ttyS2_fd);
    g_ttyS2_fd = -1;

open_err:
    HARDWARE_TEST("uart2 test failed!\n");

    return 0;
}

/************************************************************************
* 函数名: bt_enable_reset 
* 功能描述: BT BCM4343S使能和复位
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void bt_enable_reset(void)
{
    char data;

    //pmu_lc1160_aldo_en(14, 1);  //3.3
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x3a, 0xdf);
    //pmu_lc1160_aldo_en(13, 1);  //1.2
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x15, 0x4f);
    //i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x16, 0x4f);
    //i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x29, 0x7c);
    data=i2cx_ReadReg(I2C_PCMCODEC_NUM, 0x29);
    data&=~(1<<1);
    i2cx_WriteReg(I2C_PCMCODEC_NUM, 0x29, data);

    /* BT BCM4343S CHIPEN 先置0，再置1*/
    gpio_direction(BT_BCM4343S_CHIPEN, GPIO_DIR_OUTPUT);
    gpio_output_set(BT_BCM4343S_CHIPEN, LOW_LEVEL);
    usleep(50);
    gpio_output_set(BT_BCM4343S_CHIPEN, HIGH_LEVEL);

    /* BT BCM4343S POWER ON 先置0，再置1*/
    gpio_direction(BT_BCM4343S_PWDN, GPIO_DIR_OUTPUT);
    gpio_output_set(BT_BCM4343S_PWDN, LOW_LEVEL);
    usleep(50);
    gpio_output_set(BT_BCM4343S_PWDN, HIGH_LEVEL);

    gpio_direction(WIFI_BCM4343S_PWDN, GPIO_DIR_OUTPUT);
    gpio_output_set(WIFI_BCM4343S_PWDN, LOW_LEVEL);
    usleep(50);
    gpio_output_set(WIFI_BCM4343S_PWDN, HIGH_LEVEL);
    usleep(50);

    return;
}

/************************************************************************
* 函数名: sdio2_wifi_test 
* 功能描述: 测试SDIO2及BCM4343S的wifi模块
*
* 输入参数:None
*
* 输出参数: None
* 返回值: 0: failed   1: success
* 备注:  
* 
*************************************************************************/
int sdio2_wifi_test(void)
{
    unsigned int  err = 0;
     int ret = 0;

    HARDWARE_TEST("SDIO Test Begin.\n");
    /* BCM4343S WIFI上电复位*/
    wifi_power_rst(ON);
    /* 初始化芯片的SDMMC接口*/
    HARDWARE_TEST("Init SDIO  Interface.\n");

    Init_sdCard();
    HARDWARE_TEST("SDIO  Interface Init complete.\n");
    /* 初始化BCM4330 SDIO口*/
    err = MmcInitCardSdioMode();
    /*判断初始化的测试结果*/
    if(err != 0)
    {
        ret = 0;
        HARDWARE_TEST("sdio2 wifi test failed!\n");
    }
    else
    {
        ret = 1;
        HARDWARE_TEST("sdio2 wifi test ok!\n");
    }
    /* 断电，复位拉低*/
    wifi_power_rst(OFF);

    return ret;
}

/************************************************************************
* 函数名: Init_sdCard 
* 功能描述: 初始化sdio2接口控制器
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void Init_sdCard(void)
{
    unsigned int	i = 0;
    unsigned int	j = 0;

#define             SDMMC2_CLK1_DELAY	0
#define             SDMMC2_CLK2_DELAY	0

    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_SDMMC2CLKCTL1) = ((2<<8) | (SDMMC2_CLK1_DELAY<<4) | (SDMMC2_CLK2_DELAY));

    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_FIFOTH) =  0x1;		

    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_TMOUT) = 0xFFFFFFFF;						//设置响应超时和数据读超时为最大值

    //*(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC1_CTRL) = (1<<24) | 0x07;				//复位*SDMMC1控制器,使能CMD引脚的Open-Drain
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CTRL) = 0x07;	
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS) = 0xFFFF;						//清除*SDMMC1控制器所有中断标志位

    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CLKDIV) = 0x50;//0x41; 					//设置*SDMMC1控制器的输出时钟为400K
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CLKENA) = 0x10001;						//使能*SDMMC1控制器的时钟输出
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMD) = (0xa0000000 | (1<<13) | (1<<21));				
    while(*(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMD) & 0x80000000);

    /* - sdCard初始化开始*/
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS) = 0x04;
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMDARG) = 0x0000;
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMD) = (0xa0000000 |(1<<15));			//给卡发送80个CLK,CMD0
    while((*(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS) & 0x04) != 0x04);			//等待命令完成
    
    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS) = 0x04;

    *(volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CTYPE) = 0;						        //SDMMC控制器采用1 Bits Bus!

}

/************************************************************************
* 函数名: wifi_power_rst 
* 功能描述: WIFI BCM4343S复位上电
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void wifi_power_rst(unsigned char onoff)
{
    char data;

    if(onoff)
    {
        HARDWARE_TEST("Now BCM4343S is Power ON.\n");
        /* BT BCM4343S CHIPEN 先置0，再置1*/
        gpio_direction(BT_BCM4343S_CHIPEN, GPIO_DIR_OUTPUT);
        gpio_output_set(BT_BCM4343S_CHIPEN, LOW_LEVEL);
        usleep(50);
        gpio_output_set(BT_BCM4343S_CHIPEN, HIGH_LEVEL);

        /* BT BCM4343S POWER ON 先置0，再置1*/
        gpio_direction(BT_BCM4343S_PWDN, GPIO_DIR_OUTPUT);
        gpio_output_set(BT_BCM4343S_PWDN, LOW_LEVEL);
        usleep(50 );
        gpio_output_set(BT_BCM4343S_PWDN, HIGH_LEVEL);

        gpio_direction(WIFI_BCM4343S_PWDN, GPIO_DIR_OUTPUT);
        gpio_output_set(WIFI_BCM4343S_PWDN, LOW_LEVEL);
        usleep(50);
        gpio_output_set(WIFI_BCM4343S_PWDN, HIGH_LEVEL);
        usleep(50);
    }
    else
    {
        gpio_output_set(WIFI_BCM4343S_PWDN, LOW_LEVEL); /* 断电，复位拉低*/
        gpio_output_set(BT_BCM4343S_PWDN, LOW_LEVEL);
    }

    return;
}

/************************************************************************
* 函数名: MmcInitCardSdioMode 
* 功能描述: SDIO接口初始化函数  上电后SDIO 执行CMD0 和 CMD5
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int MmcInitCardSdioMode(void)
{
    unsigned int ocr = 0;
    unsigned int err = 0;

    /* SDIO 复位*/
    sdio_go_idle();
    usleep(1000);

    /* 首先发送CMD5命令 如果有相应说明是SDIO 返回OCR 寄存器的值 检测用*/
    err = Mmc_send_io_cmd5(0, &ocr); 

    if (err == MMC_CMD_OK)
    {
        err = MmcAttachCardSdio(ocr); /* 初始化卡*/
        if (err != 0)
        {
            /* 有错，打印错误号*/
            HARDWARE_TEST("MmcAttachCardSdio err=%d\n", err);
        }
    }

    return err;
}

/************************************************************************
* 函数名: sdio_go_idle 
* 功能描述: 复位函数
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int sdio_go_idle(void)
{
    unsigned char   commandBits[6] = {0};
    unsigned long   respBuff[4]={0};
    DlMmcCmdResult  resCode;

    /* CMD0 复位SDIO*/
    commandBits[0] = M_mmcCmdIdx( MMC_GO_IDLE_STATE );
    resCode = DlMmcWriteCommand( SDIO_card, commandBits, respBuff, M_mmcCmdRsp( MMC_GO_IDLE_STATE ) );

    HARDWARE_TEST("Sending CMD0 reset SDIO command status = %d", resCode);

    return 0;
}

/************************************************************************
* 函数名: Mmc_send_io_cmd5 
* 功能描述: 发送CMD5命令函数 返回OCR寄存器的值
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int Mmc_send_io_cmd5(unsigned int ocr, unsigned int * rocr)
{
    unsigned char  commandBits[6] = {0};
    unsigned long  respBuff[4];
    DlMmcCmdResult resCode;
    unsigned long  result = 0;
    unsigned long  cmd5_arg;
    unsigned long  cmd5_loop_timers = 0;

#define MMC_SMALL_NUMBER_OF_RETRIES     500
#define MMC_LARGE_NUMBER_OF_RETRIES     1000

    cmd5_loop_timers = MMC_SMALL_NUMBER_OF_RETRIES;
    while(cmd5_loop_timers-- > 0)
    {
        //CMD5 命令
        commandBits[0] = M_mmcCmdIdx(SD_IO_SEND_OP_COND);
        cmd5_arg = ocr;
        L1AlMmcWriteArgument(&commandBits[1], cmd5_arg);
        resCode = DlMmcWriteCommand(SDIO_card, commandBits, respBuff, M_mmcCmdRsp(SD_IO_SEND_OP_COND));   

        // if we're just probing, do a single pass  检测卡的话 ，直接通过
        if (ocr == 0)
        {
            result = MMC_CMD_OK;
            break;
        }

        if (0 != (respBuff[0] & OCR_CARD_POWER_UP_BUSY))
        {
            // Card is ready to operate after initialization
            result = MMC_CMD_OK; //原来的代码这句屏蔽，我觉得应该有
            break;
        }
        
        result = MMC_DTO_INTR;
    }
    
    if (rocr != NULL)
    {
        *rocr = respBuff[0]; //返回OCR寄存器的值  0x20ffff00 
    }
    
    HARDWARE_TEST("cardReg =0x%lx ,count=%d ", respBuff[0], cmd5_loop_timers);
    
    return result;
}

/************************************************************************
* 函数名: L1AlMmcWriteArgument 
* 功能描述: 获取命令执行结果的函数
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
void L1AlMmcWriteArgument( unsigned char *argField, unsigned long argVal )
{
    argField[0] = (unsigned char)(argVal >> 24) & 0xff;      /* argument MSB */
    argField[1] = (unsigned char)(argVal >> 16) & 0xff;      /* argument 2nd byte */
    argField[2] = (unsigned char)(argVal >> 8) & 0xff;       /* argument 3rd byte */
    argField[3] = (unsigned char)(argVal & 0xff);            /* argument LSB */
}

/************************************************************************
* 函数名: DlMmcWriteCommand 
* 功能描述: SD卡 MMC SDIO 写命令函数
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
DlMmcCmdResult DlMmcWriteCommand(unsigned char SDMMCx, unsigned char *commandBits , unsigned long *respBuff, unsigned char  respLen)
{
    unsigned long commandWord = 0;
    unsigned short tmpCmdIdx;
    unsigned long statusWord = 0;
    unsigned long waitLoop = 0;
    unsigned long timeout;
    DlMmcCmdResult retVal = 0;

    volatile unsigned * SDMMCx_STATUS;
    volatile unsigned * SDMMCx_RINTSTS;
    volatile unsigned * SDMMCx_CMDARG;
    volatile unsigned * SDMMCx_CMD;
    volatile unsigned * SDMMCx_RESP3;
    volatile unsigned * SDMMCx_RESP2;
    volatile unsigned * SDMMCx_RESP1;
    volatile unsigned * SDMMCx_RESP0;

    if (SDMMCx == 2)
    {
        SDMMCx_STATUS = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_STATUS);
        SDMMCx_RINTSTS = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS);
        SDMMCx_CMDARG = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMDARG);
        SDMMCx_CMD = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_CMD);
        SDMMCx_RESP3 = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RESP3);
        SDMMCx_RESP2 = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RESP2);
        SDMMCx_RESP1 = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RESP1);
        SDMMCx_RESP0 = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RESP0);
    }
    else
    {
        HARDWARE_TEST("SDIO NUM out of range!\n");
    }

    timeout = 100000;

    *SDMMCx_RINTSTS = 0xffffffff;

    /* 卡命令参数赋值*/
    *SDMMCx_CMDARG = ((unsigned long)commandBits[1]<<24) | ((unsigned long)commandBits[2]<<16) | ((unsigned long)commandBits[3]<<8) |((unsigned long)commandBits[4]);
    /* 只保留索引*/
    commandWord = commandBits[0] & 0x3f; 
    tmpCmdIdx = commandBits[0] & 0x3f;  //低6位是命令索引

    /* 对于CMD53, 增加SDMMC_CMD_DATA_TRANSFER_EXPECTED,表示有数据块*/
    if(tmpCmdIdx == M_mmcCmdIdx( SD_IO_RW_EXTENDED))
    {
        unsigned char u8temp = 0;

        u8temp = (unsigned char)commandBits[1];
        u8temp = u8temp & 0x80; 
        /* Write or Read*/
        /* Cmd53 write*/
        if (u8temp != 0)
        {
            HARDWARE_TEST("INFO:<CMD53> CMD Write\n");
            commandWord |= (1 << SDMMC_CMD_READ_WRITE);
        }
        /* Cmd53 read*/
        else
        {
            HARDWARE_TEST("INFO:<CMD53> CMD Read\n");
            //commandWord |= (0 << SDMMC_CMD_READ_WRITE);
        }

        /* Whatever byte mode or block mode, the both are regarded as block transfer mode*/
        //commandWord |= (0 << SDMMC_CMD_TRANSFER_MODE);
        commandWord |= (1 << SDMMC_CMD_DATA_TRANSFER_EXPECTED);      /* 有数据传输*/
        //commandWord |= (0 << SDMMC_CMD_UPDATE_CLOCK_REGISTERS_ONLY);
        //commandWord |= (0 << SDMMC_CMD_STOP_ABORT_CMD);
        //commandWord |= (0 << SDMMC_CMD_SEND_INITIALIZATION);
        commandWord |= (1 << SDMMC_CMD_RESPONSE_EXPECT);             /* 期望来自卡的响应*/
        commandWord |= (1 << SDMMC_CMD_WAIT_PRVDATA_COMPLETE);       /* 在发送命令之前等待前一次数据传送完成*/
        commandWord |= (1 << SDMMC_CMD_CHECK_RESPONSE_CRC);          /* 检查响应CRC*/

    }
    else
    {
        commandWord |= (0 << SDMMC_CMD_UPDATE_CLOCK_REGISTERS_ONLY); /* 正常命令序列*/
        commandWord |= (0 << SDMMC_CMD_DATA_TRANSFER_EXPECTED);      /* 无数据传输*/
        commandWord |= (0 << SDMMC_CMD_STOP_ABORT_CMD);              /* 在数据传送结束时不发送stop 命令*/
        commandWord |= (0 << SDMMC_CMD_SEND_INITIALIZATION);         /* 在发送此命令之前不发送初始化序列*/
        commandWord |= (1 << SDMMC_CMD_RESPONSE_EXPECT);             /* 期望来自卡的响应*/

        /* 如果是复位IDLE 则不希望收到来自卡的响应*/
        if (tmpCmdIdx == M_mmcCmdIdx(MMC_GO_IDLE_STATE))
        {
            commandWord |= (0 << SDMMC_CMD_RESPONSE_EXPECT);
        }

         /* 在发送命令之前等待前一次数据传送完成*/
        commandWord |= (1 << SDMMC_CMD_WAIT_PRVDATA_COMPLETE);
        //commandWord |= (1 << SDMMC_CMD_CHECK_RESPONSE_CRC);
    }

    if(tmpCmdIdx == M_mmcCmdIdx( EMMC_SEND_EXT_CSD)) //EMMC 的CMD8
    {
        commandWord = M_mmcCmdIdx( EMMC_SEND_EXT_CSD_REAL);
        commandWord |= (1 << SDMMC_CMD_DATA_TRANSFER_EXPECTED);
    }

    if ( respLen > MMC_R1_LENGTH )//长响应17
    {
        commandWord |=  ((1 << SDMMC_CMD_RESPONSE_EXPECT) | (1 << SDMMC_CMD_RESPONSE_LENGTH));
    }
    else if( respLen > 0 ) //短响应6
    {
        commandWord |= (1 << SDMMC_CMD_RESPONSE_EXPECT);
    }

    //写命令 有数据传输
    if((tmpCmdIdx == M_mmcCmdIdx( MMC_WRITE_MULTIPLE_BLOCK)) ||(tmpCmdIdx == M_mmcCmdIdx( MMC_WRITE_BLOCK)) )
    {
        commandWord |= ((1 << SDMMC_CMD_READ_WRITE) | (1 << SDMMC_CMD_DATA_TRANSFER_EXPECTED));
    }
    //读命令 有数据传输
    else if((tmpCmdIdx == M_mmcCmdIdx( MMC_READ_MULTIPLE_BLOCK)) ||(tmpCmdIdx == M_mmcCmdIdx( MMC_READ_SINGLE_BLOCK)))
    {
        commandWord |= (1 << SDMMC_CMD_DATA_TRANSFER_EXPECTED);
    }
    //读命令 有数据传输
    if(commandBits[0] == M_mmcCmdIdx( SD_SWITCH_FUNC))
    {
        //CMD6 is conflict with ACMD6, so use this method
        //或者这么写也可以commandWord = M_mmcCmdIdx(SD_SET_BUS_WIDTH);
        commandWord = CMD6 |  (1 << SDMMC_CMD_START_CMD) | (1 << SDMMC_CMD_RESPONSE_EXPECT) | (1 << SDMMC_CMD_DATA_TRANSFER_EXPECTED);
    }

    //开始命令。一旦命令被CIU 取得，该位就自动清除
    commandWord |= (1 << SDMMC_CMD_START_CMD);

    /* send the command */
    *SDMMCx_CMD  = commandWord;

    //等待命令执行完成  
    while ((*(SDMMCx_CMD) & (1UL << SDMMC_CMD_START_CMD)) && (timeout--))
    {
        usleep(1);
    }

    if (*(SDMMCx_CMD) & (1UL << SDMMC_CMD_START_CMD)) 
    {
        HARDWARE_TEST("WriteCommand timeout\n");
        return DD_SDMMC_TIMEOUT;
    }

    //命令完成，获取响应
    do
    {
        statusWord = *SDMMCx_RINTSTS;
    }while( ((statusWord & ((1<<SDMMC_CRCERR_INTR) | (1<<SDMMC_RCRC_INTR) | (1<<SDMMC_RTO_INTR) | (1 << SDMMC_CD_INTR) )) == 0) && (waitLoop++ < 60000) );

    if( respLen > MMC_R1_LENGTH )
    {
        respBuff[0] = *SDMMCx_RESP3;
        respBuff[1] = *SDMMCx_RESP2;
        respBuff[2] = *SDMMCx_RESP1;
        respBuff[3] = *SDMMCx_RESP0;
    }
    else if( respLen > 0 )
    {
        respBuff[0] = *SDMMCx_RESP0;
    }
    //获取中断响应状态号
    retVal = localGetResultCode(SDMMCx);

    //
    if( (tmpCmdIdx == M_mmcCmdIdx( MMC_SEND_OP_COND)))
    {
        retVal = MMC_CMD_OK;
    }

    //清除命令完成中断
    *SDMMCx_RINTSTS = (1<<SDMMC_CD_INTR);

    //返回结果
    return retVal;
}

/************************************************************************
* 函数名: localGetResultCode 
* 功能描述: 获取命令执行结果的函数
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
DlMmcCmdResult localGetResultCode( unsigned char SDMMCx)
{
    DlMmcCmdResult retVal = 0;
    unsigned char idx = 2; //0
    volatile unsigned * SDMMCx_RINTSTS ;

    if (SDMMCx == 2)
    {
        SDMMCx_RINTSTS = (volatile unsigned *)(SDMMC2_MODEULE_BASE + SDMMC2_RINTSTS);
    }
    else
    {
        HARDWARE_TEST("SDIO NUM out of range!\n");
    }

    while( ((*SDMMCx_RINTSTS & (1 << idx)) == 0)&&(idx < 16) )
    {
        idx++;
    }
    retVal = (DlMmcCmdResult)(idx + MMC_RESERVED_ENUM);
    
    return retVal;
}

/************************************************************************
* 函数名: MmcAttachCardSdio 
* 功能描述: SDIO初始化检测
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int MmcAttachCardSdio( unsigned int ocr)
{
    unsigned long ocr_new;
    unsigned long err = 0;

    /* Sanity check the voltages that the card claims to support.*/
    if (ocr & 0x7F)
    {
        /* 取低7位 如果>0 说明电压范围已经超出了定义 把不支持的电压域屏蔽掉*/
        ocr &= ~0x7F;
    }

    /* 根据新设定的电压值重新发送CMD5*/
    err = Mmc_send_io_cmd5(ocr, &ocr_new);
    if (err != MMC_CMD_OK)
    {
        HARDWARE_TEST("ERROR: failed at sending CMD5!!\n");
        err = -EFAULT;
        goto remove;
    }
    HARDWARE_TEST("Mmc_send_io_cmd5 success!!\n");

    //发送CMD3  设置卡的地址
    err = sdio_send_relative_addr(&rac);
    if (err != 0)
    {
        HARDWARE_TEST("ERROR: failed at sending CMD3!!\n");
        goto remove;
    }
    HARDWARE_TEST("Mmc_send_io_cmd3 success!!\n");

    //发送CMD7 选择卡
    err = sdio_select_card(rac);
    if (err != 0)
    {
        HARDWARE_TEST("ERROR: failed at sending CMD7!!\n");
        goto remove;
    }
    HARDWARE_TEST("Mmc_send_io_cmd7 success!!\n");

    //读取CCCR寄存器
    err = sdio_read_cccr(&cccr_record);
    if (err)
    {
        HARDWARE_TEST("ERROR: failed at sdio_read_cccr!!\n");
        goto remove;
    }
    HARDWARE_TEST("INFO: CCCR [END]===========\n");

remove:

    return err;
}

/************************************************************************
* 函数名: sdio_send_relative_addr 
* 功能描述: 设置卡的地址
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int sdio_send_relative_addr(unsigned int * rca)
{
    unsigned char  commandBits[6] = {0};
    unsigned long  respBuff[4];
    DlMmcCmdResult resCode = 0;

    // CMD3  设置卡的地址
    commandBits[0] = M_mmcCmdIdx(MMC_SET_RELATIVE_ADDR);
    L1AlMmcWriteArgument(&commandBits[1], mmcSdioRelAddress);
    resCode = DlMmcWriteCommand(SDIO_card, commandBits, respBuff, M_mmcCmdRsp(MMC_SET_RELATIVE_ADDR));   

    // 命令完成 得到响应
    if (resCode != MMC_CMD_RESP_END)
    {
        return resCode;
    }
    *rca = respBuff[0] >> 16;

    return 0;
}

/************************************************************************
* 函数名: sdio_select_card 
* 功能描述: 选择卡
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int sdio_select_card(unsigned int rca)
{
    unsigned char  commandBits[6] = { 0 };
    unsigned long  respBuff[4];
    DlMmcCmdResult resCode = 0;

    /* CMD7 选择卡*/
    commandBits[0] = M_mmcCmdIdx(MMC_SEL_DESEL_CARD);
    if (rca)
    {
        L1AlMmcWriteArgument(&commandBits[1], rca << 16);
    }
    else
    {
        L1AlMmcWriteArgument(&commandBits[1], 0);
    }
    resCode = DlMmcWriteCommand(SDIO_card ,commandBits, respBuff, M_mmcCmdRsp(MMC_SEL_DESEL_CARD));
    /* 命令完成 得到响应*/
    if (resCode != MMC_CMD_RESP_END)
    {
        HARDWARE_TEST("CMD7 return status resCode=%d ", resCode);
        return resCode;
    }
    
    return 0;
}

/************************************************************************
* 函数名: sdio_read_cccr 
* 功能描述: 读 CCCR寄存器的值
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int sdio_read_cccr(struct sdio_cccr *sdio_cccr)
{
    int ret = 0;
    int cccr_vsn;
    unsigned char data;

    /* 读取 SDIO_CCCR_CCCR的值 */
    ret = mmc_io_rw_direct(0, 0, SDIO_CCCR_CCCR, 0, &data);
    if (ret)//错误，退出
    {
        goto out;
    }

    cccr_vsn = data & 0x0f;

    if (cccr_vsn > SDIO_CCCR_REV_1_20)
    {
        // 最高1.2版本 大于1.2 信息不正确
        HARDWARE_TEST("unrecognised CCCR structure version %d\n", cccr_vsn);
//        return -EINVAL;
    }
    //SDIO Revision
    sdio_cccr->sdio_vsn = (data & 0xf0) >> 4;
    //读取 SDIO_CCCR_CAP的值  Card Capability 0x08
    ret = mmc_io_rw_direct( 0, 0, SDIO_CCCR_CAPS, 0, &data);
    if (ret)
    {
        goto out;
    }

    if (data & SDIO_CCCR_CAP_SMB)
    {
        HARDWARE_TEST("[CCCR] can do multi-block xfers (CMD53)\n");
        sdio_cccr->multi_block = 1;  // Multi-block is supported on BRCM SDIO device
    }
    if (data & SDIO_CCCR_CAP_LSC)
    {
        HARDWARE_TEST("[CCCR] low speed card\n");
        sdio_cccr->low_speed = 1;
    }
    if (data & SDIO_CCCR_CAP_4BLS)   // 4 bitmode is supported on BRCM SDIO device
    {
        HARDWARE_TEST("[CCCR] 4 bit low speed card\n");
        sdio_cccr->wide_bus = 1;
    }
    //版本大于=1.1
    if (cccr_vsn >= SDIO_CCCR_REV_1_10) 
    {
        ret = mmc_io_rw_direct( 0, 0, SDIO_CCCR_POWER, 0, &data);
        if (ret)
        {
            goto out;
        }

        if (data & SDIO_POWER_SMPC)
        {
            sdio_cccr->high_power = 1; // high power is supported on BRCM SDIO device
        }
    }
    //版本大于=1.2
    if (cccr_vsn >= SDIO_CCCR_REV_1_20)
    {
        ret = mmc_io_rw_direct( 0, 0, SDIO_CCCR_SPEED, 0, &data);
        if (ret)
        {
            goto out;
        }

        if (data & SDIO_SPEED_SHS)
        {
            HARDWARE_TEST("[CCCR] Supports High-Speed mode\n");
            sdio_cccr->high_speed = 1;
        }
    }

out:
    return ret;
}

/************************************************************************
* 函数名: mmc_io_rw_direct 
* 功能描述: 配置命令参数
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int mmc_io_rw_direct( int write, unsigned fn,unsigned addr, unsigned char in, unsigned char * out)
{
    unsigned int ulCmd52 = 0;
    int err = 0;

    if (write == 0)
    {
        if (out == NULL)
        {
            return -EINVAL; // for read
        }
    }
    // [47] : start bit
    // [46] : direction
    // [45:40] : command index

    // [39]: R/W flag
    // [38:36]: Function Number Note that function 0 selects the common I/O area (CIA).
    // [35]: RAW flag
    // [34]: Stuff
    //================
    // [33：17] : Register Address
    // [16] :  Stuff
    // [15:8]: Write Data/Stuff Bits :

    // [7:1] : CRC7   :
    // [0] : End bit:
    ulCmd52 = write ? 0x80000000 : 0x00000000; //cmd52命令写  If this bit is 0, this command shall read data from the SDIO card
    ulCmd52 |= fn << 28;//Function Number
    ulCmd52 |= (write && out) ? 0x08000000 : 0x00000000;//RAW 
    ulCmd52 |= addr << 9; //地址
    ulCmd52 |= in; 

    err = Mmc_cmd52(ulCmd52, write, out);

    return err;
}

/************************************************************************
* 函数名: Mmc_cmd52 
* 功能描述: 发送CMD52 命令
*
* 输入参数:None
*
* 输出参数: None
* 返回值: None
* 备注:  
* 
*************************************************************************/
int Mmc_cmd52(unsigned int ulCmd52, unsigned char uWr, unsigned char * puData)
{
    unsigned char  commandBits[6] = { 0 };
    unsigned char  raw = (ulCmd52 >> 27) & 0x01;
    unsigned long  respBuff[4];
    DlMmcCmdResult resCode;

    sdioDrvStatus = MMC_DRV_CMD_IN_PROGRESS;
    HARDWARE_TEST("enter into Mmc_cmd52\n");	

    // CMD52  
    commandBits[0] = M_mmcCmdIdx(SD_IO_RW_DIRECT);
    L1AlMmcWriteArgument(&commandBits[1], ulCmd52);

    resCode = DlMmcWriteCommand( SDIO_card, commandBits, (u32*)&respBuff, M_mmcCmdRsp(SD_IO_RW_DIRECT));
    if (resCode == MMC_CMD_OK || resCode == MMC_CMD_RESP_END)
    {
        //返回RESPONSE 状态 在R5中
        if (respBuff[0] & R5_ERROR)
        {
            return -EIO;
        }
        
        if (respBuff[0] & R5_FUNCTION_NUMBER)
        {
            return -EINVAL;
        }
        
        if (respBuff[0] & R5_OUT_OF_RANGE)
        {
            return -ERANGE;
        }
        
        //如果是读 则返回
        if (!uWr || raw)//the command shall read the value of the register after the write. RAW flag
        {
            *puData = respBuff[0] & 0x00FF;
        }
    }
    else
    {
        return -EIO;
    }

    sdioDrvStatus = MMC_DRV_IDLE;

    return 0;
}

int init_testcase_record()
{
	int test_fd;
	/* success 1; fail 0; NULL 2; */
	char test_no_do[6] = {'2','2','2','2','2','2'};
	int cru_testcase;
	char init_check[6];

	test_fd=open(EEPROM_FILE,O_RDWR);
	if(test_fd<0){
		perror("open error");
	}
	init_i2c_ioctrl(test_fd);
	for(cru_testcase=BOARD_TESTSTATE; cru_testcase<=DEVELOP_TESTSTATE; cru_testcase++)
	{
		write_sn(test_fd, EEPROM_TEST_RESULT_ADDR+cru_testcase, EEPROM_TEST_RESULT_SIZE, test_no_do+cru_testcase);
	}
	/*
	for(cru_testcase=BOARD_TESTSTATE; cru_testcase<=DEVELOP_TESTSTATE; cru_testcase++)
	{
		read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+cru_testcase, EEPROM_TEST_RESULT_SIZE, &init_check);
		if(test_no_do!=init_check)
		{
			HARDWARE_TEST("init testcase_record data fail\n");
			return -1;
		}
	}
	*/
	close(test_fd);
	return 0;
}

