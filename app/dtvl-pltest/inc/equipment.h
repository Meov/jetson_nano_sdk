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
*˵����
*
**********************************************************************/
#ifndef __EQUIPMENT_H__
#define __EQUIPMENT_H__
#include "dmd_module.h"
#include "sdio.h"

/* EMMC���Բ���*/
/*#define EMMC_TEST_PATH       "/system/etc/emmctest.log"*/
#define EMMC_TEST_PATH       "/amt/emmctest.log"
#define EMMC_TEST_BYTES     50

static char emmc_str[EMMC_TEST_BYTES] = "0123456789";

/* TF�����Բ���*/
#define TCARD_TEST_PATH       "/mnt/sdcard/tcardtest.log"
#define TCARD_TEST_BYTES     50
#define MNT_PATH_BYTES          100
static char tcard_str[TCARD_TEST_BYTES] = "9876543210";

/* gpio����*/
#define GPIO_TEST_BYTES     20

#define AllGPIONUM                  256
#define GPIO_DIR_INPUT        0
#define GPIO_DIR_OUTPUT     1
#define LOW_LEVEL                   0
#define HIGH_LEVEL                  1

/* uart0����*/
#define UART0_TEST_NUM   100
static char uart0_test_buf[UART0_TEST_NUM] = "This is ttyS0\n";

/* uart2����*/
#define UART2_TEST_NUM   100
static char uart2_test_buf[UART2_TEST_NUM] = "This is ttyS2\n";

#define WriteReg(reg, dat)   (*((volatile unsigned int*)(reg))=(dat))
#define ReadReg(reg)    (*((volatile unsigned int*)(reg)))

/* basic bit or reg operation */
#define REG_VALUE(reg)      (*(reg))//��ȡ�Ĵ�����ֵ
#define BIT_FIELD(len)      (((unsigned int)1 << (len)) - 1) //len���ȵ�����Ϊ1

/* reg�Ĵ�����opҪд�������,bit���ĸ�����λ,lenҪ�ı������λ�� *
  * �ѵ�bitλlen����op����д�룬�����ı��ֲ��� */
#define SETBITFIELD(reg, op, bit, len)  REG_VALUE(reg) = (REG_VALUE(reg))&(~(BIT_FIELD(len)<<(bit)))|((unsigned int)(op)<<(bit))
#define SETBITVALUE(reg, op, bit, len)  SETBITFIELD(reg, op, bit, len)

/* ��reg�е�bit����1����0 */
#define SETBIT(reg, bit)    SETBITVALUE(reg, 1, bit, 1)
#define CLRBIT(reg, bit)    SETBITVALUE(reg, 0, bit, 1)

#define SETBIT_ATOM(reg, bit, we_begin)     REG_VALUE(reg) = (1L+(1L<<(we_begin)))<<(bit)//1L<<(we_begin)��дʹ����1,Ȼ���������ƶ�bitλ
#define CLRBIT_ATOM(reg, bit, we_begin)     REG_VALUE(reg) = (0L+(1L<<(we_begin)))<<(bit)

//----------------- I2C_CON���ƼĴ���-----------------------------//
#define I2C_CON_NOMEANING                   6
#define I2C_CON_I2C_RESTART_EN              5 
#define I2C_CON_I2C_10BITADDR_MASTER        4
#define I2C_CON_SPEED_FAST                  2
#define I2C_CON_SPEED                       1
#define I2C_CON_MASTER_MODE                 0 

//-----------------I2C_STATUS״̬�Ĵ���----------------------------//
#define I2C_STATUS_ACTIVITY                 0// 1:active 0:not active
#define I2C_STATUS_TFNF                     1// tx fifo no full
#define I2C_STATUS_TFE                      2//tx fifo emputy
#define I2C_STATUS_RFNE                     3//rx fifo not emputy

//-----------------I2C_TARĿ���ַ�Ĵ���----------------------------//
#define I2C_TAR_10BITADDR                   12
#define I2C_TAR_SPECIAL                     11
#define I2C_TAR_GCORSTART                   10
#define I2C_TAR_TAR                         0x3FF

#define COMIP_I2C_HIGH_SPEED_MODE      0
#define COMIP_I2C_FAST_MODE                   1 
#define COMIP_I2C_STANDARD_MODE         2
#define COMIP_I2C_STANDARD_SPEED       (100000)
#define COMIP_I2C_FAST_SPEED	                 (400000)
#define COMIP_I2C_HIGH_SPEED	                 (3400000)
#define I2C_CON_HIGH_SPEED_MODE	         3
#define I2C_CON_FAST_MODE                       2
#define I2C_CON_STANDARD_MODE             1

#define I2C_SS_SPEED                 0//100KHz
#define I2C_FS_SPEED                 1//400KHz
#define I2C_HS_SPEED                 2//3400KHz
/*I2C�������Զ����Ϳ�ʼ�źż���д��ַ��
���E2PROMʵ�ʵ�ַ0xA0����1λ��д�������I2Cx_TAR�Ĵ���
(I2CӲ����ַ���λ������д��������λ�ɿ�������ȫ��
��ˣ�д��Ĵ�����I2C��ַ����Ԥ�����λ��Ϊʵ�ʵ�ַ����1λ)*/
#define E2PROM_ADDR 0x50
#define E2PROM_PAGE_SIZE 16
#define EEPROM_FILE "/dev/i2c-0"
#define EEPROM_EQSN_ADDR 00
#define EEPROM_IOSN_ADDR 32
#define EEPROM_OEMSN_ADDR 64
#define EEPROM_TEST_RESULT_ADDR 96
#define EEPROM_PAGE_SIZE        16
#define EEPROM_TEST_RESULT_SIZE 1

#define BOARD_TESTSTATE           0  /* single board test */
#define BEFORE_AGING_TESTSTATE    1  /* befor aging */
#define STARTING_AGING_TESTSTATE  2  /* start aging */
#define AFTER_AGING_TESTSTATE     3  /* after aging */
#define RESET_SET_STATE           4  /* reset device */
#define DEVELOP_TESTSTATE         5  /*  */

#define ADCI_BUFFER_SIZE          70

#define I2C_RETRIES 0x0701
#define I2C_TIMEOUT 0x0702
#define I2C_RDWR 0x0707 
#define I2C_XFERSPEED 0x721
#define I2C_TENBIT 0x0704 /*ѡ���ַλ��:=0 for 7bit , != 0 for 10 bit */
#define I2C_SLAVE 0x0703 /*���ôӻ���ַ */

/* LC1160��I2C��ַ*/
#define I2C_LC1160_addr     0x33

/* the battery voltage��ص�ѹ*/
#define ADC_VBAT                  0
/* �ͻ�ʹ�ã�ADC����*/
#define BATINSN                     1
/* DBB_TEMP�¶ȼ��*/
#define ADCI0                          2
/* OBU/RSU IO�忨Ӳ���汾��*/
#define ADCI1                          3
/* DMDģ��Ӳ���汾��*/
#define ADCI2                          4

/* ADC convert����*/
#define ADC_TEST_BYTES          10

/* I2C�ܽ�*/
#define COM_I2C_SCL                    88
#define COM_I2C_SDA                    89
#define I2C0_SCL                        213
#define I2C0_SDA                        214
#define I2C1_SCL                        207
#define I2C1_SDA                        208
#define I2C2_SCL                        163
#define I2C2_SDA                        164
#define I2C3_SCL                        167
#define I2C3_SDA                        168

/* UART�ܽ�*/
#define UART0_TX                        76
#define UART0_RX                        77
#define UART1_TX                        179
#define UART1_RX                        180
#define UART2_TX                        183
#define UART2_RX                        184
#define UART2_CTS                        185
#define UART2_RTS                        186
#define COM_UART_TX                    244
#define COM_UART_RX                    245

/* M-sensor*/
//#define AK09911
#define MMC34160PJ

/* G-sensor*/
#define MMA8653FCR1

/* Gyroscope*/
#define L3GD20

#define      GP2AP030T00F_address        0x39
#define      ST480MF_address             0x0F
#define      MMC34160PJ_address          0x30
#define      AK09911_address             0x0D
#define      L3GD20_address              0x6A
#define      MMA8653FCR1_address         0x1D
#define      KXTJ2_1009_address          0x0E
#define      KXCJK_1013_address          0x0E
#define      BMA223_address              0x18
#define      LIS3DHTR_address            0x18

/* SSI_RIS. */
#define SSI_RFF                             (1 << 4)
#define SSI_RFNE                          (1 << 3)
#define SSI_TFE                             (1 << 2)
#define SSI_TFNF                           (1 << 1)
#define SSI_BUSY                           (1 << 0)

/* MCP25625 SPI interface instruction set */
#define INSTRUCTION_WRITE               0x02
#define INSTRUCTION_READ                0x03
#define INSTRUCTION_BIT_MODIFY      0x05
#define INSTRUCTION_RESET               0xC0

/* MPC251x registers */
#define CANSTAT	      0x0e
#define CANCTRL	      0x0f

/* Modem test*/
#define SERVER_PORT_NUM             50700
#define SERVER_IP_ADDR                 "127.0.0.1"/*"192.168.62.199"*/
#define TCP_BUFFER_SIZE                 50
#define AT_COMMAND                      "AT+TMCMTX"

#define PPS_ERR                         0x0     /* GPIO33��GPIO34����PPS*/
#define PPS_GPIO33                   0x1    /* GPIO33��PPS*/
#define PPS_GPIO34                   0x2    /* GPIO34��PPS*/
#define PPS_OK                           0x3    /* GPIO33��GPIO34����PPS*/

void gps_alps_test(void);
int uart1_gps_test(void);
int gnss_cn0_test(int* cn0);
int uart0_test(void);
void uart2_test(void);
int module_hardware_ver_test(void);
int io_board_hardware_ver_test(void);
int child_board_hardware_ver_test(int board_num);
int module_firmware_ver_test(char *ver_buf);
int signature_check_test(void);
int module_cid_check_test(void *cid);
void adc_convert_test(void);
void tcard_test(void);
int emmc_test(void);
void gpio_write_test(void);
void gpio_read_test(void);
void gpio_loopback_test(void);
void i2c0_e2prom_init(void);
void i2c0_e2prom_test(void);
void sensor_test(void);
int v2x_1pps_test(void);
int usb_hsic_test(void);
int usb_host_test(void);
int usb_LAN_test(char *buffer);
int usb_otg_test(void);
int usb_network_eth_test(void);
int module_wifi_test_wlan0(void);
int module_wifi_test_mlan0(void);
int public_network_test(void);
int can_interface_test(void);
void system_opt(void);
int spi_hsm_detect(int num);
int ssi0_can_test(void);
void modem_rf_test(void);
int gpio_direction(int gpio_id, int gpio_dir);
int gpio_input_value(int gpio_id);
int gpio_output_set(int gpio_id, int level);
void PIN_Mux_set(int Mux_reg_num, int value);
int gpio_level_judge(int gpio_input, int gpio_output);
int i2cx_init(int i2cx, int speed);
void i2cx_reg_init(int I2Cx);
void get_i2cx_parent_rate(void);
int i2cx_clock_init_set_rate(int I2Cx);
void i2cx_clock_enable(int I2Cx);
void i2cx_clock_disable(int I2Cx);
void i2cx_SetDevAddr(int i2cx, unsigned short addr);
void i2cx_WriteReg(int i2cx, unsigned char reg, unsigned char data);
unsigned char i2cx_ReadReg(int i2cx, unsigned char reg);
void e2prom_byte_write(char addr, unsigned int data);
void e2prom_page_write(char page, unsigned int data);
void e2prom_seq_read(unsigned char *data);
void sensor_mmc34160pj_test(void);
void sensor_l3gd20_test(void);
void sensor_mma8653fcr1_test(void);
void pmu_lc1160_init(void);
void pmu_lc1160_test(void);
void pmu_lc1160_aldo_en(int ALDO_num, int enable);
void pmu_lc1160_aldo_en(int ALDO_num, int enable);
void pmu_lc1160_buck_en(int BUCK_num, int enable);
void pmu_lc1160_sink_set(int sink_num, int enable);
float pmu_lc1160_adc_read(unsigned int channel, unsigned short *padc);
unsigned char pmu_lc1160_rtc_read(int input);
void pmu_lc1160_reg_set_test(void);
void split_str(char *src, const char *separator, char **dest, int *num);
int split_ttyUSB_num();
int get_adci_from_device(unsigned short *adci_get);
unsigned int asctohex(char* ascii, unsigned int* val, int num);
void lc1160_read_reg(void);
void lc1160_write_reg(void);
void ssix_reg_init(int ssix);
void ssix_gpio_config(int ssix);
void ssix_clock_enable(int ssix);
void spix_dump_regs(int ssix);
void ssix_WriteReg(volatile unsigned *p_reg, int data);
int ssix_ReadReg(volatile unsigned *p_reg);
void reset_can(int ssix);
int read_reg_can(int ssix, int reg);
void write_reg_can(int ssix, int reg, int data);
void power_off(void);
void Rst_On(void);
int uart2_bt_test(void);
void bt_enable_reset(void);
int sdio2_wifi_test(void);
void wifi_power_rst(unsigned char onoff);
void Init_sdCard(void);
int MmcInitCardSdioMode(void);
int sdio_go_idle(void);
int Mmc_send_io_cmd5(unsigned int ocr, unsigned int * rocr);
void L1AlMmcWriteArgument( unsigned char *argField, unsigned long argVal );
DlMmcCmdResult DlMmcWriteCommand(unsigned char SDMMCx, unsigned char *commandBits , unsigned long *respBuff, unsigned char  respLen);
DlMmcCmdResult localGetResultCode( unsigned char SDMMCx);
int MmcAttachCardSdio( unsigned int ocr);
int sdio_send_relative_addr(unsigned int * rca);
int sdio_select_card(unsigned int rca);
int mmc_io_rw_direct( int write, unsigned fn,unsigned addr, unsigned char in, unsigned char * out);
int Mmc_cmd52(unsigned int ulCmd52, unsigned char uWr, unsigned char * puData);

#endif
