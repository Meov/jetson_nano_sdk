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
#ifndef __DTVL_PRODUCTION_LINE_TEST_H__
#define __DTVL_PRODUCTION_LINE_TEST_H__
#include "dmd_module.h"

/*#define LOG_FILE_PATH       "/data/local/log/dtvl-pltest.log"*/
#define LOG_FILE_PATH       "/var/log/dtvl-pltest.log"
#define MAX_LOG_BYTES       3072/*512*/
#define MAX_KEY_BYTES       100

#define CAN_BUF_SIZ	(255)

#if 1
/*�����Ծ���������������־�ļ�*/
#define PLTEST_LOG_DEBUG
#ifdef PLTEST_LOG_DEBUG
#define HW_LOG(S_FMT, ...) \
do \
{ \
    if (!log_file_ptr) \
        break; \
         \
    snprintf(log_buf, MAX_LOG_BYTES, S_FMT, ##__VA_ARGS__); \
    \
    fwrite(log_buf, strlen(log_buf), 1, log_file_ptr); \
    fflush(log_file_ptr); \
} while(0)
#else
#define HW_LOG(S_FMT, ...)
#endif

/*�����Ծ����������ӡ������̨*/
//#define PLTEST_PRT_DEBUG
#ifdef PLTEST_PRT_DEBUG
#define HW_PRT(S_FMT, ...) \
do \
{ \
    printf(S_FMT, ##__VA_ARGS__); \
} while(0)
#else
#define HW_PRT(S_FMT, ...)
#endif

/*ͬʱ������־�ʹ�ӡ����̨*/
#define HARDWARE_TEST(S_FMT, ...) \
do \
{ \
    HW_LOG(S_FMT, ##__VA_ARGS__); \
    HW_PRT(S_FMT, ##__VA_ARGS__); \
}while(0)

/*���ֺ궨�����ʹ��*/
#else
/*���ƿ��أ������Ծ���������������־�ļ�*/
#define PLTEST_LOG_DEBUG    1

/*���ƿ��أ������Ծ����������ӡ������̨*/
#define PLTEST_PRT_DEBUG    1

/*ͬʱ������־�ʹ�ӡ����̨*/
#define HDTEST_DEBUG(cond1, cond2, S_FMT, ...) \
do \
{ \
    if ((!log_file_ptr) && (1 == cond1)) \
    { \
        snprintf(log_buf, MAX_LOG_BYTES, S_FMT, ##__VA_ARGS__); \
        \
        fwrite(log_buf, strlen(log_buf), 1, log_file_ptr); \
        fflush(log_file_ptr); \
    } \
    \
    if (1 == cond2) \
    { \
        printf(S_FMT, ##__VA_ARGS__); \
    } \
}while(0)

#define HARDWARE_TEST(S_FMT, ...)   PLTEST_DEBUG(PLTEST_LOG_DEBUG, PLTEST_PRT_DEBUG, S_FMT, ##__VA_ARGS__)

#endif

extern char log_buf[MAX_LOG_BYTES];
extern FILE *log_file_ptr;

/* I2C NUM*/
#define I2C_E2PROM_NUM         0
#define I2C_SENSOR_NUM         1
#define I2C_TP_NUM                  2
#define I2C_PCMCODEC_NUM    4
#define I2C_CAM_NUM                3
/* ADC register */
#define LC1160_REG_ADCCR1        	0x87
#define LC1160_REG_ADCCR2         0x88
#define LC1160_REG_ADCDAT0        0x89
#define LC1160_REG_ADCDAT1        0x8a

/* SSI(SPI) NUM*/
#define SSI_0_NUM	0
#define SSI_1_NUM       1
#define SSI_2_NUM       2

#define SSI0_TX         157
#define SSI0_RX         158
#define SSI0_CLK        159
#define SSI0_SSN        160

#define SSI1_TX         219
#define SSI1_RX         220
#define SSI1_CLK        221
#define SSI1_SSN        222

#define SSI2_TX         161
#define SSI2_RX         162
#define SSI2_CLK        163
#define SSI2_SSN        164

/* SDIO NUM*/
#define MMC_TF_NUM  0
#define MMC_eMMC_NUM  1
#define MMC_WIFI_NUM  2

/* PCM NUM*/
#define PCM_CODEC_NUM  0
#define PCM_BT_NUM  1

/* PC��λ���·�����������Ϣ�궨��*/
#define PC_REQ_UE_MESSAGE                                                           0x0             /* PC��λ������������Ϣ*/
#define PC_REQ_IO_BOARD_SN_WR                                                   0x1          /* дIO�忨SN�Ų���������Ϣ*/
#define PC_REQ_IO_BOARD_SN_RD                                                   0x2           /* ��IO�忨SN�Ų���������Ϣ*/
#define PC_REQ_MODULE_EMMC_DETECT                                           0x3          /* EMMC����������Ϣ*/
#define PC_REQ_MODULE_HDWARE_VER_DETECT                             0x4          /* ģ��Ӳ���汾�Ų���������Ϣ*/
#define PC_REQ_MODULE_SDIO2_WIFI_TEST                                   0x5          /* SDIO2�ӿڲ���������Ϣ*/
#define PC_REQ_MODULE_UART2_BT_TEST                                         0x6        /* UART2����������Ϣ*/
#define PC_REQ_IO_BOARD_HDWARE_VER_DETECT                       0x7           /* IO�忨Ӳ���汾�Ų���������Ϣ*/
#define PC_REQ_IO_BOARD_UART1_GNSS_TEST                             0x8           /* UART1����������Ϣ*/
#define PC_REQ_IO_BOARD_PP1S_GNSS_TEST                                  0x9        /* PP1S����������Ϣ*/
#define PC_REQ_IO_BOARD_CN0_GNSS_TEST                                   0xA          /* GPS�����CN0ֵ��ѯ������Ϣ*/
#define PC_REQ_IO_BOARD_COMUART_TEST                                    0xB          /* COMUART����������Ϣ*/
#define PC_REQ_IO_BOARD_USB_OTG_DETECT                                  0xC         /* USB OTG����������Ϣ*/
#define PC_REQ_IO_BOARD_ETH_DETECT                                          0xD           /* ETH���ڲ���������Ϣ*/
#define PC_REQ_IO_BOARD_USB_HSIC_DETECT                                 0xE         /* HSIC�ӿڲ���������Ϣ*/
#define PC_REQ_IO_BOARD_LAN9500_DETECT                                  0xF          /* LAN9500����������Ϣ*/
#define PC_REQ_IO_BOARD_UART0_TEST                                          0x10         /* UART0����������Ϣ*/
#define PC_REQ_IO_BOARD_CAN_TEST                                                0x11       /* CAN�ӿڲ���������Ϣ*/
#define PC_REQ_IO_BOARD_SPI0_CAN_TEST                                       0x12      /* SPI0�ӿڲ���������Ϣ*/
#define PC_REQ_IO_BOARD_I2C1_SENSOR_TEST                                0x13      /* I2C1�ӿڲ���������Ϣ*/
#define PC_REQ_IO_BOARD_AP_WIFI_CONNECT_DETECT                  0x14        /* WIFI���ܲ���������Ϣ*/
#define PC_REQ_IO_BOARD_RSU_NETWORK_ONLINE_DETECT           0x15       /* SIM����������������Ϣ*/
#define PC_REQ_EQUIP_SN_WR                                                              0x16        /* д�豸SN�Ų���������Ϣ*/
#define PC_REQ_EQUIP_SN_RD                                                              0x17        /* ���豸SN�Ų���������Ϣ*/
#define PC_REQ_SN_CHECK                                                                     0x18        /* ������SN����д��SN���Ƿ�һ��*/
#define PC_REQ_IO_BOARD_USB_HOST_DETECT                                 0x19        /* USB HOST����������Ϣ*/
#define PC_REQ_SIGNATURE_DETECT                                                     0x20        /* ǩ����֤������Ϣ*/
#define PC_REQ_VERSION_DETECT                                                         0x21        /* �̼��汾��ѯ������Ϣ*/
#define PC_REQ_MODULE_CID_NUMBER                                                0x22        /* �����ȡEMMC��CID����*/
#define PC_REQ_IO_BOARD_HSM_DETECT                                             0x23        /* HSM�ӿڲ����·���Ϣ*/
#define PC_REQ_EQUIP_ADD_SN_WR_TEST  				0x24			/* �豸SN��д�� */
#define PC_REQ_EQUIP_OEM_RD    						0x25
#define PC_REQ_IO_BOARD_RSU_SIM_STATE_DETECT   0x26
#define PC_REQ_SUBBOARD1_HDWARE_VER_DETECT    0x27 /* request msg_id about subboard_1 */
#define PC_REQ_SUBBOARD2_HDWARE_VER_DETECT    0x28 /* request msg_id about subboard_2 */
#define PC_REQ_HDWARE_VER_RD    0x29	/* read hardware version */
#define PC_REQ_RESET    0x2A
#define PC_REQ_TESTSTATIONS_STATE_CHECK    0x2B
#define PC_REQ_TESTSTATIONS_STATE_WR    0x2C
#define PC_REQ_TESTER_STATE_DETECT    0x2D    /* check tester status for aging test */

/* UE��λ���ظ����Խ����Ϣ�궨��*/
#define UE_RSP_PC_MESSAGE                                                       0x100        /* UE��λ�����Խ���ظ���Ϣ*/
#define UE_RSP_IO_BOARD_SN_WR                                           0x101          /* дIO�忨SN�Żظ���Ϣ*/
#define UE_RSP_IO_BOARD_SN_RD                                               0x102       /* ��IO�忨SN�Żظ���Ϣ*/
#define UE_RSP_MODULE_EMMC_DETECT                                       0x103      /* EMMC���Իظ���Ϣ*/
#define UE_RSP_MODULE_HDWARE_VER_DETECT                         0x104      /* ģ��Ӳ���汾�Ų��Իظ���Ϣ*/
#define UE_RSP_MODULE_SDIO2_WIFI_TEST                               0x105       /* SDIO2�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_MODULE_UART2_BT_TEST                                 0x106         /* UART2���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_HDWARE_VER_DETECT                   0x107        /* IO�忨Ӳ���汾�Ų��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_UART1_GNSS_TEST                         0x108       /* UART1���Իظ�*/
#define UE_RSP_IO_BOARD_PP1S_GNSS_TEST                              0x109     /* PP1S���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_CN0_GNSS_TEST                               0x10A      /* GPS�����CN0ֵ��ѯ�ظ���Ϣ*/
#define UE_RSP_IO_BOARD_COMUART_TEST                                0x10B       /* COMUART���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_USB_OTG_DETECT                              0x10C     /* USB OTG���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_ETH_DETECT                                      0x10D        /* ETH���ڲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_USB_HSIC_DETECT                             0x10E       /* HSIC�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_LAN9500_DETECT                              0x10F        /* LAN9500���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_UART0_TEST                                          0x110      /* UART0���Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_CAN_TEST                                            0x111         /* CAN�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_SPI0_CAN_TEST                                   0x112        /* SPI0�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_I2C1_SENSOR_TEST                            0x113        /* I2C1�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_AP_WIFI_CONNECT_DETECT              0x114         /* WIFI���ܲ��Իظ���Ϣ*/
#define UE_RSP_IO_BOARD_RSU_NETWORK_ONLINE_DETECT       0x115       /* SIM���������Իظ���Ϣ*/
#define UE_RSP_EQUIP_SN_WR                                                          0x116	/* д�豸SN�Żظ���Ϣ*/
#define UE_RSP_EQUIP_SN_RD                                                          0x117         /* ���豸SN�Żظ���Ϣ*/
#define UE_RSP_SN_CHECK                                                                 0x118        /* SN�Ž�����ظ���Ϣ*/
#define UE_RSP_IO_BOARD_USB_HOST_DETECT                             0x119       /* USB OTG���Իظ���Ϣ*/
#define UE_RSP_SIGNATURE_DETECT                                                 0x120       /* ǩ����֤���Իظ���Ϣ*/
#define UE_RSP_VERSION_CHECK                                                        0x121       /* �̼��汾��ѯ�ظ���Ϣ*/
#define UE_RSP_MODULE_CID_CHECK                                                 0x122       /* EMMC CID�Ų�ѯ�ظ���Ϣ*/
#define UE_RSP_IO_BOARD_HSM_DETECT                                          0x123       /* HSM�ӿڲ��Իظ���Ϣ*/
#define UE_RSP_EQUIP_ADD_SN_WR_TEST  				0x124			/* �����豸SN��д��ӿڲ��Իظ���Ϣ*/
#define UE_RSP_EQUIP_OEM_RD    						0x125
#define UE_RSP_IO_BOARD_RSU_SIM_STATE_DETECT    0x126
#define UE_RSP_SUBBOARD1_HDWARE_VER_DETECT	  0x127 	   /* response msg_id about subboard_1 */
#define UE_RSP_SUBBOARD2_HDWARE_VER_DETECT    0x128        /* response msg_id about subboard_2 */
#define UE_RSP_HDWARE_VER_RD    0x129	/* response msg about hardware version */
#define UE_RSP_RESET	0x12A       /* �ָ��������ûظ���Ϣ*/
#define UE_RSP_TESTSTATIONS_STATE_CHECK    0x12B
#define UE_RSP_TESTSTATIONS_STATE_WR     0x12C
#define PC_RSP_TESTER_STATE_DETECT    0x12D    /* respones msg of check tester status */

/* ģ��Ӳ���汾����*/
#define LEADCORE_V1             0
#define LEADCORE_V2             1
#define LEADCORE_V3             2
#define TWMITRASTAR_V1     3
#define DTLINKTESTER_V1     4
#define DTLINKTESTER_V2     5
#define SAMSUNGKDDI_V1    6
#define DMD3A_JPALPS_V1                   7
#define SAMSUNGKDDI_V2    8
#define UNKNOWN                 0xFF

/* IO�忨Ӳ���汾���ͺ궨��*/
#define DTVL3100                0	/*3100 OBU/RSU*/
#define DTVL3000                1	/*3000 OBU/RSU*/
#define DTVL3100_VBOX       2
#define DTVL3110                3	/*3110 RSU*/
#define UMCC1_EVB               4
#define SUZUNE_EVK_ES1      5
#define VU300X                      6
#define NR_MASTER            7
#define NR_SLAVE            8
#define VU3205				9
#define DTVL3100_RSU       10
#define VU400X       11
#define SUZUNE3       12
#define DTVL3110E               13	/*3110E RSU, base on DMD3A*/
#define UNKNOWN                 0xFF

#define SUBBOARD1_PKB_V1	0

#define SUBBOARD2_SUWB_V1	0



/* hardware type */
#define UU_HARDWARE_VERSION 1
#define DMD3A_HARDWARE_VERSION 2
#define MODULE_HARDWARE_VERSION 3
#define IO_HARDWARE_VERSION 4
#define SUBBOARD1_HARDWARE_VERSION 5
#define SUBBOARD2_HARDWARE_VERSION 6

/* record test define */
#define TESTSTATIONS_NUM 6
#define TESTS_NUM 50

#define TESTS_RESULT_SUCCESS 1
#define TESTS_RESULT_FAIL 0
#define TESTS_RESULT_NULL 2

#define TESTS_DO 1
#define TESTS_NOT_DO 1

#define STATION_STATE_SUCCESS 1
#define STATION_STATE_FAIL 0

/* test stations state */
#define SINGLE_BOARD_STATE 0
#define BEFORE_AGING_STATE 1
#define START_AGING_TEST 2
#define AFTER_AGING_STATE 3
#define FACTORY_REST_STATE 4
#define DEBUGGING_STATE 5

#define SN_MAX_LEN           32            /* SN����󳤶ȣ���Ҫ����32*/

#define KEY_COMMA           0x2C        /* ����*/
#define KEY_SPACE             0x20        /* �ո�*/
#define KEY_LF                    0xa        /* \n���з���������ǰλ�õ���һ�У�������ص�����*/
#define KEY_CR                   0xd        /* \r�س������ص���ǰ�е����ף������ỻ����һ��*/

typedef unsigned int                u32;
typedef unsigned short              u16;
typedef unsigned char               u8;
typedef int                                 s32;
typedef short                           s16;
typedef char                            s8;

typedef unsigned int * uint32_ptr;
typedef unsigned char * uint8_ptr;
typedef volatile uint32_t   REGISTER;

/* These defines general macro */
#ifndef ERROR
#define ERROR       -1            
#endif

#ifndef FALSE
#define FALSE       0
#endif

#ifndef TRUE
#define TRUE            1
#endif

#ifndef NULL
#define NULL            0
#endif

/* ��λ���·���������Ϣ*/
typedef struct tag_STR_REQ_MESSAGE
{
    u16 u16MessageID;                       /* ��ϢID*/
    u16 u16Length;                              /* ��Ϣ����*/
}STR_REQ_MESSAGE;

/* ��λ���·�дIO�忨SN��������Ϣ*/
typedef struct tag_STR_IO_BOARD_SN
{
    u16 u16MessageID;                       /* ��ϢID*/
    u16 u16Length;                              /* ��Ϣ����*/
    u8 u8BoardSN[SN_MAX_LEN];        /* IO�忨��ģ��sn��*/
} STR_IO_BOARD_SN;

/* ��λ���·���Ӳ���汾����Ϣ*/
typedef struct tag_STR_HDWARE_VER_DETECT
{
    u16 u16MessageID;                       /* ��ϢID*/
    u16 u16Length;                              /* ��Ϣ����*/
    u8 u8ModType;                               /* DMDģ��Ӳ������*/
    u8 u8reserve;                               /* �����ֶ�*/
} STR_HDWARE_VER_DETECT;

/* ��λ���·�д�豸SN��������Ϣ*/
typedef struct tag_STR_EQUIP_SN
{
    u16 u16MessageID;           /* ��ϢID*/
    u16 u16Length;                  /* ��Ϣ����*/
    u8 u8EquipSN[20];               /* �豸sn��*/
} STR_EQUIP_SN;

/* ��λ���·�����SN�ż������Ϣ*/
/* 
* ��־λflag��
* 1��IO�忨SN�ż��
* 2���豸SN�ż��
* 3��IO�忨SN�ź��豸SN��ͬʱ���
* 4��δ�����sn�ų�����0��sn��ֵ�ճ���
*/
typedef struct tag_STR_SN_CHECK
{
    u16 u16MessageID;           /* ��ϢID*/
    u8 flag;                                /* ��־λ*/
    u16 u16BoardLength;         /* IO�忨SN�ų���*/
    u16 u16EquipLength;         /* �豸SN�ų���*/
    u8 u8BoardSN[20];           /* IO�忨sn��*/
    u8 u8EquipSN[20];           /* �豸sn��*/
} STR_SN_CHECK;

/* ��λ���ظ����Խ����Ϣ*/
typedef struct tag_STR_RSP_MESSAGE
{
    u16 u16MessageID;                       /* ��ϢID*/
    u16 u16Length;                              /* ��Ϣ����*/
    u8 result_flag;                              /* ��λ�����Խ����0: false;  1: success*/
    u8 u8reserve;                               /* �����ֶ�*/
} STR_RSP_MESSAGE;

/* ��λ���ظ�GPS����Ȳ�ѯ�����Ϣ*/
typedef struct tag_STR_GNSS_GPGSV_CN0
{
    u16 u16MessageID;                       /* ��ϢID*/
    u16 u16Length;                              /* ��Ϣ����*/
    u8 u8CN0[20];                              /* �ź������ȣ�0~99��������λdbHz*/
    u8 u8reserve;                               /* �����ֶ�*/
} STR_GNSS_GPGSV_CN0;

/* ��λ���ظ�SN�ż������Ϣ*/
/*
* ��־λflag��
* 1��IO�忨SN�ż��
* 2���豸SN�ż��
* 3��IO�忨SN�ź��豸SN��ͬʱ���
* 4��δ������������0
*/
typedef struct tag_STR_SN_CHECK_RESULT
{
    u16 u16MessageID;           /* ��ϢID*/
    u8 flag;                                /* ��־λ*/
    u8 u8BoardResult;               /* IO�忨SN�ż����*/
    u8 u8EquipSResult;              /* �豸sn�ż����*/
} STR_SN_CHECK_RESULT;

/* ��λ���ظ���ѯģ��̼��汾����Ϣ*/
typedef struct tag_STR_VERSION_CHECK_RESULT
{
    u16 u16MessageID;           /* ��ϢID*/
    u16 u16Length;                  /* ��Ϣ����*/
    u8 u8Version[50];           /* �汾�� */
} STR_VERSION_CHECK_RESULT;

typedef struct tag_STR_CID_CHECK_RESULT
{
    u16 u16MessageID;           /* ��ϢID*/
    u16 u16Length;                  /* ��Ϣ����*/
    u8 u8CIDResult[50];         /* CID�� */
} STR_CID_CHECK_RESULT;

typedef struct tag_STR_EQUIP_SN_Add
{
    u16 u16MessageID;           				/* ��ϢID*/
    u16 u16Length;                  			/* ��Ϣ����*/
    u8 u8EquipSN[25];                      	/* ���ӵ��豸sn��*/
} STR_EQUIP_SN_Add;

typedef void (*HW_TEST_FUNC)(void);
typedef struct tag_TEST_ITEM
{
    char * pname;
    HW_TEST_FUNC func;
}TEST_ITEM;

typedef struct tag_STR_HDWARE_VER_RD
{
    u16 u16MessageID;            /* msg id*/
    u16 u16Length;               /* msg len*/
    u8 HardWareType;			 /* hardware type*/
}STR_HDWARE_VER_RD;

typedef struct tag_STR_HDWARE_VER_RD_RESULT
{
    u16 u16MessageID;                /* msg id */
    u8 HardWareType;			     /* hardware type*/
    u16 u16Length;                   /* msg len */
    u8 u8HardWareVersionResult[50];	 /* hardware version */
} STR_HDWARE_VER_RD_RESULT;

typedef struct tag_STR_TESTSTATIONS_STATE_WR
{
    u16  u16MessageID;           /* msg id*/
    u16  u16Length;              /* msg len */
    u8  TestStationType;         /* station type*/
    u8  result_flag;             /*testcase result; 0: false;  1: success*/
} STR_TESTSTATIONS_STATE_WR ;

typedef struct tag_STR_TESTSTATIONS_STATE_CHECK_RESULT
{
    u16 u16MessageID;
    u8 BoardTestState;
    u8 BeforeAgingTestState;
    u8 StartAgingTestState;
    u8 AfterAgingTestState;
    u8 ResetSetState;
    u8 DevelopTestState;
} STR_TESTSTATIONS_STATE_CHECK_RESULT;

int create_socket(int argc, char *argv);
int message_consume_handle(char *buf, int length, int fd);
int message_produce_handle(void *up_msg, int length, int type, int fd);
void DisplayList(TEST_ITEM* list);
int GetListNum(TEST_ITEM* list);
int Char_Judge(char* ascii, int num);
unsigned int asctohex(char* ascii, unsigned int* val, int num);
unsigned int ASCToInt(char* ascii, int* val, int num);
int Input_Key_Check(char* p_buf, int* p_val, int length);
int Acq_Input_Key(char* p_buf, int length);
void Set_Module_Base_Addr(void);

#endif
