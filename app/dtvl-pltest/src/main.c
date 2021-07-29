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
/******************************* �����ļ����� *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

/* for select*/
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>

/* for bzero*/
#include <strings.h>
#include <string.h>

#include "main.h"
#include "equipment.h"
#include "utils.h"
#include "gpio_operate.h"

/******************************* �ֲ����������Ͷ��� **************************/

/******************************* ��ͳ������� *********************************/
#define BUFF_SIZE               1024
#define backlog                     3
#define PORT                        51888       /* TCP/IP ���Ӷ˿ں�*/
#define CLI_NUM                 3               /* ��������TCP�ͻ�������*/
#define MAX_TIMEOUT         60          /*�ȴ�����������ʱʱ�䣬��λs*/
#define PARAM_CNT             10            /* ��λ���´�����������*/
#define PARAM_LEN             26            /* ��λ���´�����������󳤶�*/

#define INPUT_ERR       0
#define INPUT_OK        1
#define INPUT_GO        2
#define INPUT_QUIT      3 
#define ZELAI_TEST 0
#define CTAO_TEST 1

/******************************* ȫ�ֱ�������/��ʼ�� *************************/
char log_buf[MAX_LOG_BYTES] = {0};
FILE *log_file_ptr = NULL;

int serv_sock = 0;
struct sockaddr_in serv_addr = {0};

int client_fds[CLI_NUM] = {0};

static int Test_Flag = TRUE;
static int Item_num = 0;

void *SSI0_MODEULE_BASE = NULL;
void *SSI1_MODEULE_BASE = NULL;
void *SSI2_MODEULE_BASE = NULL;
void *I2C0_MODEULE_BASE = NULL;
void *I2C1_MODEULE_BASE = NULL;
void *I2C2_MODEULE_BASE = NULL;
void *I2C3_MODEULE_BASE = NULL;
void *COM_I2C_MODEULE_BASE = NULL;
void *MUX_PIN_MODEULE_BASE = NULL;
void *GPIO_MODEULE_BASE = NULL;
void *UART0_MODEULE_BASE = NULL;
void *UART1_MODEULE_BASE = NULL;
void *UART2_MODEULE_BASE = NULL;
void *COM_UART_MODEULE_BASE = NULL;
void *AP_PWR_MODEULE_BASE = NULL;
void *AP_CTL_MODEULE_BASE = NULL;
void *SDMMC2_MODEULE_BASE = NULL;
void *DDR_PWR_MODEULE_BASE = NULL;

/********************��ӡ�Ĳ�����ѡ��***********************/
TEST_ITEM TestList[] =
{
    //1.menu name         2.function name

    {" GPS ALPS test     ", gps_alps_test},
    {" UART0 test     ", uart0_test},
    {" UART2 test     ", uart2_test},
    {" Module Hardware Version", module_hardware_ver_test},
    {" ADC Convert       ", adc_convert_test},
    {" TF test        ", tcard_test},
    {" EMMC test          ", emmc_test},
    {" GPIO write test  ", gpio_write_test},
    {" GPIO read test  ", gpio_read_test},
    {" GPIO loopback test ", gpio_loopback_test},
    {" I2C0 E2PROM test ",  i2c0_e2prom_test},
    {" Sensor test        ", sensor_test},
    {" SPI0 CAN test     ", ssi0_can_test},
    {" HSIC  test          ", usb_hsic_test},
    {" v2x 1pps test   ", v2x_1pps_test},
    {" MODEM RF test     ", modem_rf_test},
    {" LC1160 read&write test  ", pmu_lc1160_reg_set_test},
    {" Power Off & Reset  ", system_opt},

    {NULL, NULL}
};

char *io_board_ver_home[] = {
	{"DTVL3100"}, /*3100 OBU/RSU*/
	{"DTVL3000"}, /*3000 OBU/RSU*/
	{"DTVL3100_VBOX"},
	{"DTVL3110"}, /*3110 RSU*/
	{"UMCC1_EVB"},
	{"SUZUNE_EVK_ES1"},
	{"VU300X"},
	{"NR_MASTER"},
	{"NR_SLAVE"},
	{"VU3205"},
	{"DTVL3100_RSU"},
	{"VU400X"},
	{"SUZUNE3"},
	{"DTVL3110E"},
	{"UNKNOWN"}
};
char *module_board_ver_home[] = {
	{"LEADCORE_V1"},
	{"LEADCORE_V2"},
	{"LEADCORE_V3"},
	{"TWMITRASTAR_V1"},
	{"DTLINKTESTER_V1"},
	{"DTLINKTESTER_V2"},
	{"SAMSUNGKDDI_V1"},
	{"DMD3A_JPALPS_V1"},
	{"SAMSUNGKDDI_V2"},
	{"UNKNOWN"}
};
char *sub_board1_ver_home[] = {
	{"PKB_V1"}
};
char *sub_board2_ver_home[] = {
	{"SUWB_V1"}
};

extern int hwver;

extern void *map_module(int module);
extern void gps_alps_test(void);
extern int uart1_gps_test(void);
extern int gnss_cn0_test(int* cn0);
extern int uart0_test(void);
extern void uart2_test(void);
extern int module_hardware_ver_test(void);
extern int io_board_hardware_ver_test(void);
extern int module_firmware_ver_test(char *ver_buf);
extern int signature_check_test(void);
extern int module_cid_check_test(void *cid);
extern void adc_convert_test(void);
extern void tcard_test(void);
extern int emmc_test(void);
extern void gpio_write_test(void);
extern void gpio_read_test(void);
extern void gpio_loopback_test(void);
extern void i2c0_e2prom_test(void);
extern void i2c0_e2prom_init(void);
extern void sensor_test(void);
extern int v2x_1pps_test(void);
extern void pmu_lc1160_reg_set_test(void);
extern int usb_hsic_test(void);
extern int usb_host_test(void);
extern int usb_otg_test(void);
extern int usb_LAN_test(char *buffer);
extern int usb_network_eth_test(void);
extern int module_wifi_test_wlan0(void);
extern int module_wifi_test_mlan0(void);
extern int public_network_test(void);
extern int can_interface_test(void);
extern void system_opt(void);
extern int spi_hsm_detect(int num);
extern int ssi0_can_test(void);
extern void modem_rf_test(void);
extern void pmu_lc1160_test(void);
extern int uart2_bt_test(void);
extern int sdio2_wifi_test(void);

/****************************functions*************************************/
/************************************************************************
* ������: create_log_file 
* ��������: ����������־�ļ�
* �������: 
* �������: ��
* ����ֵ: �� 
* ��ע:     a+������ÿ�β������ļ���־��ÿ�β��Բ�����
*                              ������־�ļ�ĩβ׷����־����
*
*************************************************************************/
static void create_log_file(void)
{
    log_file_ptr = fopen(LOG_FILE_PATH, "a+");
}

/************************************************************************
* ������: close_log_file 
* ��������: ���沢�ر���־�ļ�
* �������: 
* �������: ��
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
static void close_log_file(void)
{
    fflush(log_file_ptr);
    fclose(log_file_ptr);
}

/************************************************************************
* ������: main 
* ��������: ���������ں���
* �������: 
* �������: ��
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int main(int argc, char *argv[])
{
    int val = 0;
    int ret = 0;
    int i = 0;
    int flags = -1;
    int clnt_sock = 0;
    socklen_t clnt_addr_size = 0;
    struct sockaddr_in clnt_addr = {0};
    char buf[BUFF_SIZE] = {0};
    char rx_buf[BUFF_SIZE] = {0};
    char tx_buf[BUFF_SIZE] = {0};
    int num = 0;
	int check_init;
    char full_message[] = "the client is full!can't join!\n";

    create_log_file();

	/* note it: maybe its not ok, when power off or dtvl-pltest restart, this will rewrite eeprom
       now, iwill juage read eeprom value, 0 is fail, 1 is sucess, other is not test
	*/
	#if 0
	check_init = init_testcase_record();
	if(0!=check_init)
	{
		HARDWARE_TEST("WARNING: init eeprom test_record data fail\n");
	}
	#endif
    HARDWARE_TEST("Begin Production Line Test!\n");

    Set_Module_Base_Addr();
    /* ����I2C0~I2C3��Ƶ��Ϊ23*/
    *(volatile unsigned *)(AP_PWR_MODEULE_BASE + AP_PWR_I2CCLK_CTL) = 0x17 |0x17<<8 | 0x17<<16 | 0x17<<24;
    /* ��ʼ��1160*/
    pmu_lc1160_init();

    /* �����׽��֣����ȴ���λ������*/
    /*if (argc != 3)
    {
        HARDWARE_TEST("Usage: %s <port>\n", argv[0]);
        goto err1;
    }*/
    ret = create_socket(argc, argv);
    if (-1 == ret)
    {
        HARDWARE_TEST("Create socket failed!\n");
        return 0;
    }

    /* fd_set*/
    fd_set ser_fdset;
    int max_fd = 1;
    struct timeval mytime;
    HARDWARE_TEST("wait for client connnect!\n");

    while (1)
    {
        mytime.tv_sec = MAX_TIMEOUT;
        mytime.tv_usec = 0;
 
        FD_ZERO(&ser_fdset);
#ifdef PLTEST_PRT_DEBUG
        /* ���ӱ�׼����select���*/
        FD_SET(STDIN_FILENO, &ser_fdset);
        if(max_fd < 0)
        {
            max_fd = 0; 
        }
#endif
        /* ���ӷ����socket��select���*/
        FD_SET(serv_sock, &ser_fdset);
        if(max_fd < serv_sock)
        {
            max_fd = serv_sock;
        }

        /* ���ӿͻ���socket��select���*/
        for(i=0; i<CLI_NUM; i++)  /* �����鶨�����ͻ���fd*/
        {
            if(client_fds[i]!=0) 
            {
                FD_SET(client_fds[i], &ser_fdset);
                if(max_fd < client_fds[i])
                {
                    max_fd = client_fds[i]; 
                }
            }
        }

        /* select��·����*/
        ret = select(max_fd + 1, &ser_fdset, NULL, NULL, &mytime);
        if(ret < 0)    
        {    
            HARDWARE_TEST("select failure\n");    
            continue;    
        }
        else if(ret == 0)
        {
            HARDWARE_TEST("time out!\n");
            continue;
        }
        else
        {
#ifdef PLTEST_PRT_DEBUG
             /* ��⵽�б�׼����*/
            if(FD_ISSET(STDIN_FILENO, &ser_fdset))
            {
                bzero(buf, sizeof(buf));
                //fgets(buf, BUFF_SIZE-1, stdin);
                num = read(STDIN_FILENO, buf, BUFF_SIZE-1);
                if(num < 0)
                {
                    HARDWARE_TEST("Can't read data from standard input.\n\r");
                    continue;
                }
                /*ֻ����0~num-1���ַ����Ӷ�ɾ���س���*/
                num = num-1;
                strncpy(tx_buf, buf, num);
                strcat(tx_buf, "\0");
                HARDWARE_TEST("read date from standard input: %s\n", tx_buf);
 #if 1
                message_consume_handle(tx_buf, num, STDOUT_FILENO);
 #else
                for(i=0; i<CLI_NUM; i++)
                {
                    if(client_fds[i] != 0)
                    {
                        HARDWARE_TEST("client_fds[%d]: %d\n", i, client_fds[i]);
                        send(client_fds[i], tx_buf, BUFF_SIZE, 0);
                    }
                }
#endif
            }
#endif
            /* ��⵽�пͻ������ӷ����*/
            if(FD_ISSET(serv_sock, &ser_fdset)) 
            {
                clnt_addr_size = sizeof(clnt_addr);
                clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
                if(clnt_sock > 0)
                {
                    flags = -1;
                    /* ����һ���ͻ��ˣ������һ��fd *
                      * �������CLI_NUM���ͻ��ˣ����ӵ�CLI_NUM+1������forѭ��*
                      * flags���±���ֵΪ-1 */
                    for(i=0; i<CLI_NUM; i++)
                    {
                        if(client_fds[i] == 0)
                        {
                            flags = i; 
                            client_fds[i] = clnt_sock;
                            break;
                        }
                    }
 
                    if (flags >= 0)
                    {
                        HARDWARE_TEST("new user client[%d] add sucessfully!\n", flags);
                    }
                    else /* flags = -1*/
                    {
                        bzero(buf, BUFF_SIZE);
                        strncpy(buf, full_message, 100);
                        send(clnt_sock, buf, BUFF_SIZE, 0);
                        /* �����ټ���ͻ��ˣ����ԣ��رճ������µ�����*/
                        close(clnt_sock);
                    }
                }
                HARDWARE_TEST("accept new socket: %d\n", clnt_sock);
                HARDWARE_TEST("connected with ip: %s and port: %d\n", inet_ntop(AF_INET, &clnt_addr.sin_addr, buf, 1024), ntohs(clnt_addr.sin_port));
            }
        }

        /* �����ͻ��˷���������Ϣ*/
        for(i=0; i<CLI_NUM; i++)
        {
            if(client_fds[i] != 0)
            {
                if(FD_ISSET(client_fds[i], &ser_fdset))
                {
                    bzero(rx_buf, BUFF_SIZE);
                    num = read(client_fds[i], rx_buf, BUFF_SIZE-1);
                    if(num > 0)
                    {
                        HARDWARE_TEST("message form client[%d]:%s\n", i, rx_buf);
                        /* ������������Ϣ����������������λ���·�������*/
                        message_consume_handle(rx_buf, num, client_fds[i]);
                    }
                    else if(num < 0)
                    {
                        HARDWARE_TEST("rescessed error!");
                    }
                    /* ĳ���ͻ����˳�*/
                    else  /* cancel fdset and set fd=0*/
                    {
                        /* ��read����С�ڵ���0�����յ�EINTR�źţ���Է�δ�Ͽ����ӣ�ֻ�Ƕ�ȡ��0�ֽ�����*/
                        if (errno == EINTR)
                        {
                            HARDWARE_TEST("read client 0 byte data.\n");
                        }
                        else
                        {
                            HARDWARE_TEST("clien[%d] exit!\n", i);
                            FD_CLR(client_fds[i], &ser_fdset);
                            client_fds[i] = 0;
                            continue;  /* �������break��һ���ͻ����˳�����ɷ����Ҳ�˳�*/
                        }
                    }
                }
            }
        }
    }

    for ( i=0; i<CLI_NUM; i++)
    {
        if(client_fds[i] != 0)
        {
            close(client_fds[i]);
        }
    }
    close(serv_sock);
    close_log_file();

    return 0;
}

/************************************************************************
* ������: create_socket 
* ��������: ����TCP ����ˣ�֧������dtvl_pltest����ʱ�´�����
* �������: int argc: ����dtvl_pltestʱ�����������
*                           char *argv[]: �����ӿͻ���ip��ַ��port�˿ں�
* �������: 0: success
*                           -1: fail
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int create_socket(int argc, char *argv)
{
    int ret = 0;

    /* �����׽��֣����ȴ���λ������*/
    /*if (argc != 3)
    {
        HARDWARE_TEST("Usage: %s <port>\n", argv[0]);
        ret = -1;
        goto err1;
    }*/

    serv_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (serv_sock < 0)
    {
        HARDWARE_TEST("socket failure\n");
        ret = -1;
        goto err1;
    }
    HARDWARE_TEST("server socket: %d\n", serv_sock);

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;                                 /* IPV4*/
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);      /* ���������е�ַ*/
    serv_addr.sin_port = htons(PORT);
    //serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
    //serv_addr.sin_port = htons(atoi(argv[2]));

    /* �������ñ��ص�ַ�Ͷ˿�*/
    //int optval = 1;
    //setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    if (bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        HARDWARE_TEST("bind failure\n");
        ret = -1;
        goto err2;
    }

    /* �����������������г���*/
    if (listen(serv_sock, backlog) < 0)
    {
        HARDWARE_TEST("listen failure\n");
        ret = -1;
        goto err2;
    }

    return ret;

err2:
    close(serv_sock);

err1:

    return ret;
}

/************************************************************************
* ������: message_consume_handle 
* ��������: ��λ���·�����Ϣ��������
* �������: char *buf: ��λ���·�����Ϣ�ִ�
*                           int length: ��λ���·�����Ϣ����
*                           int fd: ��ͻ�������£�ָ����Ϣ�Ŀͻ���
* �������: 0: success
*                           -1: fail
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int message_consume_handle(char *buf, int length, int fd)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    int cnt = 0;
    int flag = 0;
    int value = 0;
    int version = 0;
    unsigned short msg_type = 0;
    unsigned short msg_len = 0;
    char message_buf[BUFF_SIZE] = {0};
    char param_buf[PARAM_CNT][PARAM_LEN] = {0};
    int param_len[PARAM_CNT] = {0};
    char sn[PARAM_LEN] = {0};
    char sn2[PARAM_LEN] = {0};
    int cn0[20] = {0};
    int teststations_current;
    STR_IO_BOARD_SN rsp_bd_sn;
	STR_EQUIP_SN_Add rsp_oem_id;
    STR_EQUIP_SN rsp_eq_sn;
    STR_SN_CHECK req_check_sn;
    STR_SN_CHECK_RESULT rsp_check_sn;
    STR_HDWARE_VER_DETECT hw_ver;
    STR_GNSS_GPGSV_CN0 rsp_gnss_cn0;
    STR_RSP_MESSAGE rsp_msg;
    STR_VERSION_CHECK_RESULT rsp_firm_msg;
    STR_CID_CHECK_RESULT rsp_cid_msg;
	STR_HDWARE_VER_RD req_hw_ver;
	STR_HDWARE_VER_RD_RESULT rsp_hw_ver;
	STR_TESTSTATIONS_STATE_CHECK_RESULT station_test_result;

	i2c0_e2prom_init();
    /* ������λ���·������в����ַ�*/
    strncpy(message_buf, buf, length);

    /* ���������λ���·��Ĳ���*/
    for (i=0; i<length; i++)
    {
        //HARDWARE_TEST("recv char %d: 0x%x\n", i, message_buf[i]);
        /* ���ֶ��ţ����涺��ǰ�Ĳ���ֵ*/
        if (KEY_COMMA == message_buf[i])
        {
            if ((cnt >= (PARAM_CNT - 1)) ||((i - flag) > PARAM_LEN))
            {
                HARDWARE_TEST("parameters error!\n");
                break;
            }

            strncpy(param_buf[cnt], message_buf + flag, i - flag);
            param_len[cnt] = i - flag;    /* ����ÿ�������ַ�����*/
            flag = i + 1;   /* �����ƶ����ַ�*/
            //HARDWARE_TEST("param_buf[%d]: %s message_buf+%d: %s\n", cnt, param_buf[cnt], flag, message_buf + flag);
            HARDWARE_TEST("parameters buffer %d: %s\n", cnt, param_buf[cnt]);
            cnt++;
        }

        if (i == (length-1))
        {
            if ((cnt >= PARAM_CNT) ||((length - flag) > PARAM_LEN))
            {
                HARDWARE_TEST("parameters error!\n");
                break;
            }
            /* ���һ����������*/
            strncpy(param_buf[cnt], message_buf + flag , length - flag);
            param_len[cnt] = length - flag;   /* ���һ�������ַ�����*/
            //HARDWARE_TEST("param_buf[%d]: %s message_buf+%d: %s\n", cnt, param_buf[cnt], flag, message_buf + flag);
            HARDWARE_TEST("parameters buffer %d: %s\n", cnt, param_buf[cnt]);
        }
    }

    /* ��λ���·���Ϣ����*/
    ret = asctohex(param_buf[0], &msg_type, param_len[0]);
    HARDWARE_TEST("get msg_type %d from minjie.fan!\n", msg_type);
    if (0 == ret)
    {
        HARDWARE_TEST("character string hex change failed!\n");
        return 0;
    }

    ret = asctohex(param_buf[1], &msg_len, param_len[1]);
    HARDWARE_TEST("get msg_len %d from minjie.fan!\n", msg_len);
    if (0 == ret)
    {
        HARDWARE_TEST("character string hex change failed!\n");
        return 0;
    }

	/* set test result -> NULL */
	rsp_msg.result_flag = TESTS_RESULT_NULL;

    switch (msg_type)
    {
        /* PC��λ����������*/
        case PC_REQ_UE_MESSAGE:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_UE_MESSAGE)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_PC_MESSAGE;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = 1;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_PC_MESSAGE, fd);

            break;
        }
        /* дIO�忨SN�Ų�������*/
        case PC_REQ_IO_BOARD_SN_WR:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_SN_WR)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(sn, param_buf[2], msg_len);
            HARDWARE_TEST("write IO Board SN length: 0x%x SN value: %s\n", msg_len, sn);

            int open4iosn, ret;

            open4iosn=open(EEPROM_FILE,O_RDWR);
            if(open4iosn<0)
            {
                perror("open error");
            }
            init_i2c_ioctrl(open4iosn);

            if(msg_len>EEPROM_PAGE_SIZE)
            {
                write_sn(open4iosn, EEPROM_IOSN_ADDR, EEPROM_PAGE_SIZE, sn);
                write_sn(open4iosn, (EEPROM_IOSN_ADDR+EEPROM_PAGE_SIZE), (msg_len-EEPROM_PAGE_SIZE), (sn+EEPROM_PAGE_SIZE));
            }
            else
            {
                write_sn(open4iosn, EEPROM_IOSN_ADDR, msg_len, sn);
            }
            sleep(1);
            read_sn(open4iosn, EEPROM_IOSN_ADDR, msg_len, sn);
            ret = strcmp(param_buf[2], sn);
            if (0 == ret)
            {
                /* д��Ͷ���һ��*/
                rsp_msg.result_flag = 1;
            }
            else
            {
                rsp_msg.result_flag = 0;
            }
            close(open4iosn);

            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_SN_WR;
            rsp_msg.u16Length = 1;
            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d\n",
                rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_SN_WR, fd);

            break;
        }
		 /* �����豸SN��д��, дIO�忨SN�Ų�������*/
        case PC_REQ_EQUIP_ADD_SN_WR_TEST :
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_EQUIP_ADD_SN_WR_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(sn, param_buf[2], msg_len);
            HARDWARE_TEST("write OEN length: 0x%x SN value: %s\n", msg_len, sn);

            int open4oemsn, ret;
            open4oemsn=open(EEPROM_FILE,O_RDWR);
            if(open4oemsn<0)
            {
                perror("open error");
            }
            init_i2c_ioctrl(open4oemsn);
            if(msg_len>EEPROM_PAGE_SIZE)
            {
                write_sn(open4oemsn, EEPROM_OEMSN_ADDR, EEPROM_PAGE_SIZE, sn);
                write_sn(open4oemsn, (EEPROM_OEMSN_ADDR+EEPROM_PAGE_SIZE), (msg_len-EEPROM_PAGE_SIZE), (sn+EEPROM_PAGE_SIZE));
            }
            else
            {
                write_sn(open4oemsn, EEPROM_OEMSN_ADDR, msg_len, sn);
            }
            sleep(1);
            read_sn(open4oemsn, EEPROM_OEMSN_ADDR, msg_len, sn);
            ret = strcmp(param_buf[2], sn);
            if (0 == ret)
            {
                /* д��Ͷ���һ��*/
                rsp_msg.result_flag = 1;
            }
            else
            {
                rsp_msg.result_flag = 0;
            }
            close(open4oemsn);

            rsp_msg.u16MessageID = UE_RSP_EQUIP_ADD_SN_WR_TEST;
            rsp_msg.u16Length = 1;
            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d\n",
                rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_EQUIP_ADD_SN_WR_TEST, fd);

            break;
        }
        /* ��IO�忨SN�Ų�������*/
        case PC_REQ_IO_BOARD_SN_RD:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_SN_RD) len %d\n", msg_type, msg_len);

            memset(&rsp_bd_sn, 0, sizeof(STR_IO_BOARD_SN));
            rsp_bd_sn.u16MessageID = UE_RSP_IO_BOARD_SN_RD;

            extern unsigned char sn_io[33];
            int open4iosn,ret;
            open4iosn=open(EEPROM_FILE,O_RDWR);
            if(open4iosn<0)
            {
                perror("open error");
            }
            init_i2c_ioctrl(open4iosn);
            read_sn(open4iosn, EEPROM_IOSN_ADDR, msg_len, rsp_bd_sn.u8BoardSN);
            close(open4iosn);

            rsp_bd_sn.u16Length = msg_len;
            HARDWARE_TEST("read IO Board SN length: 0x%x SN value: %s\n", rsp_bd_sn.u16Length, rsp_bd_sn.u8BoardSN);
            message_produce_handle((void*)&rsp_bd_sn, sizeof(STR_IO_BOARD_SN), UE_RSP_IO_BOARD_SN_RD, fd);

            break;
        }
		case PC_REQ_EQUIP_OEM_RD:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_EQUIP_OEM_RD)\n", msg_type);

            memset(&rsp_oem_id, 0, sizeof(STR_EQUIP_SN_Add));
            rsp_oem_id.u16MessageID = UE_RSP_EQUIP_OEM_RD;
			HARDWARE_TEST("PC_REQ_EQUIP_OEM_RD read OEM_sn:\n");

            int open4oemsn,ret;
            open4oemsn=open(EEPROM_FILE,O_RDWR);
            if(open4oemsn<0)
            {
                perror("open error");
            }
            init_i2c_ioctrl(open4oemsn);
            read_sn(open4oemsn, EEPROM_OEMSN_ADDR, msg_len, rsp_oem_id.u8EquipSN);
            close(open4oemsn);

            rsp_oem_id.u16Length = msg_len;
            HARDWARE_TEST("read IO Board SN length: 0x%x SN value: %s\n", rsp_oem_id.u16Length, rsp_oem_id.u8EquipSN);
            message_produce_handle((void*)&rsp_oem_id, sizeof(STR_EQUIP_SN_Add), UE_RSP_EQUIP_OEM_RD, fd);

            break;
        }
        /* EMMC����������Ϣ*/
        case PC_REQ_MODULE_EMMC_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_MODULE_EMMC_DETECT)\n", msg_type);

            value = emmc_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_MODULE_EMMC_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_MODULE_EMMC_DETECT, fd);

            break;
        }
        /* ģ��Ӳ���汾�Ų�������*/
        case PC_REQ_MODULE_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_MODULE_HDWARE_VER_DETECT)\n", msg_type);

            version = module_hardware_ver_test();
            HARDWARE_TEST("module ver: %d\n", version);
            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            hw_ver.u16MessageID = UE_RSP_MODULE_HDWARE_VER_DETECT;
            hw_ver.u16Length = 1;
            hw_ver.u8ModType = version;
            message_produce_handle((void*)&hw_ver, sizeof(STR_HDWARE_VER_DETECT), UE_RSP_MODULE_HDWARE_VER_DETECT, fd);

            break;
        }
        /* SDIO2�ӿڲ�������*/
        case PC_REQ_MODULE_SDIO2_WIFI_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_MODULE_SDIO2_WIFI_TEST)\n", msg_type);

            value = 0/*sdio2_wifi_test()*/;
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_MODULE_SDIO2_WIFI_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_MODULE_SDIO2_WIFI_TEST, fd);

            break;
        }
        /* UART2��������*/
        case PC_REQ_MODULE_UART2_BT_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_MODULE_UART2_BT_TEST)\n", msg_type);

            value = uart2_bt_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_MODULE_UART2_BT_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_MODULE_UART2_BT_TEST, fd);

            break;
        }
        /* IO�忨Ӳ���汾�Ų�������*/
        case PC_REQ_IO_BOARD_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_HDWARE_VER_DETECT)\n", msg_type);

            if (0xff == hwver)
            {
                /* ADC������β���϶�ȡ����ֶ�ȡ�������ݵ����*/
                hwver = io_board_hardware_ver_test();
            }
            HARDWARE_TEST("io board ver: %d\n", hwver);
            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            hw_ver.u16MessageID = UE_RSP_IO_BOARD_HDWARE_VER_DETECT;
            hw_ver.u16Length = 1;
            hw_ver.u8ModType = hwver;
            message_produce_handle((void*)&hw_ver, sizeof(STR_HDWARE_VER_DETECT), UE_RSP_IO_BOARD_HDWARE_VER_DETECT, fd);

            break;
        }
		case PC_REQ_SUBBOARD1_HDWARE_VER_DETECT:
		{
            HARDWARE_TEST("req msg ID: %d(PC_REQ_SUBBOARD1_HDWARE_VER_DETECT)\n", msg_type);
			value = child_board_hardware_ver_test(1);
            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
			hw_ver.u16MessageID = UE_RSP_SUBBOARD1_HDWARE_VER_DETECT;
            hw_ver.u16Length = 1;
            hw_ver.u8ModType = value;
            message_produce_handle((void*)&hw_ver, sizeof(STR_HDWARE_VER_DETECT), UE_RSP_SUBBOARD1_HDWARE_VER_DETECT, fd);
            break;
		}
		case PC_REQ_SUBBOARD2_HDWARE_VER_DETECT:
		{
            HARDWARE_TEST("req msg ID: %d(PC_REQ_SUBBOARD2_HDWARE_VER_DETECT)\n", msg_type);
			value = child_board_hardware_ver_test(2);
            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
			hw_ver.u16MessageID = UE_RSP_SUBBOARD2_HDWARE_VER_DETECT;
            hw_ver.u16Length = 1;
            hw_ver.u8ModType = value;
            message_produce_handle((void*)&hw_ver, sizeof(STR_HDWARE_VER_DETECT), UE_RSP_SUBBOARD2_HDWARE_VER_DETECT, fd);
			break;
		}
        /* UART1��������*/
        case PC_REQ_IO_BOARD_UART1_GNSS_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_UART1_GNSS_TEST)\n", msg_type);

            value = uart1_gps_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_UART1_GNSS_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_UART1_GNSS_TEST, fd);

            break;
        }
        /* PP1S��������*/
        case PC_REQ_IO_BOARD_PP1S_GNSS_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_PP1S_GNSS_TEST)\n", msg_type);

            value = v2x_1pps_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_PP1S_GNSS_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_PP1S_GNSS_TEST, fd);

            break;
        }
        /* GPS�����CN0ֵ��ѯ����*/
        case PC_REQ_IO_BOARD_CN0_GNSS_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_CN0_GNSS_TEST)\n", msg_type);

            value = gnss_cn0_test(cn0);
            memset(&rsp_gnss_cn0, 0, sizeof(STR_GNSS_GPGSV_CN0));
            rsp_gnss_cn0.u16MessageID = UE_RSP_IO_BOARD_CN0_GNSS_TEST;
            rsp_gnss_cn0.u16Length = 20;
            if (1 == value)
            {
                for(j=0; j<20; j++)
                {
                    rsp_gnss_cn0.u8CN0[j] = (char)cn0[j];
                    HARDWARE_TEST("rsp_gnss_cn0.u8CN0[%d]: 0x%x. cn0[%d]:0x%x\n", j, rsp_gnss_cn0.u8CN0[j], j, (char)cn0[j]);
                }
            }
            message_produce_handle((void*)&rsp_gnss_cn0, sizeof(STR_GNSS_GPGSV_CN0), UE_RSP_IO_BOARD_CN0_GNSS_TEST, fd);

            break;
        }
        /* COMUART��������*/
        case PC_REQ_IO_BOARD_COMUART_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_COMUART_TEST)\n", msg_type);

            value = comuart_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_COMUART_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_COMUART_TEST, fd);

            break;
        }
        /* USB OTG��������*/
        case PC_REQ_IO_BOARD_USB_OTG_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_USB_OTG_DETECT)\n", msg_type);

            value = usb_otg_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_USB_OTG_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_USB_OTG_DETECT, fd);

            break;
        }
        /* ETH���ڲ�������*/
        case PC_REQ_IO_BOARD_ETH_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_ETH_DETECT)\n", msg_type);

            value = usb_network_eth_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_ETH_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_ETH_DETECT, fd);

            break;
        }
        /* HSIC�ӿڲ�������*/
        case PC_REQ_IO_BOARD_USB_HSIC_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_USB_HSIC_DETECT)\n", msg_type);

            value = usb_hsic_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_USB_HSIC_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_USB_HSIC_DETECT, fd);

            break;
        }
        /* LAN9500��������*/
        case PC_REQ_IO_BOARD_LAN9500_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_LAN9500_DETECT)\n", msg_type);
            int io_version = 0;
            char LAN_buf[20];

            io_version = io_board_hardware_ver_test();
            if(DTVL3110E == io_version)
            {
                memcpy(LAN_buf, "ID 0424:7500", strlen("ID 0424:7500"));
            }
            else
            {
                memcpy(LAN_buf, "ID 0424:9e00", strlen("ID 0424:9e00"));
            }

            value = usb_LAN_test(LAN_buf);
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_LAN9500_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_LAN9500_DETECT, fd);

            break;
        }
        /* UART0��������*/
        case PC_REQ_IO_BOARD_UART0_TEST:
        {
			int io_version = 0;
			io_version = io_board_hardware_ver_test();
			HARDWARE_TEST("io version ID: %d ctao (PC_REQ_IO_BOARD_UART0_TEST)\n", io_version);
			if(VU3205 == io_version)
			{
				HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_UART0_TEST), test for V3205\n", msg_type);

	            value = uart0_gps_test4VU3205();
			}
			else
			{
	            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_UART0_TEST)\n", msg_type);

	            value = uart0_test();
			}
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
	        rsp_msg.u16MessageID = UE_RSP_IO_BOARD_UART0_TEST;
	        rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
	        message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_UART0_TEST, fd);
            break;
        }
        /* CAN�ӿڲ�������*/
        case PC_REQ_IO_BOARD_CAN_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_CAN_TEST)\n", msg_type);

            value = can_interface_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_CAN_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_CAN_TEST, fd);

            break;
        }
        /* SPI0�ӿڲ�������*/
        case PC_REQ_IO_BOARD_SPI0_CAN_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_SPI0_CAN_TEST)\n", msg_type);

            value = ssi0_can_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_SPI0_CAN_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_SPI0_CAN_TEST, fd);

            break;
        }
        /* I2C1�ӿڲ�������*/
        case PC_REQ_IO_BOARD_I2C1_SENSOR_TEST:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_I2C1_SENSOR_TEST)\n", msg_type);

            value = i2c1_sensor_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_I2C1_SENSOR_TEST;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_I2C1_SENSOR_TEST, fd);

            break;
        }
        /* WIFI���ܲ�������*/
        case PC_REQ_IO_BOARD_AP_WIFI_CONNECT_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_AP_WIFI_CONNECT_DETECT)\n", msg_type);
            int io_version = 0;
            io_version = io_board_hardware_ver_test();
            HARDWARE_TEST("io version ID: %d in PC_REQ_IO_BOARD_AP_WIFI_CONNECT_DETECT\n", io_version);
            /* Ublox Lily makes netcard mlan0, BCM4343S makes netcard wlan0 */
            if((VU400X==io_version)||(DTVL3110E==io_version))
            {
                value = module_wifi_test_mlan0();
            }
            else
            {
                value = module_wifi_test_wlan0();
            }
			
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_AP_WIFI_CONNECT_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_AP_WIFI_CONNECT_DETECT, fd);

            break;
        }
        /* SIM��������������*/
        case PC_REQ_IO_BOARD_RSU_NETWORK_ONLINE_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_RSU_NETWORK_ONLINE_DETECT)\n", msg_type);

            value = public_network_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_RSU_NETWORK_ONLINE_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_RSU_NETWORK_ONLINE_DETECT, fd);

            break;
        }
		case PC_REQ_IO_BOARD_RSU_SIM_STATE_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_RSU_SIM_STATE_DETECT)\n", msg_type);

			/* We used fixed values name_ttyUSB2 before,
			  * Now use dynamic lookup. Prevent ttyUSB interface disappearance.
			  */
			char *name_ttyUSB2 = "/dev/ttyUSB2";
			char *name_ttyUSBbase = "/dev/ttyUSB";
			int ttyUSB2_value = 0;
			ttyUSB2_value = 0;
			char msg[200] = {0};
			char *cmd = "AT+CPIN?\n";
			int ttyUSB_num = 0;
			char *name_ttyUSBnum;

			/* merge str of ttyUSB */
			name_ttyUSBnum = malloc(sizeof(name_ttyUSBbase)+1);
			ttyUSB_num = split_ttyUSB_num();
			HARDWARE_TEST("get ttyUSB num %d\n", ttyUSB_num);
			sprintf(name_ttyUSBnum, "%s%d", name_ttyUSBbase, ttyUSB_num);

			memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
			/* closed serial echo */
			system("stty -F /dev/ttyUSB2 -echo");
			ttyUSB2_value = simcard_state_test(name_ttyUSBnum, cmd, msg);
			if((1==ttyUSB2_value))
			{
				/* judge SIM is ok, use str READY or OK */
				if(strstr(msg, "READY"))
				{
					rsp_msg.result_flag = 1;
					HARDWARE_TEST("\nfind str_READY result: %d\n\n", rsp_msg.result_flag);			
				 }
				else if(strstr(msg, "ERROR")){
					rsp_msg.result_flag = 0;
					HARDWARE_TEST("\nfind str_READY result: %d\n\n", rsp_msg.result_flag);
				}else{
					rsp_msg.result_flag = 0;
					HARDWARE_TEST("\ncant find valid_str\n\n");
				}
			}
			else
			{
				rsp_msg.result_flag = 0;
				HARDWARE_TEST("\nresult: %d, read %s fail\n", 
						rsp_msg.result_flag, name_ttyUSBnum);
			}

            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_RSU_SIM_STATE_DETECT;
            rsp_msg.u16Length = 1;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_RSU_SIM_STATE_DETECT, fd);

            break;
        }
        /* д�豸SN�Ų�������*/
        case PC_REQ_EQUIP_SN_WR:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_EQUIP_SN_WR)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(sn, param_buf[2], msg_len);

            int open4eqsn, ret;
            open4eqsn=open(EEPROM_FILE,O_RDWR);
            if(open4eqsn<0)
            {
                perror("open error");
            }
            init_i2c_ioctrl(open4eqsn);
            if(msg_len>EEPROM_PAGE_SIZE)
            {
                write_sn(open4eqsn, EEPROM_EQSN_ADDR, EEPROM_PAGE_SIZE, sn);
                write_sn(open4eqsn, (EEPROM_EQSN_ADDR+EEPROM_PAGE_SIZE), (msg_len-EEPROM_PAGE_SIZE), (sn+EEPROM_PAGE_SIZE));
            }
            else
            {
                write_sn(open4eqsn, EEPROM_EQSN_ADDR, msg_len, sn);
            }
            sleep(1);
            read_sn(open4eqsn, EEPROM_EQSN_ADDR, msg_len, sn);
            ret = strcmp(param_buf[2], sn);
            if (0 == ret)
            {
                /* д��Ͷ���һ��*/
                rsp_msg.result_flag = 1;
            }
            else
            {
                rsp_msg.result_flag = 0;
            }
            close(open4eqsn);

            rsp_msg.u16MessageID = UE_RSP_EQUIP_SN_WR;
            rsp_msg.u16Length = 1;
            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d\n",
                rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_EQUIP_SN_WR, fd);

            break;
        }
        /* ���豸SN�Ų�������*/
        case PC_REQ_EQUIP_SN_RD:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_EQUIP_SN_RD)\n", msg_type);

            memset(&rsp_eq_sn, 0, sizeof(STR_EQUIP_SN));
            rsp_eq_sn.u16MessageID = UE_RSP_EQUIP_SN_RD;

			int open4eqsn, ret;
			open4eqsn=open(EEPROM_FILE,O_RDWR);
			if(open4eqsn<0){
				perror("open error");
			}
			init_i2c_ioctrl(open4eqsn);
			read_sn(open4eqsn, EEPROM_EQSN_ADDR, msg_len, rsp_eq_sn.u8EquipSN);
			close(open4eqsn);

			rsp_eq_sn.u16Length = msg_len;
            HARDWARE_TEST("read Equipment SN length: 0x%x SN value: %s\n", rsp_eq_sn.u16Length, rsp_eq_sn.u8EquipSN);
            message_produce_handle((void*)&rsp_eq_sn, sizeof(STR_EQUIP_SN), UE_RSP_EQUIP_SN_RD, fd);

            break;
        }
        /* SN�Ž������������*/
        case PC_REQ_SN_CHECK:
        {
            int check_type;
            char sn_check_eq[33];
            char sn_check_io[33];

            HARDWARE_TEST("req msg ID: %d(PC_REQ_SN_CHECK)\n", msg_type);
            if (cnt < 5)
            {
                HARDWARE_TEST("upper param error!, sn_check need 5 parameters!\n");
                return 0;
            }
            /* init req_msg and rsp_msg */
            memset(&req_check_sn, 0, sizeof(STR_SN_CHECK));
            memset(&rsp_check_sn, 0, sizeof(STR_SN_CHECK_RESULT));

            /* get parameter about sn from uuper */
            ret = asctohex(param_buf[1], &check_type, param_len[1]);
            if (0 == ret)
            {
                HARDWARE_TEST("character string hex change failed!\n");
                return 0;
            }
            HARDWARE_TEST("get check_type: %d\n", check_type);
            ret = asctohex(param_buf[2], &req_check_sn.u16BoardLength, param_len[2]);
            if (0 == ret)
            {
                HARDWARE_TEST("character string hex change failed!\n");
                return 0;
            }
            HARDWARE_TEST("get u16BoardLength: %d\n", req_check_sn.u16BoardLength);
            ret = asctohex(param_buf[3], &req_check_sn.u16EquipLength, param_len[3]);
            if (0 == ret)
            {
                HARDWARE_TEST("character string hex change failed!\n");
                return 0;
            }
            HARDWARE_TEST("get u16EquipLength: %d\n", req_check_sn.u16EquipLength);
            memcpy(req_check_sn.u8BoardSN, param_buf[4], req_check_sn.u16BoardLength);
            memcpy(req_check_sn.u8EquipSN, param_buf[5], req_check_sn.u16EquipLength);
            HARDWARE_TEST("get form uuper sn num:\nio:%s\neq:%s\n", req_check_sn.u8BoardSN, req_check_sn.u8EquipSN);

			/* init for i2c to read eeprom */
            int open4checksn;
            open4checksn=open(EEPROM_FILE,O_RDWR);
            if(open4checksn<0){
                perror("open error");
            }
            init_i2c_ioctrl(open4checksn);

            switch(check_type){
            case 1:
                read_sn(open4checksn, EEPROM_IOSN_ADDR, req_check_sn.u16BoardLength, sn_check_io);
                if (0 == strncmp(req_check_sn.u8BoardSN, sn_check_io, req_check_sn.u16BoardLength))
                {
	                rsp_check_sn.u8BoardResult = 1;
                }
	            else
	            {
	                rsp_check_sn.u8BoardResult = 0;
	            }
                break;
            case 2:
				read_sn(open4checksn, EEPROM_EQSN_ADDR, req_check_sn.u16EquipLength, sn_check_eq);
                if (0 == strncmp(req_check_sn.u8EquipSN, sn_check_eq, req_check_sn.u16EquipLength))
                {
	                rsp_check_sn.u8EquipSResult = 1;
	            }
                else
                {
	                rsp_check_sn.u8EquipSResult = 0;
                }
                break;
            case 3:
				read_sn(open4checksn, EEPROM_EQSN_ADDR, req_check_sn.u16EquipLength, sn_check_eq);
				usleep(5000);
				read_sn(open4checksn, EEPROM_IOSN_ADDR, req_check_sn.u16BoardLength, sn_check_io);
		        if (0 == strncmp(req_check_sn.u8BoardSN, sn_check_io, req_check_sn.u16BoardLength))
		        {
		            rsp_check_sn.u8BoardResult = 1;
		        }
	            else
	            {
		            rsp_check_sn.u8BoardResult = 0;
	            }
		        if (0 == strncmp(req_check_sn.u8EquipSN, sn_check_eq, req_check_sn.u16EquipLength))
		        {
		            rsp_check_sn.u8EquipSResult = 1;
		        }
	            else
	            {
		            rsp_check_sn.u8EquipSResult = 0;
	            }
	            break;

			default:
            break;
			}			
			/* sleep ensure eeprom read ok */
			usleep(5000);
			close(open4checksn);
            rsp_check_sn.u16MessageID = UE_RSP_SN_CHECK;
            rsp_check_sn.flag = check_type;
			message_produce_handle((void*)&rsp_check_sn, sizeof(STR_SN_CHECK_RESULT), UE_RSP_SN_CHECK, fd);
            break;
        }
        /* USB HOST��������*/
        case PC_REQ_IO_BOARD_USB_HOST_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_USB_HOST_DETECT)\n", msg_type);

            value = usb_host_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_USB_HOST_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_USB_HOST_DETECT, fd);

            break;
        }
        /* ǩ����֤������Ϣ*/
        case PC_REQ_SIGNATURE_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_SIGNATURE_DETECT)\n", msg_type);

            value = signature_check_test();
            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            rsp_msg.u16MessageID = UE_RSP_SIGNATURE_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;

            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_SIGNATURE_DETECT, fd);

            break;
        }
        /* �̼��汾��ѯ������Ϣ*/
        case PC_REQ_VERSION_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_VERSION_DETECT)\n", msg_type);

            memset(&rsp_firm_msg, 0, sizeof(STR_VERSION_CHECK_RESULT));

            msg_len = module_firmware_ver_test(rsp_firm_msg.u8Version);

            rsp_firm_msg.u16MessageID = UE_RSP_VERSION_CHECK;
            rsp_firm_msg.u16Length = msg_len;
            message_produce_handle((void*)&rsp_firm_msg, sizeof(STR_VERSION_CHECK_RESULT), UE_RSP_VERSION_CHECK, fd);

            break;
        }
        /* �����ȡEMMC��CID����*/
        case PC_REQ_MODULE_CID_NUMBER:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_MODULE_CID_NUMBER)\n", msg_type);

            memset(&rsp_cid_msg, 0, sizeof(STR_CID_CHECK_RESULT));
            value = module_cid_check_test(rsp_cid_msg.u8CIDResult);
            rsp_cid_msg.u16MessageID = UE_RSP_MODULE_CID_CHECK;
            rsp_cid_msg.u16Length = value;

            message_produce_handle((void*)&rsp_cid_msg, sizeof(STR_CID_CHECK_RESULT), UE_RSP_MODULE_CID_CHECK, fd);

            break;
        }
        /* SPI2������ͨ�Բ��ԣ�SPI2��HSM�ӿ�*/
		/* RSU3110 used spi0 for hsm, V4004/5 used spi1 for hsm */
        case PC_REQ_IO_BOARD_HSM_DETECT:
        {
            HARDWARE_TEST("req msg ID: %d(PC_REQ_IO_BOARD_HSM_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
			int io_version = 0;
			io_version = io_board_hardware_ver_test();
			HARDWARE_TEST("io version ID: %d ctao (PC_REQ_IO_BOARD_HSM_DETECT)\n", io_version);
			if(DTVL3110 == io_version){
                gpio_init(156,GPIO_OUT, GPIO_HIGN);
                gpio_init(155,GPIO_IN, GPIO_HIGN);
				gpio_init(219,GPIO_OUT, GPIO_HIGN);
                value = spi_hsm_detect(0);
			}else if((VU400X == io_version)||(DTVL3110E == io_version)){
                gpio_init(209,GPIO_OUT, GPIO_HIGN);
                gpio_init(215,GPIO_IN, GPIO_HIGN);
				gpio_init(31,GPIO_OUT, GPIO_HIGN);
                value = spi_hsm_detect(1);
			}else{
				HARDWARE_TEST("not support temporary\n");
			}
            rsp_msg.u16MessageID = UE_RSP_IO_BOARD_HSM_DETECT;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = value;

            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_IO_BOARD_HSM_DETECT, fd);

            break;
        }
        case PC_REQ_HDWARE_VER_RD:
        {
            int hardware_io_version = 0;
            int hardware_module_version = 0;
            int hardware_subboard1_version = 0;
            int hardware_subboard2_version = 0;
			unsigned short hardware_type = 0;
            HARDWARE_TEST("req msg ID: %d(PC_REQ_HDWARE_VER_RD)\n", msg_type);

			ret = asctohex(param_buf[2], &hardware_type, param_len[2]);
			HARDWARE_TEST("get hardware_type %d from minjie.fan!\n", hardware_type);
			if (0 == ret)
			{
				HARDWARE_TEST("character string hex change failed!\n");
				return 0;
			}
			memset(&rsp_hw_ver, 0, sizeof(STR_HDWARE_VER_RD_RESULT));
			rsp_hw_ver.u16MessageID = UE_RSP_HDWARE_VER_RD;
			rsp_hw_ver.u16Length = 1;

			switch (hardware_type)
			{
				case UU_HARDWARE_VERSION:
				{
					/* We used fixed values name_ttyUSB2 before, 
			  		  * Now use dynamic lookup. Prevent ttyUSB interface disappearance.
			  		  */
					char *name_ttyUSB2 = "/dev/ttyUSB2";
					int ttyUSB2_value = 0;
					char *name_ttyUSBbase = "/dev/ttyUSB"; /* need to merge with ttyUSB num */
					int ttyUSB_num = 0; /* find ttyUSB num used ls /dev/ttyUSB* */
					char *name_ttyUSBnum; /* final name of ttyUSB */
					char msg[200] = {0};
					char *cmd = "ATI\n";
					int num = 0;
					int sec_num = 0;
					int nloop;
					int uu_ver;
					char *revbuf[8] = {0};
					char *sec_revbuf[50] = {0};

					name_ttyUSBnum = malloc(sizeof(name_ttyUSBbase)+1);
					ttyUSB_num = split_ttyUSB_num();
					HARDWARE_TEST("get ttyUSB num %d\n", ttyUSB_num);
					sprintf(name_ttyUSBnum, "%s%d", name_ttyUSBbase, ttyUSB_num);

					/* closed serial echo */
					system("stty -F /dev/ttyUSB2 -echo");
					ttyUSB2_value = simcard_state_test(name_ttyUSBnum, cmd, msg);
					if((1==ttyUSB2_value)&&(strstr(msg, "Revision")!=NULL))
					{
						/*find str from msg_buf;*/
						split_str(msg, "\n", revbuf, &num); 
						for(i = 0;i < num; i ++) {
							if(strstr(revbuf[i], "Revision") != NULL)
							{
								uu_ver = i;
								HARDWARE_TEST("find %s in num %d\n", revbuf[i], i);
								break;
							}
						}

						split_str(revbuf[uu_ver], " ", sec_revbuf, &sec_num); 
						for(i = 0;i < sec_num; i ++) {
							if(strstr(sec_revbuf[i], "Revision") != NULL)
							{
								uu_ver = i+1;
								HARDWARE_TEST("find Revision %s\n", sec_revbuf[uu_ver]);
							}
						}
						HARDWARE_TEST("find Revision value %d %s\n\n", uu_ver, sec_revbuf[uu_ver]);
						
						memcpy((void*)(rsp_hw_ver.u8HardWareVersionResult),
                        	(void*)(sec_revbuf[uu_ver]),
                        	strlen(sec_revbuf[uu_ver]));
					}
					else
					{
						HARDWARE_TEST("\nat recv seccess but find Revision fail\n");
					}

                    rsp_hw_ver.HardWareType = UU_HARDWARE_VERSION;
					HARDWARE_TEST("get uu_ver: %s\n", rsp_hw_ver.u8HardWareVersionResult);
					break;
				}
				case DMD3A_HARDWARE_VERSION:
				{
                    msg_len = module_firmware_ver_test(rsp_hw_ver.u8HardWareVersionResult);
                    rsp_hw_ver.u16Length = msg_len;
                    rsp_hw_ver.HardWareType = DMD3A_HARDWARE_VERSION;
					HARDWARE_TEST("get dmd3a harware: %s\n", rsp_hw_ver.u8HardWareVersionResult);
                    break;
				}
				case MODULE_HARDWARE_VERSION:
				{
                    hardware_module_version = module_hardware_ver_test();
                    memcpy((void*)(rsp_hw_ver.u8HardWareVersionResult),
                        (void*)(module_board_ver_home[hardware_module_version]), 
                        strlen(module_board_ver_home[hardware_module_version]));
                    rsp_hw_ver.HardWareType = MODULE_HARDWARE_VERSION;
					HARDWARE_TEST("get mudlue: %s\n", rsp_hw_ver.u8HardWareVersionResult);
                    break;
				}
				case IO_HARDWARE_VERSION:
				{
                    hardware_io_version = io_board_hardware_ver_test();
                    memcpy((void*)(rsp_hw_ver.u8HardWareVersionResult),
                        (void*)(io_board_ver_home[hardware_io_version]),
                        strlen(io_board_ver_home[hardware_io_version]));
                    rsp_hw_ver.HardWareType = IO_HARDWARE_VERSION;
					HARDWARE_TEST("get io: %s\n", rsp_hw_ver.u8HardWareVersionResult);
                    break;
				}
				case SUBBOARD1_HARDWARE_VERSION:
				{
                    hardware_subboard1_version = child_board_hardware_ver_test(1);
                    if(SUBBOARD1_PKB_V1 == hardware_subboard1_version)
                    {
	                    memcpy((void*)(rsp_hw_ver.u8HardWareVersionResult),
	                        (void*)(sub_board1_ver_home[SUBBOARD1_PKB_V1]),
	                        strlen(sub_board1_ver_home[SUBBOARD1_PKB_V1]));
                        rsp_hw_ver.u16Length = strlen(sub_board1_ver_home[0]);
                    }
                    HARDWARE_TEST("get sub1: %s\n", rsp_hw_ver.u8HardWareVersionResult);
                    rsp_hw_ver.HardWareType = SUBBOARD1_HARDWARE_VERSION;
                    break;
				}
				case SUBBOARD2_HARDWARE_VERSION:
				{
                    hardware_subboard2_version = child_board_hardware_ver_test(2);
                    if(SUBBOARD2_SUWB_V1 == hardware_subboard2_version)
                    {
                        memcpy((void*)(rsp_hw_ver.u8HardWareVersionResult),
                            (void*)(sub_board2_ver_home[SUBBOARD2_SUWB_V1]),
                            strlen(sub_board2_ver_home[SUBBOARD2_SUWB_V1]));
                        rsp_hw_ver.u16Length = strlen(sub_board2_ver_home[0]);
                    }
                    rsp_hw_ver.HardWareType = SUBBOARD2_HARDWARE_VERSION;
					HARDWARE_TEST("get sub2: %s\n", rsp_hw_ver.u8HardWareVersionResult);
                    break;
				}
				default:
				{
					HARDWARE_TEST("req hardware type error!\n");
					return 0;
				}
			}
			message_produce_handle((void*)&rsp_hw_ver, sizeof(STR_HDWARE_VER_RD_RESULT), UE_RSP_HDWARE_VER_RD, fd);
			break;
		}
		case PC_REQ_RESET:
		{
			int test_fd;
			char reset_result[1] = {'1'};

			HARDWARE_TEST("req msg ID: %d(PC_REQ_RESET)\n", msg_type);
			system("rm -rf /var/log/*");
			memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));

			test_fd=open(EEPROM_FILE,O_RDWR);
			if(test_fd<0){
				perror("open error");
			}
			init_i2c_ioctrl(test_fd);
			HARDWARE_TEST("reset result: %s\n", reset_result);
			write_sn(test_fd, EEPROM_TEST_RESULT_ADDR+RESET_SET_STATE, EEPROM_TEST_RESULT_SIZE, reset_result);
			close(test_fd);

            rsp_msg.u16MessageID = UE_RSP_RESET;
            rsp_msg.u16Length = 1;
            rsp_msg.result_flag = 1;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_RESET, fd);
			system("rm -rf /var/log/*");
			break;
		}
		case PC_REQ_TESTSTATIONS_STATE_CHECK:
		{
			int tmp_testcase;
			int test_fd;
			char test_result[6] = {0};

			HARDWARE_TEST("req msg ID: %d(PC_REQ_TESTSTATIONS_STATE_CHECK)\n", msg_type);
			memset(&station_test_result, 0, sizeof(STR_TESTSTATIONS_STATE_CHECK_RESULT));
			station_test_result.u16MessageID = UE_RSP_TESTSTATIONS_STATE_CHECK;

			/* open it in eeprom */
			test_fd=open(EEPROM_FILE,O_RDWR);
			if(test_fd<0){
				perror("open error");
			}
			init_i2c_ioctrl(test_fd);

			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+BOARD_TESTSTATE, EEPROM_TEST_RESULT_SIZE, test_result+BOARD_TESTSTATE);
			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+BEFORE_AGING_TESTSTATE, EEPROM_TEST_RESULT_SIZE, test_result+BEFORE_AGING_TESTSTATE);
			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+STARTING_AGING_TESTSTATE, EEPROM_TEST_RESULT_SIZE, test_result+STARTING_AGING_TESTSTATE);
			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+AFTER_AGING_TESTSTATE, EEPROM_TEST_RESULT_SIZE, test_result+AFTER_AGING_TESTSTATE);
			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+RESET_SET_STATE, EEPROM_TEST_RESULT_SIZE, test_result+RESET_SET_STATE);
			read_sn(test_fd, EEPROM_TEST_RESULT_ADDR+DEVELOP_TESTSTATE, EEPROM_TEST_RESULT_SIZE, test_result+DEVELOP_TESTSTATE);
			close(test_fd);
			HARDWARE_TEST("read in PC_REQ_TESTSTATIONS_STATE_CHECK: %s\n", test_result);

			u8 board_test_state, beforeaging, startaging, after_aging, reset_test, develop_test;
			board_test_state = test_result[BOARD_TESTSTATE]-'0';
			beforeaging = test_result[BEFORE_AGING_TESTSTATE]-'0';
			startaging = test_result[STARTING_AGING_TESTSTATE]-'0';
			after_aging = test_result[AFTER_AGING_TESTSTATE]-'0';
			reset_test = test_result[RESET_SET_STATE]-'0';
			develop_test = test_result[DEVELOP_TESTSTATE]-'0';

			/* this is a garbage code, need to modify it, but time is too hard, sorry */
			if((0==board_test_state) || (1==board_test_state))
			{
				station_test_result.BoardTestState = board_test_state;
				HARDWARE_TEST("\n BOARD_TESTSTATE: %d, BoardTestState: %d\n",
						board_test_state, station_test_result.BoardTestState);
			}
			else{
				station_test_result.BoardTestState = 2;
				HARDWARE_TEST("\n BOARD_TESTSTATE have not test\n");
			}
			if((0==beforeaging) || (1==beforeaging))
			{
				station_test_result.BeforeAgingTestState = beforeaging;
				HARDWARE_TEST("\n BEFORE_AGING_TESTSTATE: %d, BeforeAgingTestState: %d\n",
						beforeaging, station_test_result.BeforeAgingTestState);
			}
			else{
				station_test_result.BeforeAgingTestState = 2;
				HARDWARE_TEST("\n BEFORE_AGING_TESTSTATE have not test\n");
			}
			if((0==startaging) || (1==startaging))
			{
				station_test_result.StartAgingTestState = startaging;
				HARDWARE_TEST("\n STARTING_AGING_TESTSTATE: %d, StartAgingTestState: %d\n",
						startaging, station_test_result.StartAgingTestState);
			}else{
				station_test_result.StartAgingTestState = 2;
				HARDWARE_TEST("\n STARTING_AGING_TESTSTATE have not test\n");
			}
			if((0==after_aging) || (1==after_aging))
			{
				station_test_result.AfterAgingTestState = after_aging;
				HARDWARE_TEST("\n AFTER_AGING_TESTSTATE: %d, AfterAgingTestState: %d\n",
						after_aging, station_test_result.AfterAgingTestState);
			}
			else
			{
				station_test_result.AfterAgingTestState = 2;
				HARDWARE_TEST("\n AFTER_AGING_TESTSTATE have not test\n");
			}
			if((0==reset_test) || (1==reset_test))
			{
				station_test_result.ResetSetState = reset_test;
				HARDWARE_TEST("\n RESET_SET_STATE: %d, ResetSetState: %d\n",
						reset_test, station_test_result.ResetSetState);
			}
			else
			{
				station_test_result.ResetSetState = 2;
				HARDWARE_TEST("\n RESET_SET_STATE have not test\n");
			}
			if((0==develop_test) || (1==develop_test))
			{
				station_test_result.DevelopTestState = develop_test;
				HARDWARE_TEST("\n DEVELOP_TESTSTATE: %d, DevelopTestState: %d\n",
						develop_test, station_test_result.DevelopTestState);
			}
			else
			{
				station_test_result.DevelopTestState = 2;
				HARDWARE_TEST("\n DEVELOP_TESTSTATE have not test\n");
			}

			message_produce_handle((void*)&station_test_result, sizeof(STR_TESTSTATIONS_STATE_CHECK_RESULT), UE_RSP_TESTSTATIONS_STATE_CHECK, fd);
			break;
		}
		case PC_REQ_TESTSTATIONS_STATE_WR:
		{
			unsigned short teststation_type = 0;
			char teststation_result[1];
			char read_eeprom[6] = {0};
			int write_test_result, ret;

			HARDWARE_TEST("req msg ID: %d(PC_REQ_TESTSTATIONS_STATE_WR)\n", msg_type);

			memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
			/* get teststation_type and teststation_result from upper, and write it in eeprom */
			ret = asctohex(param_buf[2], &teststation_type, param_len[2]);
			HARDWARE_TEST("get teststation_type %d from minjie.fan!\n", teststation_type);
			if (0 == ret)
			{
				HARDWARE_TEST("line %d: character string hex change failed!\n", __LINE__);
				return 0;
			}
			//memcpy(teststation_result, param_buf[3], 1);
			HARDWARE_TEST("get teststation_result %s from minjie.fan!\n", param_buf[3]);

			/* write it in eeprom */
			write_test_result=open(EEPROM_FILE,O_RDWR);
			if(write_test_result<0){
				perror("open error");
			}
			init_i2c_ioctrl(write_test_result);

			/*
				BOARD_TESTSTATE           0
				BEFORE_AGING_TESTSTATE    1
				STARTING_AGING_TESTSTATE  2
				AFTER_AGING_TESTSTATE     3
				RESET_SET_STATE           4
				DEVELOP_TESTSTATE         5
			*/
			write_sn(write_test_result, EEPROM_TEST_RESULT_ADDR+teststation_type, EEPROM_TEST_RESULT_SIZE, param_buf[3]);
			sleep(1);
			read_sn(write_test_result, EEPROM_TEST_RESULT_ADDR+teststation_type, EEPROM_TEST_RESULT_SIZE, read_eeprom+teststation_type);
			HARDWARE_TEST("write result %s, read result %s\n", param_buf[3], read_eeprom+teststation_type);

			/* this is ensure write success */
            if((read_eeprom[teststation_type] == param_buf[3][0]))
            {
                rsp_msg.result_flag = 1;
            }
            else
            {
                rsp_msg.result_flag = 0;
            }
			HARDWARE_TEST("checkout result_flag %d\n", rsp_msg.result_flag);
			close(write_test_result);

            rsp_msg.u16MessageID = UE_RSP_TESTSTATIONS_STATE_WR;
            rsp_msg.u16Length = 1;
            message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), UE_RSP_TESTSTATIONS_STATE_WR, fd);

            break;
        }
		case PC_REQ_TESTER_STATE_DETECT:
		{
			HARDWARE_TEST("req msg ID: %d(PC_REQ_TESTER_STATE_DETECT)\n", msg_type);
			char buf[6];
			int fd_process;
			int test_fd;
			char result_sucecss[1] = {'1'};
			char result_fail[1] = {'0'};

			/*check tester state and save it*/
			system("ps -ef | grep tester | grep -v grep | awk '{print $1}' > process.pid");
			fd_process = fopen("process.pid", "r");
			fread(buf, 1, 6, fd_process);
			HARDWARE_TEST("xds_pid: %d\n", atoi(buf));

			/* init response msg */
			memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
			rsp_msg.u16MessageID = PC_RSP_TESTER_STATE_DETECT;
			rsp_msg.u16Length = 1;

			/* record result to eeprom */
			test_fd=open(EEPROM_FILE,O_RDWR);
			if(test_fd<0)
			{
				perror("open error");
			}
			init_i2c_ioctrl(test_fd);

			/* fill response msg */
			if( (atoi(buf)) > 0 )
			{
				write_sn(test_fd, EEPROM_TEST_RESULT_ADDR+STARTING_AGING_TESTSTATE, EEPROM_TEST_RESULT_SIZE, result_sucecss);
				close(test_fd);
				HARDWARE_TEST("PC_REQ_TESTER_STATE_DETECT sucecss\n");
				rsp_msg.result_flag = 1;
			}
			else
			{
				write_sn(test_fd, EEPROM_TEST_RESULT_ADDR+STARTING_AGING_TESTSTATE, EEPROM_TEST_RESULT_SIZE, result_fail);
				close(test_fd);
				HARDWARE_TEST("PC_REQ_TESTER_STATE_DETECT fail\n");
				rsp_msg.result_flag = 0;
			}
			message_produce_handle((void*)&rsp_msg, sizeof(STR_RSP_MESSAGE), PC_RSP_TESTER_STATE_DETECT, fd);
			break;
		}
        default:
        {
            HARDWARE_TEST("req msg type error!\n");
            return 0;
        }
    }
#if 0
    for (i=2; i<(cnt+1); i++)
    {
        ret = asctohex(param_buf[i], &value, param_len[i]);
        if (0 == ret)
        {
            HARDWARE_TEST("character string hex change failed!\n");
            return 0;
        }
        HARDWARE_TEST("recv upper computer messeage: %s\n", param_buf[i]);
        HARDWARE_TEST("changed upper computer messeage: %d(0x%x)\n", value, value);
    }
#endif

    return ret;
}

/************************************************************************
* ������: message_produce_handle 
* ��������: ��λ���·�����Ϣ��������
* �������: void *up_msg: ���ظ���λ����Ϣ
*                           int length: ���ظ���Ϣ����
*                           int type: ���ظ���λ����Ϣ����
*                           int fd: ���ظ���Ϣ�Ŀͻ��˾��
* �������: 0: success
*                           -1: fail
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int message_produce_handle(void *up_msg, int length, int type, int fd)
{
    int ret = 0;
    int i = 0;
    unsigned short msg_type = 0;
    unsigned short msg_len = 0;
    STR_IO_BOARD_SN rsp_bd_sn;
	STR_EQUIP_SN_Add rsp_oem_id;
    STR_EQUIP_SN rsp_eq_sn;
    STR_SN_CHECK_RESULT rsp_check_sn;
    STR_HDWARE_VER_DETECT hw_ver;
    STR_GNSS_GPGSV_CN0 rsp_gnss_cn0;
    STR_RSP_MESSAGE rsp_msg;
    STR_VERSION_CHECK_RESULT rsp_firm_msg;
    STR_CID_CHECK_RESULT rsp_cid_msg;
	STR_HDWARE_VER_RD_RESULT rsp_hw_ver;
    char rsp_buf[BUFF_SIZE] = {0};
    char hw_buf[20] = {0};

    msg_type = type;
    switch (msg_type)
    {
        /* PC��λ����������*/
        case UE_RSP_PC_MESSAGE:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_PC_MESSAGE)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("production line test.\n");
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* дIO�忨SN�Ų�������*/
        case UE_RSP_IO_BOARD_SN_WR:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_SN_WR)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));
#if 0
            write(fd, (char*)&rsp_msg, sizeof(STR_RSP_MESSAGE));
#endif

            break;
        }
		/* дIO�忨SN�Ų�������, add for bit 64-95*/
        case UE_RSP_EQUIP_ADD_SN_WR_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(PC_REQ_EQUIP_ADD_SN_WR_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));
#if 0
            write(fd, (char*)&rsp_msg, sizeof(STR_RSP_MESSAGE));
#endif

            break;
        }
        /* ��IO�忨SN�Ų�������*/
        case UE_RSP_IO_BOARD_SN_RD:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_SN_RD)\n", msg_type);

            memset(&rsp_bd_sn, 0, sizeof(STR_IO_BOARD_SN));
            memcpy(&rsp_bd_sn, up_msg, sizeof(STR_IO_BOARD_SN));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%s", rsp_bd_sn.u16MessageID, rsp_bd_sn.u16Length, rsp_bd_sn.u8BoardSN);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
		/* ��IO�忨SN�Ų������� ��λ���ظ�OEM��Ϣ, add for bit 64-95*/
        case UE_RSP_EQUIP_OEM_RD:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_SN_RD)\n", msg_type);

            memset(&rsp_oem_id, 0, sizeof(STR_EQUIP_SN_Add));
            memcpy(&rsp_oem_id, up_msg, sizeof(STR_EQUIP_SN_Add));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%s", rsp_oem_id.u16MessageID, rsp_oem_id.u16Length, rsp_oem_id.u8EquipSN);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* EMMC����������Ϣ*/
        case UE_RSP_MODULE_EMMC_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_MODULE_EMMC_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* ģ��Ӳ���汾�Ų�������*/
        case UE_RSP_MODULE_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_MODULE_HDWARE_VER_DETECT)\n", msg_type);

            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            memset(hw_buf, 0, sizeof(hw_buf));
            memcpy(&hw_ver, up_msg, sizeof(STR_HDWARE_VER_DETECT));

            memset(rsp_buf, 0, BUFF_SIZE);
            switch(hw_ver.u8ModType)
            {
                case LEADCORE_V1:
                {
                    strcpy(hw_buf, "LEADCORE_V1");
                    break;
                }
                case LEADCORE_V2:
                {
                    strcpy(hw_buf, "LEADCORE_V2");
                    break;
                }
                case LEADCORE_V3:
                {
                    strcpy(hw_buf, "LEADCORE_V3");
                    break;
                }
                case TWMITRASTAR_V1:
                {
                    strcpy(hw_buf, "TWMITRASTAR_V1");
                    break;
                }
                case DTLINKTESTER_V1:
                {
                    strcpy(hw_buf, "DTLINKTESTER_V1");
                    break;
                }
                case DTLINKTESTER_V2:
                {
                    strcpy(hw_buf, "DTLINKTESTER_V2");
                    break;
                }
                case SAMSUNGKDDI_V1:
                {
                    strcpy(hw_buf, "SAMSUNGKDDI_V1");
                    break;
                }
                case DMD3A_JPALPS_V1:
                {
                    strcpy(hw_buf, "DMD3A_JPALPS_V1");
                    break;
                }
				case SAMSUNGKDDI_V2:
                {
                    strcpy(hw_buf, "SAMSUNGKDDI_V2");
                    break;
                }
                default:
                {
                    strcpy(hw_buf, "UNKNOWN");
                    break;
                }
            }
            sprintf(rsp_buf, "%x,%x,%s", hw_ver.u16MessageID, hw_ver.u16Length, hw_buf);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SDIO2�ӿڲ�������*/
        case UE_RSP_MODULE_SDIO2_WIFI_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_MODULE_SDIO2_WIFI_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* UART2��������*/
        case UE_RSP_MODULE_UART2_BT_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_MODULE_UART2_BT_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* IO�忨Ӳ���汾�Ų�������*/
        case UE_RSP_IO_BOARD_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_HDWARE_VER_DETECT)\n", msg_type);

            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            memset(hw_buf, 0, sizeof(hw_buf));
            memcpy(&hw_ver, up_msg, sizeof(STR_HDWARE_VER_DETECT));

            memset(rsp_buf, 0, BUFF_SIZE);
            switch(hw_ver.u8ModType)
            {
                case DTVL3100:
                {
                    strcpy(hw_buf, "DTVL3100");
                    break;
                }
                case DTVL3000:
                {
                    strcpy(hw_buf, "DTVL3000");
                    break;
                }
                case DTVL3100_VBOX:
                {
                    strcpy(hw_buf, "DTVL3100_VBOX");
                    break;
                }
                case DTVL3110:
                {
                    strcpy(hw_buf, "DTVL3110");
                    break;
                }
                case UMCC1_EVB:
                {
                    strcpy(hw_buf, "UMCC1_EVB");
                    break;
                }
                case SUZUNE_EVK_ES1:
                {
                    strcpy(hw_buf, "SUZUNE_EVK_ES1");
                    break;
                }
                case VU300X:
                {
                    strcpy(hw_buf, "VU300X");
                    break;
                }
				case NR_MASTER:
                {
                    strcpy(hw_buf, "NR_MASTER");
                    break;
                }
				case NR_SLAVE:
                {
                    strcpy(hw_buf, "NR_SLAVE");
                    break;
                }
				case VU3205:
                {
                    strcpy(hw_buf, "VU3205");
                    break;
                }
				case DTVL3100_RSU:
                {
                    strcpy(hw_buf, "DTVL3100_RSU");
                    break;
                }
				case VU400X:
                {
                    strcpy(hw_buf, "VU400X");
                    break;
                }
				case SUZUNE3:
                {
                    strcpy(hw_buf, "SUZUNE3");
                    break;
                }				
                default:
                {
                    strcpy(hw_buf, "UNKNOWN");
                    break;
                }
            }
            sprintf(rsp_buf, "%x,%x,%s", hw_ver.u16MessageID, hw_ver.u16Length, hw_buf);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        case UE_RSP_SUBBOARD1_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_SUBBOARD1_HDWARE_VER_DETECT)\n", msg_type);

            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            memset(hw_buf, 0, sizeof(hw_buf));
            memcpy(&hw_ver, up_msg, sizeof(STR_HDWARE_VER_DETECT));
            memset(rsp_buf, 0, BUFF_SIZE);

			switch(hw_ver.u8ModType)
			{
				case SUBBOARD1_PKB_V1:
				{
					strcpy(hw_buf, "PKB_V1");
					break;
				}
				default:
                {
                    strcpy(hw_buf, "UNKNOWN");
                    break;
                }
			}
            sprintf(rsp_buf, "%x,%x,%s", hw_ver.u16MessageID, hw_ver.u16Length, hw_buf);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;

        }
        case UE_RSP_SUBBOARD2_HDWARE_VER_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_SUBBOARD2_HDWARE_VER_DETECT)\n", msg_type);

            memset(&hw_ver, 0, sizeof(STR_HDWARE_VER_DETECT));
            memset(hw_buf, 0, sizeof(hw_buf));
            memcpy(&hw_ver, up_msg, sizeof(STR_HDWARE_VER_DETECT));
            memset(rsp_buf, 0, BUFF_SIZE);
			HARDWARE_TEST("rsp %d\n", hw_ver.u8ModType);

			switch(hw_ver.u8ModType)
			{
				case SUBBOARD2_SUWB_V1:
				{
					strcpy(hw_buf, "SUWB_V1");
					break;
				}
				default:
                {
                    strcpy(hw_buf, "UNKNOWN");
                    break;
                }
			}
            sprintf(rsp_buf, "%x,%x,%s", hw_ver.u16MessageID, hw_ver.u16Length, hw_buf);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));
            break;
        }
        /* UART1��������*/
        case UE_RSP_IO_BOARD_UART1_GNSS_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_UART1_GNSS_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* PP1S��������*/
        case UE_RSP_IO_BOARD_PP1S_GNSS_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_PP1S_GNSS_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* GPS�����CN0ֵ��ѯ����*/
        case UE_RSP_IO_BOARD_CN0_GNSS_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_CN0_GNSS_TEST)\n", msg_type);

            memset(&rsp_gnss_cn0, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_gnss_cn0, up_msg, sizeof(STR_GNSS_GPGSV_CN0));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", 
                                                            rsp_gnss_cn0.u16MessageID, rsp_gnss_cn0.u16Length,
                                                            rsp_gnss_cn0.u8CN0[0], rsp_gnss_cn0.u8CN0[1], rsp_gnss_cn0.u8CN0[2], rsp_gnss_cn0.u8CN0[3], 
                                                            rsp_gnss_cn0.u8CN0[4], rsp_gnss_cn0.u8CN0[5], rsp_gnss_cn0.u8CN0[6], rsp_gnss_cn0.u8CN0[7], 
                                                            rsp_gnss_cn0.u8CN0[8], rsp_gnss_cn0.u8CN0[9], rsp_gnss_cn0.u8CN0[10], rsp_gnss_cn0.u8CN0[11], 
                                                            rsp_gnss_cn0.u8CN0[12], rsp_gnss_cn0.u8CN0[13], rsp_gnss_cn0.u8CN0[14], rsp_gnss_cn0.u8CN0[15], 
                                                            rsp_gnss_cn0.u8CN0[16], rsp_gnss_cn0.u8CN0[17], rsp_gnss_cn0.u8CN0[18], rsp_gnss_cn0.u8CN0[19]);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* COMUART��������*/
        case UE_RSP_IO_BOARD_COMUART_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_COMUART_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* USB OTG��������*/
        case UE_RSP_IO_BOARD_USB_OTG_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_USB_OTG_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* ETH���ڲ�������*/
        case UE_RSP_IO_BOARD_ETH_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_ETH_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* HSIC�ӿڲ�������*/
        case UE_RSP_IO_BOARD_USB_HSIC_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_USB_HSIC_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* LAN9500��������*/
        case UE_RSP_IO_BOARD_LAN9500_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_LAN9500_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* UART0��������*/
        case UE_RSP_IO_BOARD_UART0_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_UART0_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* CAN�ӿڲ�������*/
        case UE_RSP_IO_BOARD_CAN_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_CAN_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SPI0�ӿڲ�������*/
        case UE_RSP_IO_BOARD_SPI0_CAN_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_SPI0_CAN_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* I2C1�ӿڲ�������*/
        case UE_RSP_IO_BOARD_I2C1_SENSOR_TEST:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_I2C1_SENSOR_TEST)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* WIFI���ܲ�������*/
        case UE_RSP_IO_BOARD_AP_WIFI_CONNECT_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_AP_WIFI_CONNECT_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SIM��������������*/
        case UE_RSP_IO_BOARD_RSU_NETWORK_ONLINE_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_RSU_NETWORK_ONLINE_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SIM card state test*/
        case UE_RSP_IO_BOARD_RSU_SIM_STATE_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_RSU_SIM_STATE_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }

        /* д�豸SN�Ų�������*/
        case UE_RSP_EQUIP_SN_WR:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_EQUIP_SN_WR)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* ���豸SN�Ų�������*/
        case UE_RSP_EQUIP_SN_RD:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_EQUIP_SN_RD)\n", msg_type);

            memset(&rsp_eq_sn, 0, sizeof(STR_EQUIP_SN));
            memcpy(&rsp_eq_sn, up_msg, sizeof(STR_EQUIP_SN));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%s", rsp_eq_sn.u16MessageID, rsp_eq_sn.u16Length, rsp_eq_sn.u8EquipSN);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SN�Ž������������*/
        case UE_RSP_SN_CHECK:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_SN_CHECK)\n", msg_type);

            memset(&rsp_check_sn, 0, sizeof(STR_SN_CHECK_RESULT));
            memcpy(&rsp_check_sn, up_msg, sizeof(STR_SN_CHECK_RESULT));

            HARDWARE_TEST("rsp msg u16MessageID: %d u8flag: %d IO Board check result: %d equipment check result: %d\n", rsp_check_sn.u16MessageID, rsp_check_sn.flag, rsp_check_sn.u8BoardResult, rsp_check_sn.u8EquipSResult);
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x,%x", rsp_check_sn.u16MessageID, rsp_check_sn.flag, rsp_check_sn.u8BoardResult, rsp_check_sn.u8EquipSResult);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* USB HOST��������*/
        case UE_RSP_IO_BOARD_USB_HOST_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_USB_HOST_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* ǩ����֤���Իظ���Ϣ*/
        case UE_RSP_SIGNATURE_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_SIGNATURE_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* �̼��汾��ѯ�ظ���Ϣ*/
        case UE_RSP_VERSION_CHECK:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_VERSION_CHECK)\n", msg_type);

            memset(&rsp_firm_msg, 0, sizeof(STR_VERSION_CHECK_RESULT));
            memcpy(&rsp_firm_msg, up_msg, sizeof(STR_VERSION_CHECK_RESULT));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%s", rsp_firm_msg.u16MessageID, rsp_firm_msg.u16Length, rsp_firm_msg.u8Version);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* EMMC CID�Ų�ѯ�ظ���Ϣ*/
        case UE_RSP_MODULE_CID_CHECK:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_MODULE_CID_CHECK)\n", msg_type);

            memset(&rsp_cid_msg, 0, sizeof(STR_CID_CHECK_RESULT));
            memcpy(&rsp_cid_msg, up_msg, sizeof(STR_CID_CHECK_RESULT));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%s", rsp_cid_msg.u16MessageID, rsp_cid_msg.u16Length, rsp_cid_msg.u8CIDResult);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        /* SPI2������ͨ�Բ��ԣ�SPI2��HSM�ӿ�*/
        case UE_RSP_IO_BOARD_HSM_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_IO_BOARD_HSM_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
		/* test for read hardware version when device up */
        case UE_RSP_HDWARE_VER_RD:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_HDWARE_VER_RD)\n", msg_type);

            memset(&rsp_hw_ver, 0, sizeof(STR_HDWARE_VER_RD_RESULT));
            memcpy(&rsp_hw_ver, up_msg, sizeof(STR_HDWARE_VER_RD_RESULT));

            HARDWARE_TEST("hw_version: MessageID: %d, HardWareType %d, Length: %d, hw_version: %s sizeof(STR_RSP_MESSAGE): %d\n",
				rsp_hw_ver.u16MessageID, rsp_hw_ver.HardWareType, rsp_hw_ver.u16Length, rsp_hw_ver.u8HardWareVersionResult, sizeof(STR_HDWARE_VER_RD_RESULT));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x,%s",
				rsp_hw_ver.u16MessageID,
				rsp_hw_ver.HardWareType,
				rsp_hw_ver.u16Length,
				rsp_hw_ver.u8HardWareVersionResult
				);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }

		case UE_RSP_RESET:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_HDWARE_VER_RD)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
		case UE_RSP_TESTSTATIONS_STATE_CHECK:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_HDWARE_VER_RD)\n", msg_type);

            STR_TESTSTATIONS_STATE_CHECK_RESULT station_test_result;
            memset(&station_test_result, 0, sizeof(STR_TESTSTATIONS_STATE_CHECK_RESULT));
            memcpy(&station_test_result, up_msg, sizeof(STR_TESTSTATIONS_STATE_CHECK_RESULT));

            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x,%x,%x,%x,%x",
				station_test_result.u16MessageID,
				station_test_result.BoardTestState,
				station_test_result.BeforeAgingTestState,
				station_test_result.StartAgingTestState,
				station_test_result.AfterAgingTestState,
				station_test_result.ResetSetState,
				station_test_result.DevelopTestState
				);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        case UE_RSP_TESTSTATIONS_STATE_WR:
        {
            HARDWARE_TEST("rsp msg ID: %d(UE_RSP_HDWARE_VER_RD)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));

            break;
        }
        case PC_RSP_TESTER_STATE_DETECT:
        {
            HARDWARE_TEST("rsp msg ID: %d(PC_RSP_TESTER_STATE_DETECT)\n", msg_type);

            memset(&rsp_msg, 0, sizeof(STR_RSP_MESSAGE));
            memcpy(&rsp_msg, up_msg, sizeof(STR_RSP_MESSAGE));

            HARDWARE_TEST("rsp msg u16MessageID: %d u16Length: %d result_flag: %d sizeof(STR_RSP_MESSAGE): %d\n", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag, sizeof(STR_RSP_MESSAGE));
            memset(rsp_buf, 0, BUFF_SIZE);
            sprintf(rsp_buf, "%x,%x,%x", rsp_msg.u16MessageID, rsp_msg.u16Length, rsp_msg.result_flag);
            HARDWARE_TEST("rsp_buf: %s\n", rsp_buf);
            write(fd, rsp_buf, strlen(rsp_buf));
            break;
        }
        default:
        {
            HARDWARE_TEST("rsp msg type error!\n");
            return 0;
        }
    }

    return ret;
}

/************************************************************************
* ������: DisplayList 
* ��������: ��ӡ������
* �������: ������ĿTestList
* �������: ��
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
void DisplayList(TEST_ITEM* list)
{
    unsigned int i = 0;

    HARDWARE_TEST("\n\r");
    HARDWARE_TEST("*************************HARDWARE FUNCTION TEST MENU*************************\n\r");
    while(1)
    {
        if(NULL == list[i].func)
        {
            if((i%3) && (i))
            {
                HARDWARE_TEST("\n\r");
            }
            break;
        }
        else
        {
            if(i+1 < 10)
            {
                HARDWARE_TEST(" ");
            }
            HARDWARE_TEST("%d:", i+1);

            if(NULL == list[i].pname)
            {
                HARDWARE_TEST("Test item %d", i+1);
            }
            else
            {
                HARDWARE_TEST("%s", list[i].pname);
            }
            
            i++;
            if(i%3 == 0)
            {
                HARDWARE_TEST("\n\r");
            }
        }
    }
    HARDWARE_TEST("***************************Y(y):confirm, Q(q):quit***************************\n\r");

    return;
}

/************************************************************************
* ������: GetListNum 
* ��������: ����List���ж��ٸ�func������
* �������: ������ĿList����
* �������: ��
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int GetListNum(TEST_ITEM* list)
{
    int cnt = 0;

    while(1)
    {
        if(NULL == list[cnt].func)
        {
            break;
        }
        else
        {
            cnt++;
        }
    }

    return cnt;
}

/************************************************************************
* ������: Char_Judge 
* ��������: �ж�������ַ��Ƿ�Ϊ:0~9��Q(q)��Y(y)
* �������:   ascii:����̨console�����ASCII������ascii
*                             num:��ת����ASCII���ַ���
* �������: 1:��ȷ��������0~9
*                           2:�����ַ�Y(y)��������
*                           3:�����ַ�Q(q)�˳�����ѡ��
*                           0:����̨console�������
* ����ֵ: �� 
* ��ע: 
*
*************************************************************************/
int Char_Judge(char* ascii, int num)
{
    int i = 0;

    /* ����̨console�����ַ�Ϊ���ַ�ʱ��*
      * ����ȡ�����ַ��Ƿ�Ϊ1~9��Q(q)��Y(y)��13���ַ�*/
    if (1 == num)
    {
        /*����1~9����һ�������ַ�����Ϊ0*/
        if((ascii[0] <= 0x39) && (ascii[0] > 0x30))
        {
            return INPUT_OK;
        }
        /*��ĸY��yȷ�ϼ�������*/
        else if ((ascii[0] == 0x59) || (ascii[0] == 0x79))
        {
            HARDWARE_TEST("Key:Y(y).\n\r");
            return INPUT_GO;
        }
        /*��ĸQ��q�˳����Գ���*/
        else if ((ascii[0] == 0x51) || (ascii[0] == 0x71))
        {
            HARDWARE_TEST("Key:Q(q).\n\r");
            return INPUT_QUIT;
        }
        else
        {
            HARDWARE_TEST("Console input error.\n\r");
            return INPUT_ERR;
        }
    }
    /* ����̨console�����ַ����ڵ���2��ʱ��*
      * ��Щ���ڵ���2�����ַ�����Ϊ�����ַ�*/
    else
    {
        /* ���������ַ�Ϊ'0'���ش���*/
        if (ascii[0] == 0x30)
        {
            HARDWARE_TEST("Console input error.\n\r");
            return INPUT_ERR;
        }

        /*����ȡ�����ַ��Ƿ�Ϊ0~9��10���ַ�*/
        for(i=0; i<num; i++)
        {
            /*����0~9*/
            if((ascii[i] <= 0x39) && (ascii[i] >= 0x30))
            {
                if (i < num -1)
                {
                    continue;
                }
                return INPUT_OK;
            }
            else
            {
                HARDWARE_TEST("Console input error.\n\r");
                return INPUT_ERR;
            }
        }
    }

    return INPUT_OK;
}

/************************************************************************
* ������: asctohex
* ��������:  �������ʮ��������Ч�ַ���ת��Ϊint������
*
* �������:None
*
* �������: None
* ����ֵ: None
* ��ע:  
* 
*************************************************************************/
unsigned int asctohex(char* ascii, unsigned int* val, int num)
{
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 1;

    /*�ж�������ַ��Ƿ�Ϊ0-9, A-F, a-f ����Ч�ַ�������ֱ���˳�*/
    for(i=0; i<num; i++)
    {
        if(((ascii[i] >= 0x30) && (ascii[i] <= 0x39))||((ascii[i] >= 0x41) && (ascii[i] <= 0x46))||((ascii[i] >= 0x61) && (ascii[i] <= 0x66)))
        {
            continue;
        }
        else
        {
            return 0;
        }
    }

    /* ���������ַ���������Ч�ַ����򷵻�0*/
    if(i != num)
    {
        return 0;
    }
    else
    {
        /* ������������룬���ASCII������ת��16��������*/
        while(i > 0)
        {
            i--;
            /* �ж��Ƿ�A-F*/
            if((ascii[i]>=0x41)&&(ascii[i]<=0x46))
            {
                j += k*(ascii[i]-55);
                k *= 16;
            }
            /* �ж��Ƿ�a-f*/
            else if((ascii[i]>=0x61)&&(ascii[i]<=0x66))
            {
                j += k*(ascii[i]-87);
                k *= 16;
            }
            /* 0-9*/
            else
            {
                j += k*(ascii[i]-48);
                k *= 16;
            }
        }
        /* �ѻ�ȡ��ʮ���������ݣ�ͨ��ָ��val����*/
        *val = j;

        /* ���ASCII��ת10�������ݺ󷵻�1*/
        return 1;
    }
}

/************************************************************************
* ������: ASCToInt 
* ��������: ����List���ж��ٸ�func������
* �������:   ascii:����̨console�����ASCII������ascii
*                             val:ת�����ʮ������
*                             num:��ת����ASCII���ַ���
* �������: ASCII��ת��10��������val
* ����ֵ: �� 
* ��ע: 
* ASCII��
* 0x30	0
* 0x31	1
* ...
* 0x39	9
* 0x41	A
* ...
* 0x5A	Z
*
*************************************************************************/
unsigned int ASCToInt(char* ascii, int* val, int num)
{
    int i = 0;
    int j = 1;

    /*�ж�������ַ��Ƿ�Ϊ0-9����Ч�ַ�������ֱ���˳�*/
    for(i=0; i<num; i++)
    {
        if((ascii[i] >= 0x30) && (ascii[i] <= 0x39))
        {
            continue;
        }
        else
        {
            return 0;
        }
    }

    /* ��num���ַ�����ASCIIת������Щ�ַ�Ϊ0~9�ַ�*/
    if(num == 0)//���������ַ���Ϊ0���򷵻�0
    {
        return 0;
    }
    else
    {
        while(num > 0)//������ַ����룬���ASCII���ַ�ת��10��������
        {
            num--;
            i += j*(ascii[num] - 0x30);
            j *= 10;
        }
        *val = i;//�ѻ�ȡ��ʮ�������ݣ�ͨ��ָ��val����

        return 1;//���ASCII��ת10�������ݺ󷵻�1
    }
}

/************************************************************************
* ������: Input_Key_Check 
* ��������: �ж϶�ȡ�����ַ��Ƿ�Ϊ:0~9��Q(q)��Y(y)
*
* �������:       p_buf                              p_val                                length
*                           char������        ��ѡ�������           ���鳤��
* �������:
*  
* ����ֵ: �� 
* ��ע: ��׼������������stdin,��׼�����stdout,��׼������stderr��ʾ
*               STDIN_FILENO��ʾ��׼����,��׼������STDOUT_FILENO,��׼������STDERR_FILENO
* stdin��STDIN_FILENO������
*              stdin����FILE *���ͣ����ڱ�׼I/O����<stdio.h>��
*              STDIN_FILENO�����ļ����������ǷǸ�������
*              һ�㶨��Ϊ0, 1, 2������û��buffer��I/O��ֱ�ӵ���ϵͳ���ã���<unistd.h>��
*
*************************************************************************/
int Input_Key_Check(char* p_buf, int* p_val, int length)
{
    int ret = -1;
    int bck = 0;
    int num = 0;
    char buf[MAX_KEY_BYTES] = {0};

    if (NULL == p_val)
    {
        bck = INPUT_ERR;
        HARDWARE_TEST("Input error.\n\r");
        return bck;
    }

    /*��ȡ����̨console������ַ�*/
    if (length >= MAX_KEY_BYTES)
    {
        num = MAX_KEY_BYTES;
    }
    else
    {
        num = length;
    }

    strncpy(buf, p_buf, num);
    /*����ȡ�����ַ��Ƿ�Ϊ0~9��Q(q)��Y(y)��14���ַ�*/
    ret = Char_Judge(buf, num);
    switch (ret)
    {
        case 1:
        {
            /*�������ѡ����ȷ*/
            ASCToInt(buf, p_val, num);
            bck = INPUT_OK;
            if (*p_val > Item_num)
            {
                HARDWARE_TEST("Item choice error.\n\r");
                bck = INPUT_ERR;
            }
            break;
        }
        case 2:
        {
            /*��ĸY��yȷ�ϼ�������*/
            bck = INPUT_GO;
            break;
        }
        case 3:
        {
            /*��ĸQ��q���˳����Գ���*/
            bck = INPUT_QUIT;
            break;
        }
        case 0:/*����̨console��������*/
        default:
        {
            bck = INPUT_ERR;
            break;
        }
    }

    return bck;
}

/************************************************************************
* ������: Acq_Input_Key 
* ��������: �ӱ�׼�����ж��ַ���
* �������:       p_buf                        length
*                           char������        ���鳤��
* �������:
* ����ֵ: -1:����̨�������
*                     ����:����ʵ�ʴӿ���̨console��ȡ�����ַ���
*
* ��ע: ��׼������������stdin,��׼�����stdout,��׼������stderr��ʾ
*               STDIN_FILENO��ʾ��׼����,��׼������STDOUT_FILENO,��׼������STDERR_FILENO
* stdin��STDIN_FILENO������
*              stdin����FILE *���ͣ����ڱ�׼I/O����<stdio.h>��
*              STDIN_FILENO�����ļ����������ǷǸ�������
*              һ�㶨��Ϊ0, 1, 2������û��buffer��I/O��ֱ�ӵ���ϵͳ���ã���<unistd.h>��
*
*************************************************************************/
int Acq_Input_Key(char* p_buf, int length)
{
    int Ret = -1;
    int num = -1;
    fd_set fs_ext;
    int fd_max = 0;
    struct timeval tval;
    char buf[MAX_KEY_BYTES] = {0};

    tval.tv_sec = MAX_TIMEOUT;       /*��ʱ�ȴ����������ַ�*/
    tval.tv_usec = 0;
    FD_ZERO(&fs_ext);
    FD_SET(STDIN_FILENO, &fs_ext);
    fd_max = STDIN_FILENO + 1;

    /*������ȡ����̨������ַ�*/
    while(1)
    {
        Ret = select(fd_max, &fs_ext, NULL, NULL, &tval);
        if (Ret < 0)
        {
            HARDWARE_TEST("select error, Ret %d\n\r", Ret);
            break;
        }
        else if (0 == Ret)
        {
            HARDWARE_TEST("Waitting Input TimeOut!\n\r");
            return -1;
        }
        else
        {
            if (FD_ISSET(STDIN_FILENO, &fs_ext))
            {
                /* ��ȡ��׼�����ַ�����buf�У����ض�ȡ�ֽ���*
                  * Ĭ��һ�����벻����100���ַ���ֻ֧��0~9��Q(q)��Y(y)��14���ַ�*
                  * 0~9��ʾѡ������Q(q)�˳����Գ���Y(y)�������Գ���*
                  * ���ڱ�׼����(����̨console)�Իس�����ʾ���������*
                  * ��˴ӱ�׼����read()��buf������Ϊ�����ַ�+�س�����������ȥ���س���*/
                num = read(STDIN_FILENO, buf, MAX_KEY_BYTES);
                if(num < 0)
                {
                    HARDWARE_TEST("Can't Obtain characters from console!Please input again.\n\r");
                    continue;
                }

                num = num-1;    /*ֻ����0~num-1���ַ����Ӷ�ɾ���س���*/
                if(0 == num)
                {
                    HARDWARE_TEST("You input only Key Enter.Please input again!\n\r");
                    continue;
                }

                if (length >= num)
                {
                    strncpy(p_buf, buf, num);
                    return num;
                }
                else
                {
                    strncpy(p_buf, buf, length);
                    return length;
                }
            }/*end if (FD_ISSET(STDIN_FILENO, &fs_ext))*/
        }
    }/*end while(1)*/

    return -1;
}

/************************************************************************
* ������: Set_Module_Base_Addr
* ��������: ��ȡSSI��GPIO��I2C��AP_PWR��1860ģ��mmapӳ���ĵ�ַ
* �������:  None
*   
* �������: None
* ����ֵ: None
*   
* ��ע: 
*
*************************************************************************/
void Set_Module_Base_Addr(void)
{
    SSI0_MODEULE_BASE = map_module(SPI0);
    SSI1_MODEULE_BASE = map_module(SPI1);
    SSI2_MODEULE_BASE = map_module(SPI2);
    I2C0_MODEULE_BASE = map_module(I2C0);
    I2C1_MODEULE_BASE = map_module(I2C1);
    I2C2_MODEULE_BASE = map_module(I2C2);
    I2C3_MODEULE_BASE = map_module(I2C3);
    COM_I2C_MODEULE_BASE = map_module(COM_I2C);
    MUX_PIN_MODEULE_BASE = map_module(MUX_PIN);
    GPIO_MODEULE_BASE = map_module(GPIO);
    UART0_MODEULE_BASE = map_module(UART0);
    UART1_MODEULE_BASE = map_module(UART1);
    UART2_MODEULE_BASE = map_module(UART2);
    COM_UART_MODEULE_BASE = map_module(COM_UART);
    AP_PWR_MODEULE_BASE = map_module(AP_PWR);
    AP_CTL_MODEULE_BASE = map_module(AP_CTL);
    SDMMC2_MODEULE_BASE = map_module(SDMMC2);
    DDR_PWR_MODEULE_BASE = map_module(DDR_PWR);
}
