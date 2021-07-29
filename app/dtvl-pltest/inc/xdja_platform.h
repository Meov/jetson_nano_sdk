/** 
 * @file 
 * @brief 平台实现文件，包括芯片设备IO操作，平台提供的延时函数等
 * @author xdja
 * @version 1.0.0.1
 * @date 20181023
 */
#ifndef __XDJA_PLATFORM_H_INC__
#define __XDJA_PLATFORM_H_INC__

#define XDJA_DEBUG
#ifdef XDJA_DEBUG
    #define xdja_debug(fmt, args...) printf(fmt, ##args)
#else
    #define xdja_debug(fmt, args...)
#endif

/************************************************************************/
/* 错误代码定义                                                         */
/************************************************************************/
#define SPI_RES_OK                            0x00000000             //成功
#define SPI_RES_WAKEUP_TIMEOUT                -1                     //唤醒芯片超时
#define SPI_RES_IDLE_TIMEOUT                  -2                     //等待芯片空闲超时
#define SPI_RES_RECV_HEAD	                  -3                     //接收数据头错误
#define SPI_RES_RECV_LEN	                  -4                     //接收数据长度错误
#define SPI_RES_RECV_SN						  -5                     //接收数据SN错误
#define SPI_RES_RECV_CRC	                  -6                     //接收数据CRC错误
#define SPI_RES_OPEN_SPI	                  -7                     //打开SPI设备失败
#define SPI_RES_CFG_SPI	                      -8                     //配置SPI设备失败
#define SPI_RES_OVER_TRY	                  -9                     //超出重试次数
#define SPI_RES_CHECK_DEV	                  -10                    //检查设备出错

/**
* @brief 初始化SPI
*
*/
int XDJA_SPI_Init(void);

/**
* @brief SPI读数据
*
* @param[out] r_buf	存储获取的数据的基地址
* @param[in]  len   数据长度
*
* @retval 成功返回0
*/
int XDJA_SPI_Read(int fd, unsigned char* r_buf, int len);

/**
* @brief SPI写数据
*
* @param[in]  w_buf  待写入数据的基地址
* @param[in]  len    数据长度
*
* @retval 成功返回0
*/
int XDJA_SPI_Write(int fd, unsigned char* w_buf, int len);

/**
* @brief 获取INT1引脚状态（从状态）
**
* @retval 高电平返回1,低电平返回0
*/
int XDJA_SPI_INT1(int fd);
/**
* @brief 设置INT0引脚状态（主状态）
*
* @param[in] stat：高电平设置1，低电平设置0
*
* @retval 成功返回0
*/
void XDJA_SPI_INT0(int fd, int stat);

/**
* @brief 芯片复位
*
* @retval 成功返回0
*/
void XDJA_SPI_RESET(int fd);
								
/**
* @brief 毫秒级延时
*
* @param[in] ms  延时毫秒数
*
*/
void XDJA_Delayms(int ms);

/**
* @brief 资源释放接口
*
*
*/

void  XDJA_SPI_FINIAL();

#endif

