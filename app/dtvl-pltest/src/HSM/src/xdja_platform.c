#include "xdja_platform.h"
//platform include files
//#include "stm32f7xx_hal.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "sys/ioctl.h"
#include "unistd.h"
#include "spidev.h"
#include "errno.h"

int int0_handle=-1;
int int1_handle=-1;
static int spi_speed=30;

/* del for compatible spi0 and spi1 hsm test, 20200928 ctao */
#if 0
#define XDJA_INT0_GPIO	156
#define XDJA_INT1_GPIO	155
//#define XDJA_INT0_GPIO	209
//#define XDJA_INT1_GPIO	215
#endif
#define XDJA_POR_GPIO	16
#define XDJA_POWER_GPIO	47

/* IO板卡硬件版本类型宏定义*/
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

//检查Linux GPIO节点是否存在，成功返回0
static int CheckGpio(int n)  
{  
	char path[64];  
	int fd;

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", n); 
	//printf("%s\n",path); 
#if 1
	if(access(path, F_OK) != 0)
	{
		//printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;
	}
#endif
	return 0;  
}

//创建Linux GPIO节点
static int exportGpio(int n)  
{  
	char path[64];  
	int handle;
	char exportStr[6]={0};  
	int ret;

	handle = open("/sys/class/gpio/export",O_WRONLY);
	if(handle == -1)
	{
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;
	} 
	snprintf(exportStr, sizeof(exportStr), "%d", n); 
	//printf("%s %d\n",exportStr,strlen(exportStr));
	ret = write(handle,exportStr,strlen(exportStr));//ret
	close(handle);
	return ret;  
}

//检查并配置GPIO方向
static int gpioDirectioninit(int n,int value)  
{  
	char path[64];  
	char value_str[4]={0}; 
	int handle;  
	int ret=0;
	memset(value_str,0,4);
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", n); 
	//printf("%s\n",path); 
	handle = open(path, O_RDWR);  
	if (handle <= 0) {  
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;  
	}  

	if (read(handle, value_str, 3) < 0) { 
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;  
	} 

	lseek(handle,0,0);
#if 1	
	if(value)
	{
		ret = write(handle,"out",3);
		//printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);	 
	}
	else
	{
		ret = write(handle,"in",3);
		//printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);	 
	}
#endif	

	return 0;  
}

//GPIO初始化
//int value 0 ==in 1==out
static int GpioInit(int n,int value)  
{
	int ret;
	//1.检查是否存在
	ret = CheckGpio(n);
	if(ret == -1)
	{
		//2.不存在则创建
		ret = exportGpio(n);
		if(ret == -1)
			return ret;
	}
	//3.检查并配置GPIO
	ret = gpioDirectioninit(n,value);

	return ret;
	
}
//获取GPIO值
static int getGpioValue(int n) 
{  
	char path[64];  
	char value_str[3]={0};  
	int fd;  
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", n); 
	//printf("%s\n",path); 
	fd = open(path, O_RDONLY);  
	if (fd <= 0) 
	{  
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;  
	}  

	if (read(fd, value_str, 1) < 0) 
	{  
		printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);	  
		return -1;  
	}  
	close(fd);  
	if(value_str[0] == '0')
		return 0;
	else if(value_str[0] == '1')
		return 1;
	else
		return -1;
 }

//SPI初始化
int XDJA_SPI_Init()
{
    int handle;
    int ret;
    char path[64]={0};

	int io_version = 0;
	io_version = io_board_hardware_ver_test();
	printf("io version ID: %d\n", io_version);
	if((VU3205==io_version)||(DTVL3110==io_version))
	{
		int XDJA_INT0_GPIO = 156;
		int XDJA_INT1_GPIO = 155;
		ret = GpioInit(XDJA_INT0_GPIO,1); //int 0 out
	    usleep(1);
	    ret = GpioInit(XDJA_INT1_GPIO,0); //int 1 in
	    usleep(1);
		//2.预打开GPIO节点
	    if(int0_handle == -1){
			snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT0_GPIO);
	        int0_handle = open(path,O_RDWR);
	        if (int0_handle < 0)
			{
	        	printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
	            return -1;
	        }
	    }
	    usleep(1);
	    if(int1_handle ==-1){
			snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT1_GPIO);
	        int1_handle = open(path,O_RDWR);
	        if(int1_handle < 0){
	            printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
	            return -1;
	        }
	    }
	    usleep(1);		
	}else if(VU400X == io_version){
		int XDJA_INT0_GPIO = 209;
		int XDJA_INT1_GPIO = 215;
		ret = GpioInit(XDJA_INT0_GPIO,1); //int 0 out
	    usleep(1);
	    ret = GpioInit(XDJA_INT1_GPIO,0); //int 1 in
	    usleep(1);
		//2.预打开GPIO节点
	    if(int0_handle == -1){
			snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT0_GPIO);
	        int0_handle = open(path,O_RDWR);
	        if (int0_handle < 0){
	        	printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
	            return -1;
	        }
	    }
	    usleep(1);
	    if(int1_handle ==-1){
			snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT1_GPIO);
	        int1_handle = open(path,O_RDWR);
	        if(int1_handle < 0){
	            printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
	            return -1;
	        }
	    }
	    usleep(1);
	}
	/* del for compatible spi0 and spi1 hsm test, 20200928 ctao */
	#if 0
	//1.初始化GPIO （失败是否判断？设置默认方向，是否设置默认值 ？）
    ret = GpioInit(XDJA_INT0_GPIO,1); //int 0 out
    usleep(1);
    ret = GpioInit(XDJA_INT1_GPIO,0); //int 1 in
    usleep(1);
    //ret = GpioInit(XDJA_POR_GPIO,1);  //por  out
    usleep(1);
	// GpioInit(XDJA_POWER_GPIO,1);   //power out

	//2.预打开GPIO节点
    if(int0_handle == -1)
    {
		snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT0_GPIO);
        int0_handle = open(path,O_RDWR);
        if (int0_handle < 0)
		{
        	printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
            return -1;
        }
    }
    usleep(1);
    if(int1_handle ==-1)
    {
		snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_INT1_GPIO);
        int1_handle = open(path,O_RDWR);
        if (int1_handle < 0) {
            printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);
            return -1;
        }
    }
    usleep(1);
#endif
	//3.检查POR为低，执行RESET
//  if(getGpioValue(XDJA_POWER_GPIO)==0)
//     XDJA_SPI_POWER(0,1);//power on
//    if(getGpioValue(XDJA_POR_GPIO) == 0)
//		XDJA_SPI_RESET(0);
    return 0;
}

//
void XDJA_SPI_RESET(int fd)
{
#if 0
	int handle;
	char path[64]={0};
	int ret;

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_POR_GPIO); 
	handle = open(path,O_RDWR);
	if(handle == -1)
	{
		printf("%s:%d  XDJA_SPI_RESET error=%d\n",__FUNCTION__,__LINE__,errno);	  
		return;
	}
	ret = write(handle,"0",1);
	XDJA_Delayms (200);
	if (lseek(handle, 0, SEEK_SET) == -1)
	{
		printf("lseek error\n");
		return ;
	}
	ret = write(handle,"1",1);
	//printf("%s:%d XDJA_SPI_RESET OK",__FUNCTION__,__LINE__);
	close(handle);
#endif
}

//SPI上电
void XDJA_SPI_POWER(int fd,int value)
{
#if 0
	int handle;
	char path[64]={0};
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", XDJA_POWER_GPIO); 
	handle = open(path,O_RDWR);	
	if(handle == -1)
	{
		printf("%s:%d XDJA_SPI_POWER error=%d\n",__FUNCTION__,__LINE__,errno);	  
		return;
	}
	if(value ==1)
		write(handle,"1",1);
	else if(value ==0)
		write(handle,"0",1);
	close(handle);
#endif
}

#define SPI_MAX_LEN 2048
static const __u8 TX_FF_BUF[SPI_MAX_LEN]={0xff};
static __u8 RX_BUF[SPI_MAX_LEN] = {0};
static void dumpdata(unsigned char* data,int len)
{
	int  i = 0;
	for(i = 0;i<len;i++)
	{
		printf("%02x",data[i]);
		if( (i+1)%8 == 0)
			printf("\n");
	}
	printf("\n");
}
int recv_bytes(int fd,unsigned char* buf,int len)
{
    int ret = -1;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)TX_FF_BUF, //must use 0xFF
                .rx_buf = (unsigned long)buf,
                .len = len,
                .delay_usecs = 0,
                .speed_hz = spi_speed * 1000 * 1000,
                .bits_per_word = 8,
    };
    if(len == 0)
        return 0;
    if( buf == 0 || len < 0  || len > SPI_MAX_LEN )
    {
        printf("SPI. recv_bytes  parameter error \n");
        return -1;
    }
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if( ret < 1 )
    {
		printf("%s:%d XDJA_SPI_POWER error=%d\n",__FUNCTION__,__LINE__,errno);	  
        return -1;
    }
    //printf("recv:%d\n", len);
    //dumpdata(buf,len);
    if( ret != len )
    {
        printf("SPI. retLen != readlen, readlen, retlen=%d \n", ret, len);
        return -1;
    }

    return 0;
}

int  send_bytes(int fd, unsigned char* buf,int len)
{
    int ret = -1;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)buf,
        .rx_buf = (unsigned long)RX_BUF,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = spi_speed * 1000 * 1000,
        .bits_per_word = 8,
    };
    //printf("send:%d\n", len);
    //dumpdata(buf,len);
    if( buf == 0 || len <=0  /*|| len > SPI_MAX_LEN*/)
    {
        printf ("SPI. send_bytes  parameter error \n");
        return -1;
    }
    //use ioctl  to send message
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if( ret < 1 )
    {
		printf("%s:%d XDJA_SPI_POWER error=%d\n",__FUNCTION__,__LINE__,errno);	  
        return -1;
    }
    if( ret != len )
    {
        printf ("SPI. retLen != writeLen, writeLen=%d, retLen=%d \n",ret,len);
        return -1;
    }
    return 0;
}

int XDJA_SPI_Read(int fd, unsigned char* r_buf, int len)
{
    return recv_bytes(fd, r_buf, len);
}

int XDJA_SPI_Write(int fd, unsigned char* w_buf, int len)
{	
    return send_bytes(fd, w_buf, len);
}

void XDJA_Delayms(int ms)
{
    usleep(ms * 1000);
}

void XDJA_SPI_INT0(int fd, int x)
{
	int ret;
	//int handle;
	if(int0_handle <0)
	{
		printf("%s:%d int0_handle < 0\n",__FUNCTION__,__LINE__);	  
		return;
	}
	if (lseek(int0_handle, 0, SEEK_SET) == -1)
	{
		printf("lseek failed,errno=%d\n",errno);
		return ;
	}
	if(x == 0)
	{
		//printf("%s:%d XDJA_SPI_INT0 set  value=0",__FUNCTION__,__LINE__);
		ret = write(int0_handle,"0",1);
	}
	else
	{
		//printf("%s:%d XDJA_SPI_INT0 set  value=1",__FUNCTION__,__LINE__);
		ret = write(int0_handle,"1",1);
	}
}

int XDJA_SPI_INT1(int fd)
{
	int handle;
	int ret;
	unsigned char buf[10] = {0};
	if(int1_handle < 0)
	{
		printf("%s:%d int1_handle < 0 \n",__FUNCTION__,__LINE__);
		return -1;
	}
	if (lseek(int1_handle, 0, SEEK_SET) == -1)
	{
		printf("lseek failed, errno=%d\n",errno);
		return -1;
	}
	ret = read(int1_handle,buf,1);
	//if(ret == -1)

	//close(handle);
	if(buf[0] == '0')
	{
		//printf("%s:%d XDJA_SPI_INT1  value=0",__FUNCTION__,__LINE__);
		return 0;
	}
	else
	{
		//printf("%s:%d XDJA_SPI_INT1  value=1",__FUNCTION__,__LINE__);
		return 1;
	}
}

void XDJA_SPI_FINIAL()
{
	if(-1 != int0_handle)
	{
		close(int0_handle);
		int0_handle=-1;
	}
	if(-1 != int1_handle)
	{
		close(int1_handle);
		int1_handle=-1;
	}
}



