#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include "gpio_operate.h"

/*check for gpio in /sys/class/gpio, success return 0 */
int check_gpio(int n)  
{  
	char path[GPIO_PATH_SIZE];  

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", n); 
	if(access(path, F_OK) != 0){
		if(GPIO_DEBUG)
			printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);	  
		return -1;
	}
	return 0;  
}

//create gpio node in /sys/class/gpio
int export_gpio(int n)  
{  
	char path[GPIO_PATH_SIZE];  
	int handle;
	char exportStr[GPIO_READSTR_SIZE]={0};  
	int ret;

	handle = open("/sys/class/gpio/export",O_WRONLY);
	if(handle == -1){
		printf("%s:%d gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;
	}else{
		if(GPIO_DEBUG)
			printf("%s:%d gpio=%d open success\n",__FUNCTION__,__LINE__,n); 
	}
	snprintf(exportStr, sizeof(exportStr), "%d", n); 
	if(GPIO_DEBUG)
		printf("%s %d\n",exportStr,strlen(exportStr));
	ret = write(handle,exportStr,strlen(exportStr));
	close(handle);
	return ret;  
}

int get_gpio_direction(int n)  
{  
	char path[GPIO_PATH_SIZE];  
	char value_str[GPIO_READSTR_SIZE]={0}; 
	int handle;  
	int ret=0;
	
	if(0!=check_gpio(n))
		export_gpio(n);
	
	memset(value_str,0,GPIO_READSTR_SIZE);
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", n); 
	if(GPIO_DEBUG)
		printf("%s-%d: %s\n", __FUNCTION__,__LINE__, path);
	handle = open(path, O_RDWR);  
	if (handle <= 0){  
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno); 
		return -1;  
	}  

	if (read(handle, value_str, DIRECTION_SIZE) < 0){ 
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;  
	}
	close(handle);
	
	if(strstr(value_str, "out"))
		return GPIO_OUT;
	else if(strstr(value_str, "in"))
		return GPIO_IN;
	else
		return -1;
}

int set_gpio_direction(int n, int value)
{  
	char path[64];	
	char value_str[GPIO_READSTR_SIZE]={0}; 
	int handle;  
	int ret=0;

	if(0!=check_gpio(n))
		export_gpio(n);

	memset(value_str,0,GPIO_READSTR_SIZE);
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", n); 
	handle = open(path, O_RDWR);  
	if (handle <= 0) {	
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;	
	}
	if (read(handle, value_str, DIRECTION_SIZE) < 0) { 
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;	
	} 

	lseek(handle,0,0);
	if(value){
		ret = write(handle,"in",DIRECTION_SIZE);
		if(GPIO_DEBUG)
			printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);  
	}else{
		ret = write(handle,"out",DIRECTION_SIZE);
		if(GPIO_DEBUG)
			printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);  
	}
	close(handle);
	return 0; 

}

/*get gpio value*/
int get_gpio_value(int n) 
{  
	char path[GPIO_PATH_SIZE];  
	char value_str[GPIO_READSTR_SIZE]={0};  
	int fd;
	
	if(0!=check_gpio(n))
		export_gpio(n);
	
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", n); 
	fd = open(path, O_RDONLY);  
	if (fd <= 0) {  
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;  
	}  

	if (read(fd, value_str, 1) < 0) {  
		printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);	  
		return -1;  
	}  
	close(fd);
	
	if(value_str[0] == '0')
		return GPIO_OUT;
	else if(value_str[0] == '1')
		return GPIO_IN;
	else
		return -1;
 }

/*set gpio value*/
int set_gpio_value(int n, int value) 
{  
	char path[GPIO_PATH_SIZE];  
	char value_str[GPIO_READSTR_SIZE]={0};  
	int handle;
	int ret=0;

	if(0!=check_gpio(n)){
		if(GPIO_DEBUG)
			printf("%s line%d: gpio%d is not exit\n", __FUNCTION__, __LINE__, n);
		export_gpio(n);
	}
	memset(value_str,0,GPIO_READSTR_SIZE);
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", n); 
	handle = open(path, O_RDWR);  
	if (handle <= 0) {  
		printf("%s:%d  gpio=%d error=%d\n",__FUNCTION__,__LINE__,n,errno);
		return -1;  
	}  

	if (read(handle, value_str, 1) < 0) {  
		printf("%s:%d  error=%d\n",__FUNCTION__,__LINE__,errno);	  
		return -1;  
	} 
	lseek(handle,0,0);
	if(value){
		ret = write(handle,"1",DIRECTION_SIZE);
		if(GPIO_DEBUG)
			printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);  
	}else{
		ret = write(handle,"0",DIRECTION_SIZE);
		if(GPIO_DEBUG)
			printf("%s:%d  ret=%d\n",__FUNCTION__,__LINE__, ret);  
	}
	close(handle);
	return 0;
 }

/*GPIO init
  *direction: 1 means in;
  *		  0 means out;
  *
  *value: 1 means high
  *		   0 means low
  */
int gpio_init(int n,int direction, int value)  
{
	int ret;
	
	ret = check_gpio(n);
	if(ret == -1){
		ret = export_gpio(n);
		if(ret == -1)
			return ret;
	}
	
	if(direction != get_gpio_direction(n))
	{
		set_gpio_direction(n, direction);
	}
	if(value != get_gpio_value(n))
	{
		if(GPIO_OUT == get_gpio_direction(n))
		{
			set_gpio_value(n, value);
		}
		if(GPIO_IN == get_gpio_direction(n))
		{
			set_gpio_direction(n, GPIO_OUT);
			set_gpio_value(n, value);
			set_gpio_direction(n, GPIO_IN);
		}
	}
	
	return 0;
}
#if 0
int main(int argc, char **argv)
{
	int n = atoi(argv[1]);
	int d = atoi(argv[2]);
	int v = atoi(argv[3]);
	int op = atoi(argv[4]);
	if(op == 1)
		printf("gpio %d: direction %d\n", n, get_gpio_direction(n));
	else if(op == 2)
		printf("gpio %d: value %d\n", n, get_gpio_value(n));
	else if(op == 3){
		set_gpio_direction(n, d);
	}else if(op ==4){
		set_gpio_value(n, v);
	}else
		printf("input error\n");
	
	return 0;
}
#endif
