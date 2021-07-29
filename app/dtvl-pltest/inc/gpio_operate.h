#define GPIO_DEBUG 0
#define DIRECTION_SIZE 3
#define GPIO_PATH_SIZE 64
#define GPIO_READSTR_SIZE 6
#define GPIO_OUT    0
#define GPIO_IN     1
#define GPIO_HIGN   1
#define GPIO_LOW    0

int check_gpio(int n);
int export_gpio(int n);
int get_gpio_direction(int n);
int set_gpio_direction(int n,int value);
int get_gpio_value(int n);
int set_gpio_value(int n, int value);
int gpio_init(int n,int direction, int value);
