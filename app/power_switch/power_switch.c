#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

static int fd = -1;

static int time = 0;

void gpio_handler(int signum, siginfo_t *siginfo, void * arg)
{
    printf("gpio 239 interrupt has occurred\n");

    printf("siginfo->sicode: %d\n", siginfo->si_code);
    switch(siginfo->si_code){
        case 1:
            alarm(time);
            break;
        case 5:
            alarm(0);
            break;
        default:
            break;
    }

    return;
}

void sig_alarm()
{
    close(fd);
    system("poweroff");
    return ;
}

int main(int argc, char *argv[])
{
    int oflags;
    char c;
    struct sigaction sigio_act;
    sigset_t mask;
    
    sigemptyset(&mask);
    sigaddset(&mask,SIGIO);
    sigio_act.sa_handler = NULL;
    sigio_act.sa_sigaction = gpio_handler;
    sigio_act.sa_flags = SA_SIGINFO;

    if(sigaction(SIGIO, &sigio_act, NULL) < 0){
        perror("sigaction");
        return -1;
    }
  
    signal(SIGALRM, sig_alarm);

    if(argc < 2){
        printf("usage: argv[1]\n");
        return -1;
    }

    time = atoi(argv[1]);
    printf("time: %d\n", time);

    if((fd = open("/dev/power_switch_cdev", O_RDWR)) < 0){
        perror("open");
        return -1;
    }
    fcntl(fd, F_SETOWN, getpid());
    oflags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, oflags | O_ASYNC);
    fcntl(fd, F_SETSIG, SIGIO);

    read(fd, &c, 1);

    if(c == 1){
        printf("trigger off, gpio 239 value is 1\n");
        close(fd);
        system("poweroff");

    }

    while(1){
        usleep(1000);
    }
}
