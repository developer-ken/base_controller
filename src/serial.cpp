#include <stdio.h>     /*标准输入输出*/
#include <string.h>    /*标准字符串处理*/
#include <stdlib.h>    /*标准函数库*/
#include <unistd.h>    /*Unix标准函数*/
#include <sys/types.h> /*数据类型定义，比如一些XXX_t的类型*/
#include <sys/stat.h>  /*返回值结构*/
#include <fcntl.h>     /*文件控制*/
#include <termios.h>   /*PPSIX终端控制*/
#include <errno.h>     /*错误号*/
#include "serial.h"

struct termio
{
    tcflag_t c_iflag; /*输入模式标志，unsigned short*/
    tcflag_t c_oflag; /*输出模式标志，unsigned short*/
    tcflag_t c_cflag; /*控制模式标志，unsigned short*/
    tcflag_t c_lflag; /*本地模式标志，unsigned short*/
    cc_t c_line;      /*线路规程，unsigned char*/
    cc_t c_cc[NCCS];  /*控制字符，unsigned char*/
    speed_t c_ispeed; /*输入波特率，unsigned int*/
    speed_t c_ospeed; /*输出波特率，unsigned int*/
};

int open_dev(const char *Dev)
{
    int fd = 0;
    fd = open(Dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd)
    {
        perror("open COM error");
        return -1;
    }
    return fd;
}

/*设置串口通信速率*/
int set_speed(int fd, int speed)
{
    int index = 0;
    int status = 0;
    int speed_arr[] = {
        B38400,
        B19200,
        B9600,
        B4800,
        B2400,
        B1200,
        B300,
        B38400,
        B19200,
        B9600,
        B4800,
        B2400,
        B1200,
        B300,
    };
    int name_arr[] = {
        38400,
        19200,
        9600,
        4800,
        2400,
        1200,
        300,
        38400,
        19200,
        9600,
        4800,
        2400,
        1200,
        300,
    };
    struct termios Opt = {0};

    tcgetattr(fd, &Opt);
    for (index = 0; index < sizeof(speed_arr) / sizeof(int); index++)
    {
        if (speed == name_arr[index])
        {
            /*tcflush函数刷清(抛弃)输入缓存(终端驱动程序已接收到，但用户程序尚未读)或输出缓
            存(用户程序已经写，但尚未发送)。queue参数应是下列三个常数之一：TCIFLUSH刷清输入
            队列，TCOFLUSH刷清输出队列，TCIOFLUSH刷清输入输出队列。*/

            /*设置前flush*/
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[index]);
            cfsetospeed(&Opt, speed_arr[index]);

            /*tcsetattr(串口描述符, 立即使用或者其他标示, 指向termios的指针)，通过
            tcsetattr函数把新的属性设置到串口上。*/
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (0 != status)
            {
                perror("tcsetattr COM error");
                return -1;
            }

            /*设置后flush*/
            tcflush(fd, TCIOFLUSH);
            return 0;
        }
    }

    return 0;
}

/*设置数据位、停止位和校验位*/
int set_parity(int fd, int databits, int stopbits, int parity)
{
    struct termios Opt = {0};

    if (0 != tcgetattr(fd, &Opt))
    {
        perror("tcgetattr COM error");
        return -1;
    }

    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
    Opt.c_oflag &= ~OPOST;                          /*Output*/

    /*设置数据位，取值为7或8*/
    Opt.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
        Opt.c_cflag |= CS7;
        break;
    case 8:
        Opt.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return -1;
        break;
    }

    /*设置停止位，取值为1或2*/
    switch (stopbits)
    {
    case 1:
        Opt.c_cflag &= ~CSTOPB;
        break;
    case 2:
        Opt.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return -1;
        break;
    }

    /*设置校验位，取值为E,N,O,S*/
    switch (parity)
    {
    case 'e':
    case 'E':
    {
        Opt.c_cflag |= PARENB;  /*Enable parity*/
        Opt.c_cflag &= ~PARODD; /*转换为偶效验*/
        Opt.c_iflag |= INPCK;   /*Disnable parity checking*/
    }
    break;
    case 'n':
    case 'N':
    {
        Opt.c_cflag &= ~PARENB; /*Clear parity enable*/
        Opt.c_iflag &= ~INPCK;  /*Enable parity checking*/
    }
    break;
    case 'o':
    case 'O':
    {
        Opt.c_cflag |= (PARODD | PARENB); /*设置为奇效验*/
        Opt.c_iflag |= INPCK;             /*Disnable parity checking*/
    }
    break;
    case 's': /*as no parity*/
    case 'S': /*as no parity*/
    {
        Opt.c_cflag &= ~PARENB;
        Opt.c_cflag &= ~CSTOPB;
    }
    break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return -1;
        break;
    }

    /*设置结构体输入校验位*/
    if ('n' != parity)
    {
        Opt.c_iflag |= INPCK;
    }

    tcflush(fd, TCIFLUSH);
    Opt.c_cc[VTIME] = 150; /*设置超时15秒*/
    Opt.c_cc[VMIN] = 0;    /*更新结构体并立即执行*/
    if (0 != tcsetattr(fd, TCSANOW, &Opt))
    {
        perror("tcsetattr COM error");
        return -1;
    }

    return 0;
}

Serial::Serial(const char *port, int baudrate)
{
    fd = open_dev(port);
    set_speed(fd, baudrate);
    set_parity(fd, 8, 1, 'N');
};

int Serial::Read(char *buf, int len)
{
    return read(fd, buf, len);
}

int Serial::Write(const char *buf, int len)
{
    return write(fd, buf, len);
}

void Serial::Close()
{
    close(fd);
}