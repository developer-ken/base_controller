#include <stdio.h>      /*标准输入输出*/
#include <string.h>     /*标准字符串处理*/
#include <stdlib.h>     /*标准函数库*/
#include <unistd.h>     /*Unix标准函数*/
#include <sys/types.h>  /*数据类型定义，比如一些XXX_t的类型*/
#include <sys/stat.h>   /*返回值结构*/
#include <fcntl.h>      /*文件控制*/
#include <termios.h>    /*PPSIX终端控制*/
#include <errno.h>      /*错误号*/
#include "pid.h"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world base_controller package\n");
  return 0;
}
