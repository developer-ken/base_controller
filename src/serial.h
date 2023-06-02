#ifndef _H_LIBSERIAL_
#define _H_LIBSERIAL_

class Serial
{
public:
    /// @brief 串口控制器
    /// @param port 要操作的串口设备
    /// @param baudrate 波特率
    Serial(const char *port, int baudrate);

    /// @brief 串口收
    /// @param buf 缓冲区
    /// @param len 要读取的数据长度
    /// @return 读取长度
    int Read(char *buf, int len);

    /// @brief 串口发
    /// @param buf 缓冲区
    /// @param len 要发送的数据长度
    /// @return 发送长度
    int Write(const char *buf, int len);

    /// @brief 关闭串口
    void Close();

private:
    int fd = 0;
};

#endif