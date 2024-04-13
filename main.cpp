//
// Created by ziyu on 24-3-28.
//

#include "ZiYuSerialPort.hpp"
#include "iostream"

// 设置输入结构体
struct InTest_t
{
    int input = 12;
};

// 设置输出结构体
struct OutTest_t
{
    int output = 100;
};

int main()
{
    ZiYuSerialPort<InTest_t, OutTest_t> serialPort("/dev/ttyUSB0", BAUD_RATE_115200); // 打开串口设置波特率
    serialPort.open(100, READ_AND_WRITE); // 设置频率和串口模式
    OutTest_t outTest; // 创建输出变量
    InTest_t inTest; // 创建输入变量
    inTest.input = 10000; // 设置输入变量值
    serialPort.updateSentBuffer(inTest); // 向发送线程上传值

    while (true)
    {
        serialPort.getReadBuffer(outTest); // 从输出缓冲区中取值
        std::cout << outTest.output << std::endl; // 输出读取到的内容
    }
    return 0;
}