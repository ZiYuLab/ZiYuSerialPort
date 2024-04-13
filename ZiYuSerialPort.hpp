//
// Created by ziyu on 24-4-5.
//

#ifndef ZIYUSERIALPORT_ZIYUSERIALPORT_HPP
#define ZIYUSERIALPORT_ZIYUSERIALPORT_HPP

#ifdef _WIN32
#include <windows.h>
#define imSleep(microsecond) Sleep(microsecond) // ms
#else
#include <unistd.h>
#define imSleep(microsecond) usleep(1000 * microsecond) // ms
#endif

#define DEBUG

#ifdef DEBUG
#include <assert.h>
#define ASSERT(f) assert(f)
#else
#define ASSERT(f) ((void)0)
#endif

#include <vector>
#include <thread>
#include <iostream>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "CSerialPort/SerialPortListener.h"
#include <stdio.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <cstring>

enum BaudRate_t
{
    BAUD_RATE_115200 = itas109::BaudRate115200,
    BAUD_RATE_9600 = itas109::BaudRate9600,
    BAUD_RATE_19200 = itas109::BaudRate19200
};

enum SerialMode_t
{
    READ_AND_WRITE = 1,
    READ_ONLY = 2,
    WRITE_ONLY = 3
};

class Listener : public itas109::CSerialPortListener
{
private:
    itas109::CSerialPort *p_sp;
public:
    std::shared_ptr<void> readBufferPtr_;
    unsigned int readBufferSize_ = 0;

public:
    Listener(itas109::CSerialPort *sp)
            : p_sp(sp)
    {
    }

    void onReadEvent(const char *portName, unsigned int readBufferLen)
    {
        //
        // std::cout << readBufferSize_ << std::endl;
        if (readBufferLen)
        {
            auto data = new uint8_t [readBufferLen];
            if (data)
            {
                p_sp->readData(data, readBufferLen);
                readBufferSize_ = 0; // 数据复制前置0,防止异步状况下,出现size != 0 但是地址为空情况
                readBufferPtr_.reset(data);
                readBufferSize_ = readBufferLen;
            }
        }
    }

    std::shared_ptr<void> getData()
    {
        if (readBufferSize_)
            return readBufferPtr_;
        else
            return nullptr;
    }
};

template<class TIn, class TOut>
class ZiYuSerialPort {
private:
    char *portName_ = nullptr;
    itas109::CSerialPort *sp_ = nullptr;
    Listener *listener_ = nullptr;
    std::mutex writeMutex_;
    std::shared_ptr<TIn> writeBuffer_ = std::make_shared<TIn>();
//    std::shared_ptr<TOut> readBuffer_ = std::make_shared<TOut>();
//    Listener *listener_;
    bool threadRun_ = true;
    std::thread *wThread_;


//    static void readThread(itas109::CSerialPort *serial, int frequency, std::shared_ptr<void> readBuffer, unsigned int readBufferSize, std::mutex readMutex);
    static void writeThread(itas109::CSerialPort *serial, int frequency, std::shared_ptr<void> writeBuffer, unsigned int writeBufferSize, std::mutex *writeMutex, bool *threadRun);

public:
    /*
     * 参数依次为
     * 串口地址
     * 波特率
     * 缓冲区大小(未实装)
     */
    explicit ZiYuSerialPort(char *portName, BaudRate_t baudRate, unsigned int bufferSize = 4096);
    ~ZiYuSerialPort();

    bool open(int frequency, SerialMode_t serialMode = READ_AND_WRITE);

    bool updateSentBuffer(TIn & inBuffer);
    bool getReadBuffer(TOut & outBuffer);
};


// 线程函数
template<class TIn, class TOut>
void ZiYuSerialPort<TIn, TOut>::writeThread(itas109::CSerialPort *serial, int frequency, std::shared_ptr<void> writeBuffer, unsigned int writeBufferSize,
                                            std::mutex *writeMutex, bool *threadRun)
{
    while (*threadRun)
    {
        const int sentNum = serial->writeData(writeBuffer.get(), writeBufferSize);
        if (sentNum == -1)
        {
//            printf("\e[1;31m[SERIAL_ERROR]\e[0m Serial Sent Fail With ERROR_CODE: %d\n", serial->getLastError());
            printf("\e[1;31m[SERIAL_ERROR]\e[0m Serial Sent Fail!");
        }
        imSleep((int)round(1000.0 / frequency));
    }
}

//template<class TIn, class TOut>
//void ZiYuSerialPort<TIn, TOut>::readThread(itas109::CSerialPort *serial, int frequency, std::shared_ptr<void> readBuffer, unsigned int readBufferSize,
//                                           std::mutex readMutex)
//{
////    serial->readAllData()
//}


template<class TIn, class TOut>
ZiYuSerialPort<TIn, TOut>::ZiYuSerialPort(char *portName, BaudRate_t baudRate, unsigned int bufferSize)
        :portName_(portName)
{
    sp_ = new itas109::CSerialPort();
    sp_->init(portName_, baudRate); // 初始化
    listener_ = new Listener(sp_);
    printf("\e[0;34m[SERIAL_INFO]\e[0m %s\n", sp_->getVersion());
}

template<class TIn, class TOut>
ZiYuSerialPort<TIn, TOut>::~ZiYuSerialPort()
{
    threadRun_ = false;
    wThread_->join();
    sp_->close();
    delete listener_;
    delete sp_;
    printf("\e[0;34m[SERIAL_INFO]\e[0m Serial Port Stopped!\n");
}

template<class TIn, class TOut>
bool ZiYuSerialPort<TIn, TOut>::getReadBuffer(TOut &outBuffer)
{
    //std::cout << listener_->readBufferSize_ << std::endl;
    if (listener_->readBufferSize_ == sizeof(TOut))
    {
        memcpy(&outBuffer, (TOut*)listener_->readBufferPtr_.get(), sizeof(TOut));
//        outBuffer = *;
        return true;
    }
    else
    {
        return false;
    }
}

template<class TIn, class TOut>
bool ZiYuSerialPort<TIn, TOut>::updateSentBuffer(TIn &inBuffer)
{
    //writeMutex_.lock();
    *writeBuffer_ = inBuffer;
    //writeMutex_.unlock();
    return true;
}

template<class TIn, class TOut>
bool ZiYuSerialPort<TIn, TOut>::open(int frequency, SerialMode_t serialMode)
{

    //const bool status = true;
    if (!sp_->open())
    {

        printf("\e[1;31m[SERIAL_ERROR]\e[0m Serial Open Fail With ERROR_CODE: %d\n", sp_->getLastError());
        //printf("\e[1;31m[SERIAL_ERROR]\e[0m Serial Open Fail!\n");
        return false;
    }
    else
    {
        if (serialMode == WRITE_ONLY || serialMode == READ_AND_WRITE)
        {
            // 开写线程
            wThread_ = new std::thread(writeThread, sp_, frequency, writeBuffer_, sizeof(TIn), &writeMutex_, &threadRun_);
//            wThread.detach();
        }

        if (serialMode == READ_AND_WRITE || serialMode == READ_ONLY)
        {
            // 设置读回调函数
            sp_->connectReadEvent(listener_);
        }

        printf("\e[0;34m[SERIAL_INFO]\e[0m Serial Port Opened in \"%s\"\n", portName_);
        return true;
    }
//    return status;
}

#endif //ZIYUSERIALPORT_ZIYUSERIALPORT_HPP
