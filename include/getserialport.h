#pragma once
#ifndef GETSERIALPORT_H
#define GETSERIALPORT_H
#include "imu_msg.h"
#include <thread>
#include <mutex>

    void getSerialPort();

    void readparam();

    //用于保存解析后数据
    extern gpchc gpchc_msg;//全局变量声明

#endif // GETSERIALPORT_H
