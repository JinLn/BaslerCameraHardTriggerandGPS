#pragma once
#ifndef IMU_MSG_H
#define IMU_MSG_H

#include <stdint.h>

struct gpchc{
    char Header[7];      //0 GPCHC 协议头
    double GPSWeek;      //1 自 1980-1-6 至当前的星期数
    double GPSTime;      //2 自本周日 0:00:00 至当前的秒数
    double Heading;      //3 偏航角
    double Pitch;        //4 俯仰角(-90 至 90)
    double Roll;         //5 横滚角(-180 至 180)
    double gyro_x;       //6 陀螺 X 轴
    double gyro_y;       //7 陀螺 Y 轴
    double gyro_z;       //8 陀螺 Z 轴
    double acc_x;        //9 加表 X 轴
    double acc_y;        //10 加表 Y 轴
    double acc_z;        //11 加表 Z 轴
    double Lattitude;     //12 纬度(-90 至 90)
    double Longitude;     //13 经度(-180 至 180)
    double Altitude;      //14 高度,单位(米)
    double Ve;            //15 东向速度,单位(米/秒)
    double Vn;            //16 北向速度,单位(米/秒)
    double Vu;            //17 天向速度,单位(米/秒)
    double V;             //18 车辆速度,单位(米/秒)
    double NSV1;          //19 主天线 1 卫星数
    double NSV2;          //20 副天线 2 卫星数
    double Status;        //21
    double Age;           //22 差分延时
    char Warming;         //23
    char Cs[5];           //24 校验
};


#endif 
