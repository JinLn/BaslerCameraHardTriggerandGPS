
#include "imu_msg.h"
#include "uart_api.h"
#include "iostream"
#include "string"
#include "cstring"
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>


using namespace std;

#define W 15

gpchc gpchc_msg;
mutex m;

void getSerialPort(){

    std::string sstr = "../Data/GPSData.txt";
    std::ofstream fout(sstr);
    //fout.open( sstr, std::ios::out | std::ios::app);//以写入和在文件末尾添加的方式打开.txt文件，没有的话就创建该文件。
    if (!fout.is_open()) return ;
    std::string recv_string;
    std::string gpg_msg;
    std::string header;

    int start_position = 0;
    int back_position = 0;         //记录$位置
    int comma_position_front = 0;
    int comma_position_back = 0;   //记录，位置

    int index = 0;
    ssize_t ret = 0;
    int read_num = 0;

    char msg_buff[512];
    int uart = uart_open("/dev/ttyUSB0");
    fcntl(uart,F_SETFL,0);  //阻塞
    uart_conf_set(uart,115200,8,1,'N');
    memset(msg_buff,0,512);


   while (1){
        for (read_num=0 ; read_num<500; read_num+=ret) { //强制读取500字符
            ret = read(uart,msg_buff+read_num,500-read_num);
        }
        recv_string = msg_buff;

        start_position = 0;
        back_position = 0;

        while(1){
            // string中find()返回值是字母在母串中的位置（下标记录），如果没有找到，那么会返回一个特别的标记npos。（返回值可以看成是一个int型的数）
            if((start_position = recv_string.find("$",back_position)) == std::string::npos)
                break;
            if((back_position = recv_string.find("$",start_position+1)) == std::string::npos)
                break;
            // string substr(size_type _Off = 0,size_type _Count = npos) const;
            // _Off：所需的子字符串的起始位置。字符串中第一个字符的索引为 0,默认值为0。
            // _Count：复制的字符数目。
            // 返回值：一个子字符串，从其指定的位置开始。
            gpg_msg = recv_string.substr(start_position+1,back_position-start_position-2);//截取$之间的字符串
            // header="GPCHC"
            header = recv_string.substr(start_position+1,5);

            //std::cout<< gpg_msg <<std::endl;

            comma_position_front = 0;
            comma_position_back = 0;
            index = 0;

            //解析GPCHC消息
            // int strncmp ( const char * str1, const char * str2, size_t n );功能是把 str1 和 str2 进行比较，最多比较前 n 个字节，
            // 若str1与str2的前n个字符相同，则返回0；若s1大于s2，则返回大于0的值；若s1 小于s2，则返回小于0的值
            if(!strncmp(header.c_str(),"GPCHC",5)){
                // 复制header.c_str()到gpchc_msg.Header
                strcpy(gpchc_msg.Header,header.c_str());

                while(1){
                    if((comma_position_front = gpg_msg.find(",",comma_position_back)) == std::string::npos)
                        break;
                    if((comma_position_back = gpg_msg.find(",",comma_position_front+1)) == std::string::npos)
                        break;

                    index++;

                    switch (index){
                    case 1:{
                        gpchc_msg.GPSWeek = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    }
                    case 2:{
                        gpchc_msg.GPSTime = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    }
                    case 3:{
                        gpchc_msg.Heading = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    }
                    case 4:{
                        gpchc_msg.Pitch = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    }
                    case 5:{
                        gpchc_msg.Roll = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    }
                    case 6:{
                        gpchc_msg.gyro_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 7:{
                        gpchc_msg.gyro_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 8:{
                        gpchc_msg.gyro_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 9:{
                        gpchc_msg.acc_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 10:{
                        gpchc_msg.acc_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 11:{
                        gpchc_msg.acc_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 12:{
                        gpchc_msg.Lattitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 13:{
                        gpchc_msg.Longitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 14:{
                        gpchc_msg.Altitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 15:{
                        gpchc_msg.Ve = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 16:{
                        gpchc_msg.Vn = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 17:{
                        gpchc_msg.Vu = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 18:{
                        gpchc_msg.V = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 19:{
                        gpchc_msg.NSV1 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 20:{
                        gpchc_msg.NSV2 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 21:{
                        gpchc_msg.Status = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    case 22:{
                        gpchc_msg.Age = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;}
                    default:
                        break;
                    }

                }
                 ///std::cout<< "gpvtg_msg.Speed_over_ground_km:" << gpvtg_msg.Speed_over_ground_km <<std::endl;
                 // UTC time / 纬度　/ 经度　/ 高度 / 航向角　/ 俯仰角　/ 横滚角
                fout  <<fixed<< setprecision(10)<< gpchc_msg.GPSTime <<" " <<gpchc_msg.Lattitude <<" " <<gpchc_msg.Longitude<<" "<<gpchc_msg.Altitude<<" "<<gpchc_msg.Heading<<" "<<gpchc_msg.Pitch<<" "<<gpchc_msg.Roll;
            }
            fout << endl;
        }
    }
}

void readparam(){
    while(1){
        usleep(1000);
        m.lock();//加锁
        cout << "Read gpchc_msg.GPSTime = " << gpchc_msg.GPSTime<< endl;
        m.unlock();//解锁
    }
}
