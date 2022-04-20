#include "camthread.h"
#include <pylon/PylonIncludes.h>
#include <pylon/usb/PylonUsbIncludes.h>
#include <pylon/usb/_BaslerUsbCameraParams.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <queue>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "include/getserialport.h"
#include "include/imu_msg.h"

using namespace Pylon;
using namespace std;
using namespace cv;
using namespace Basler_UsbCameraParams;


String_t camnumber0 = "21915583";
String_t camnumber1 = "21915643";
String_t camnumber2 = "21915630";

bool exposureuto = true;

int exposuretime = 20000;

int exposureautotimemax = 20000;
int exposureautotimemin = 200;

bool finishth = false;
bool finishsave = false;
queue<Mat> qcam0, qcam1, qcam2;

int64_t Adjust(int64_t val, int64_t minimum, int64_t maximum, int64_t inc)
{
    // Check the input parameters.
    if (inc <= 0)
    {
        // Negative increments are invalid.
        throw LOGICAL_ERROR_EXCEPTION("Unexpected increment %d", inc);
    }
    if (minimum > maximum)
    {
        // Minimum must not be bigger than or equal to the maximum.
        throw LOGICAL_ERROR_EXCEPTION("minimum bigger than maximum.");
    }

    // Check the lower bound.
    if (val < minimum)
    {
        return minimum;
    }

    // Check the upper bound.
    if (val > maximum)
    {
        return maximum;
    }

    // Check the increment.
    if (inc == 1)
    {
        // Special case: all values are valid.
        return val;
    }
    else
    {
        // The value must be min + (n * inc).
        // Due to the integer division, the value will be rounded down.
        return minimum + (((val - minimum) / inc) * inc);
    }
}

bool th0 = false, th1 = false, th2 = false;

void cam0()
{
    th0 = true;
    while (1) {
        if (th0&&th1&&th2)
        {
            std::string sstr = "../flow/times.txt";
            std::ofstream fout(sstr);
            if (!fout.is_open()) {
                cerr << "not open cam0_time.txt"<< endl;
                return ;
            }
            std::string gettime = "../Data/GPStimeandFPS.txt";
            std::ofstream gettimefout(gettime);
            if (!gettimefout.is_open()) {
                cerr << "not open GPStimeandFPS.txt"<< endl;
                return ;
            }
            PylonAutoInitTerm autoInitTerm;
            int i = 0;
            int q = 0;
            int64_t pre_frame_timestamp, cur_frame_timestamp, grab_time;

            //É??÷?äÁ?
            //SYSTEMTIME sys_time;
            vector<Mat> images;
            try
            {
                CBaslerUsbInstantCamera usbcamera;
                CBaslerUsbGrabResultPtr ptrGrabResult;

                CTlFactory& tlFactory = CTlFactory::GetInstance();
                CBaslerUsbDeviceInfo dev;
                dev.SetSerialNumber(camnumber0);
                dev.SetDeviceClass(BaslerUsbDeviceClass);
                usbcamera.Attach(tlFactory.CreateDevice(dev));

                usbcamera.Open();

                /**********************************************************************/
                GenApi::INodeMap& nodemap = usbcamera.GetNodeMap();

                /* ?â?¾???????¹®í */
                usbcamera.TriggerMode.SetValue(TriggerMode_On);
                /* ExposureAuto_Off, ExposureAuto_Once,  ExposureAuto_Continuous */
                if (!exposureuto) {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Off);
                    usbcamera.ExposureTime.SetValue(exposuretime);
                }
                else {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Continuous);
                    usbcamera.AutoExposureTimeLowerLimit.SetValue(exposureautotimemin);
                    usbcamera.AutoExposureTimeUpperLimit.SetValue(exposureautotimemax);
                }
                usbcamera.TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
                usbcamera.TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
                usbcamera.TriggerActivation.SetValue(Basler_UsbCameraParams::TriggerActivation_RisingEdge);

                //usbcamera.ShutterMode.SetIntValue(ShutterMode_Global);
                //usbcamera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
                //usbcamera.TriggerSource.SetValue(TriggerSource_Line1);
                //usbcamera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                //usbcamera.TriggerDelay.SetValue(0.0); // unit is microseconds
                //usbcamera.LineDebouncerTime.SetValue(10.0); // unit is microseconds

                ////???????ú?É???í???Í¾ß??
                //GenApi::CIntegerPtr OffsetX = nodemap.GetNode("OffsetX");
                //GenApi::CIntegerPtr OffsetY = nodemap.GetNode("OffsetY");

                //GenApi::CIntegerPtr width = nodemap.GetNode("Width");
                //GenApi::CIntegerPtr height = nodemap.GetNode("Height");
                //width->SetValue(640);
                //height->SetValue(480);
                //OffsetX->SetValue(0);
                //OffsetY->SetValue(0);
        /**********************************************************************/
                GenApi::CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
                GenApi::CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
                GenApi::CIntegerPtr width(nodemap.GetNode("Width"));
                GenApi::CIntegerPtr height(nodemap.GetNode("Height"));

                // On some cameras the offsets are read-only,
                // so we check whether we can write a value. Otherwise, we would get an exception.
                // GenApi has some convenience predicates to check this easily.
                if (IsWritable(offsetX))
                {
                    offsetX->SetValue(offsetX->GetMin());
                }
                if (IsWritable(offsetY))
                {
                    offsetY->SetValue(offsetY->GetMin());
                }

                // Some properties have restrictions. Use GetInc/GetMin/GetMax to make sure you set a valid value.
                int64_t newWidth = width->GetMax();
                newWidth = Adjust(newWidth, width->GetMin(), width->GetMax(), width->GetInc());

                int64_t newHeight = height->GetMax();
                newHeight = Adjust(newHeight, height->GetMin(), height->GetMax(), height->GetInc());

                width->SetValue(newWidth);
                height->SetValue(newHeight);

                cout << "OffsetX 0         : " << offsetX->GetValue() << endl;
                cout << "OffsetY 0         : " << offsetY->GetValue() << endl;

                cout << "Width 0           : " << width->GetValue() << endl;
                cout << "Height 0          : " << height->GetValue() << endl;

                // Enable chunks in general.
                if (IsWritable(usbcamera.ChunkModeActive))
                {
                    usbcamera.ChunkModeActive.SetValue(true);
                }
                else
                {
                    throw RUNTIME_EXCEPTION("The camera doesn't support chunk features");
                }

                // Enable time stamp chunks.
                usbcamera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
                usbcamera.ChunkEnable.SetValue(true);
                usbcamera.StartGrabbing(GrabStrategy_OneByOne);
                CImageFormatConverter   formatConverter;//¾???×???
                CPylonImage             pylonImage;//???úÔ­??Í???¾???
                formatConverter.OutputPixelFormat = PixelType_BGR8packed;
                auto start = std::chrono::high_resolution_clock::now();
                auto start1 = std::chrono::high_resolution_clock::now();
                int temp = 0;
                bool flag = false;
                while (usbcamera.IsGrabbing())
                {
                    if(!flag)
                        usbcamera.RetrieveResult(10000, ptrGrabResult, TimeoutHandling_ThrowException);
                    else {
                        usbcamera.RetrieveResult(100, ptrGrabResult, TimeoutHandling_ThrowException);
                    }
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        flag = true;
                        //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                        //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                        //Pylon::DisplayImage(1, ptrGrabResult);
                        //???äÁ?Ö?É?Ö?Î????????ä
                        //GetLocalTime(&sys_time);

                        //?ä?ö???ä
                        //printf("%4d/%02d/%02d %02d:%02d:%02d.%03d ?Ç?Ú%1d\n", sys_time.wYear,
                        //	sys_time.wMonth,
                        //	sys_time.wDay,
                        //	sys_time.wHour,
                        //	sys_time.wMinute,
                        //	sys_time.wSecond,
                        //	sys_time.wMilliseconds,
                        //	sys_time.wDayOfWeek);

                        //Pylon::DisplayImage(1, ptrGrabResult);

                        // Check to see if a buffer containing chunk data has been received.
                        if (PayloadType_ChunkData != ptrGrabResult->GetPayloadType())
                        {
                            throw RUNTIME_EXCEPTION("Unexpected payload type received.");
                        }

                        if (IsReadable(ptrGrabResult->ChunkTimestamp))
                        {
                            //cout << "TimeStamp (Result): " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
                        }
                        //cout << "TimeStamp (Result) 0: " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;

                        char cam0[150] = {};
                        char fpsname[15] = {};
                        sprintf(fpsname,"%06d",i);
                        //sprintf_s(cam0, "./flow/0/%06d.png", i);
                        //string str1 = cam0;
                        i++;
                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat src0 = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                        resize(src0, src0, Size(src0.cols * 0.47, src0.rows * 0.47));
                        cvtColor(src0, src0, COLOR_BGR2GRAY);
                        qcam0.push(src0);
                        //?????time???
                        gettimefout << fixed << setprecision(10)<< gpchc_msg.GPSTime <<" "<< fpsname << endl;

                        //cout << cam0 << endl;
                        //cout << "qcam0 size = " << qcam0.size() << endl;
                        /*while(!qcam0.empty())
                        {
                            cout << "qcam0 size = " << qcam0.size() << endl;
                            sprintf_s(cam0, "./flow/0/%06d.png", q);
                            Mat img_cam0 = qcam0.front();
                            qcam0.pop();
                            cout << cam0 << endl;
                            imwrite(cam0, img_cam0);
                            //Sleep(4);
                            q++;
                        }*/
                        //cv::imshow("cam0", src0);
                        //cv::waitKey(1);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);

                        auto end = std::chrono::high_resolution_clock::now();
                        auto end1 = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<float, std::milli> tm = end - start;
                        std::chrono::duration<float, std::milli> tm1 = end1 - start1;
                        temp++;
                        if (tm1.count() / 1000 > 1)
                        {
                            cout << "FPS 0 = " << temp << endl;
                            start1 = std::chrono::high_resolution_clock::now();
                            temp = 0;
                        }
                        int S = (int)log10(tm.count() / 1000);
                        if ((tm.count() / 1000) < 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) * 10 << "e-01" << endl;
                        }
                        if ((tm.count() / 1000) >= 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) / pow(10, S) << "e+0" << S << endl;
                        }


                        //std::stringstream stemp1;
                        //stemp1 << i;
                        //string scount1 = stemp1.str();
                        //string str1 = "flow//0//cam0_";
                        //str1 += scount1;
                        //str1 += ".png";
                        //i++;
                        //str1.c_str();
                        ////CImagePersistence::Save(ImageFileFormat_Png, str3.c_str(), ptrGrabResult);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);
                        if (finishth)
                            return;
                    }
                    else
                    {
                        cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
                    }
                }

            }
            catch (const GenericException& e)
            {
                // Error handling.
                cerr << "An exception occurred." << endl
                    << e.GetDescription() << endl;
            }

        }
    }
}


void cam1()
{
    th1 = true;
    while (1) {
        if (th0 && th1 && th2)
        {
            std::string sstr = "../flow/times1.txt";
            std::ofstream fout(sstr);
            if (!fout.is_open()) {
                cerr << "not open cam1_time.txt"<< endl;
                return ;
            }
            PylonAutoInitTerm autoInitTerm;
            int i = 0;
            int q = 0;
            int64_t pre_frame_timestamp, cur_frame_timestamp, grab_time;

            //É??÷?äÁ?
            //SYSTEMTIME sys_time;
            vector<Mat> images;
            try
            {
                CBaslerUsbInstantCamera usbcamera;
                CBaslerUsbGrabResultPtr ptrGrabResult;

                CTlFactory& tlFactory = CTlFactory::GetInstance();
                CBaslerUsbDeviceInfo dev;
                dev.SetSerialNumber(camnumber1);
                dev.SetDeviceClass(BaslerUsbDeviceClass);
                usbcamera.Attach(tlFactory.CreateDevice(dev));

                usbcamera.Open();

                /**********************************************************************/
                GenApi::INodeMap& nodemap = usbcamera.GetNodeMap();

                /* ?â?¾???????¹®í */
                usbcamera.TriggerMode.SetValue(TriggerMode_On);
                /* ExposureAuto_Off, ExposureAuto_Once,  ExposureAuto_Continuous */
                if (!exposureuto) {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Off);
                    usbcamera.ExposureTime.SetValue(exposuretime);
                }
                else {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Continuous);
                    usbcamera.AutoExposureTimeLowerLimit.SetValue(exposureautotimemin);
                    usbcamera.AutoExposureTimeUpperLimit.SetValue(exposureautotimemax);
                }
                usbcamera.TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
                usbcamera.TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
                usbcamera.TriggerActivation.SetValue(Basler_UsbCameraParams::TriggerActivation_RisingEdge);

                //usbcamera.ShutterMode.SetIntValue(ShutterMode_Global);
                ///usbcamera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
                //usbcamera.TriggerSource.SetValue(TriggerSource_Line1);
                //usbcamera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                //usbcamera.TriggerDelay.SetValue(0.0); // unit is microseconds
                //usbcamera.LineDebouncerTime.SetValue(10.0); // unit is microseconds

                ////???????ú?É???í???Í¾ß??
                //GenApi::CIntegerPtr OffsetX = nodemap.GetNode("OffsetX");
                //GenApi::CIntegerPtr OffsetY = nodemap.GetNode("OffsetY");

                //GenApi::CIntegerPtr width = nodemap.GetNode("Width");
                //GenApi::CIntegerPtr height = nodemap.GetNode("Height");
                //width->SetValue(640);
                //height->SetValue(480);
                //OffsetX->SetValue(0);
                //OffsetY->SetValue(0);
        /**********************************************************************/
                GenApi::CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
                GenApi::CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
                GenApi::CIntegerPtr width(nodemap.GetNode("Width"));
                GenApi::CIntegerPtr height(nodemap.GetNode("Height"));

                // On some cameras the offsets are read-only,
                // so we check whether we can write a value. Otherwise, we would get an exception.
                // GenApi has some convenience predicates to check this easily.
                if (IsWritable(offsetX))
                {
                    offsetX->SetValue(offsetX->GetMin());
                }
                if (IsWritable(offsetY))
                {
                    offsetY->SetValue(offsetY->GetMin());
                }

                // Some properties have restrictions. Use GetInc/GetMin/GetMax to make sure you set a valid value.
                int64_t newWidth = width->GetMax();
                newWidth = Adjust(newWidth, width->GetMin(), width->GetMax(), width->GetInc());

                int64_t newHeight = height->GetMax();
                newHeight = Adjust(newHeight, height->GetMin(), height->GetMax(), height->GetInc());

                width->SetValue(newWidth);
                height->SetValue(newHeight);

                cout << "OffsetX 1          : " << offsetX->GetValue() << endl;
                cout << "OffsetY 1          : " << offsetY->GetValue() << endl;

                cout << "Width 1            : " << width->GetValue() << endl;
                cout << "Height 1           : " << height->GetValue() << endl;


                // Enable chunks in general.
                if (IsWritable(usbcamera.ChunkModeActive))
                {
                    usbcamera.ChunkModeActive.SetValue(true);
                }
                else
                {
                    throw RUNTIME_EXCEPTION("The camera doesn't support chunk features");
                }

                // Enable time stamp chunks.
                usbcamera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
                usbcamera.ChunkEnable.SetValue(true);

                usbcamera.StartGrabbing(GrabStrategy_OneByOne);
                CImageFormatConverter   formatConverter;//¾???×???
                CPylonImage             pylonImage;//???úÔ­??Í???¾???
                formatConverter.OutputPixelFormat = PixelType_BGR8packed;
                auto start = std::chrono::high_resolution_clock::now();
                auto start1 = std::chrono::high_resolution_clock::now();
                int temp = 0;
                bool flag = false;
                while (usbcamera.IsGrabbing())
                {
                    if (!flag)
                        usbcamera.RetrieveResult(10000, ptrGrabResult, TimeoutHandling_ThrowException);
                    else {
                        usbcamera.RetrieveResult(100, ptrGrabResult, TimeoutHandling_ThrowException);
                    }
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        flag = true;
                        //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                        //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                        //Pylon::DisplayImage(1, ptrGrabResult);
                        //???äÁ?Ö?É?Ö?Î????????ä
                        //GetLocalTime(&sys_time);

                        //?ä?ö???ä
                        //printf("%4d/%02d/%02d %02d:%02d:%02d.%03d ?Ç?Ú%1d\n", sys_time.wYear,
                        //	sys_time.wMonth,
                        //	sys_time.wDay,
                        //	sys_time.wHour,
                        //	sys_time.wMinute,
                        //	sys_time.wSecond,
                        //	sys_time.wMilliseconds,
                        //	sys_time.wDayOfWeek);

                        //Pylon::DisplayImage(1, ptrGrabResult);

                        // Check to see if a buffer containing chunk data has been received.
                        if (PayloadType_ChunkData != ptrGrabResult->GetPayloadType())
                        {
                            throw RUNTIME_EXCEPTION("Unexpected payload type received.");
                        }

                        if (IsReadable(ptrGrabResult->ChunkTimestamp))
                        {
                            //cout << "TimeStamp (Result): " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
                        }
                        //cout << "TimeStamp (Result) 1: " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;

                        char cam0[150] = {};
                        //sprintf_s(cam0, "./flow/1/%06d.png", i);
                        //cout << cam0 << endl;
                        //string str1 = cam0;
                        i++;
                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat src0 = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                        resize(src0, src0, Size(src0.cols * 0.47, src0.rows * 0.47));
                        cvtColor(src0, src0, COLOR_BGR2GRAY);
                        qcam1.push(src0);
                        //cout << cam0 << endl;
                        /*while (!qcam1.empty())
                        {
                            cout << "qcam1 size = " << qcam1.size() << endl;
                            sprintf_s(cam0, "./flow/1/%06d.png", q);
                            Mat img_cam1 = qcam1.front();
                            qcam1.pop();
                            cout << cam0 << endl;
                            imwrite(cam0, img_cam1);
                            //Sleep(4);
                            q++;
                        }*/
                        //imwrite(cam0, src0);
                        //cv::imshow("cam1", src0);
                        //cv::waitKey(1);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);

                        auto end = std::chrono::high_resolution_clock::now();
                        auto end1 = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<float, std::milli> tm = end - start;
                        std::chrono::duration<float, std::milli> tm1 = end1 - start1;
                        temp++;
                        if (tm1.count() / 1000 > 1)
                        {
                            cout << "FPS 1 = " << temp << endl;
                            start1 = std::chrono::high_resolution_clock::now();
                            temp = 0;
                        }
                        int S = (int)log10(tm.count() / 1000);
                        if ((tm.count() / 1000) < 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) * 10 << "e-01" << endl;
                        }
                        if ((tm.count() / 1000) >= 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) / pow(10, S) << "e+0" << S << endl;
                        }
                        //std::stringstream stemp1;
                        //stemp1 << i;
                        //string scount1 = stemp1.str();
                        //string str1 = "flow//1//cam1_";
                        //str1 += scount1;
                        //str1 += ".png";
                        //i++;
                        //str1.c_str();
                        ////CImagePersistence::Save(ImageFileFormat_Png, str3.c_str(), ptrGrabResult);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);
                        if (finishth)
                            return;
                    }
                    else
                    {
                        cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
                    }

                }

            }
            catch (const GenericException& e)
            {
                // Error handling.
                cerr << "An exception occurred." << endl
                    << e.GetDescription() << endl;
            }
        }
    }
}

void cam2()
{
    th2 = true;
    while (1) {
        if (th0&&th1&&th2)
        {
            std::string sstr = "../flow/times2.txt";//???äÎÄ??
            std::ofstream fout(sstr);
            if (!fout.is_open()) {
                cerr << "not open cam2_time.txt"<< endl;
                return ;
            }
            PylonAutoInitTerm autoInitTerm;
            int i = 0;
            int q = 0;
            int64_t pre_frame_timestamp, cur_frame_timestamp, grab_time;

            //É??÷?äÁ?
            //SYSTEMTIME sys_time;
            vector<Mat> images;
            try
            {
                CBaslerUsbInstantCamera usbcamera;
                CBaslerUsbGrabResultPtr ptrGrabResult;

                CTlFactory& tlFactory = CTlFactory::GetInstance();
                CBaslerUsbDeviceInfo dev;
                dev.SetSerialNumber(camnumber2);
                dev.SetDeviceClass(BaslerUsbDeviceClass);
                usbcamera.Attach(tlFactory.CreateDevice(dev));

                usbcamera.Open();

                /**********************************************************************/
                GenApi::INodeMap& nodemap = usbcamera.GetNodeMap();

                /* ?â?¾???????¹®í */
                usbcamera.TriggerMode.SetValue(TriggerMode_On);
                /* ExposureAuto_Off, ExposureAuto_Once,  ExposureAuto_Continuous */
                if (!exposureuto) {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Off);
                    usbcamera.ExposureTime.SetValue(exposuretime);
                }
                else {
                    usbcamera.ExposureAuto.SetValue(ExposureAuto_Continuous);
                    usbcamera.AutoExposureTimeLowerLimit.SetValue(exposureautotimemin);
                    usbcamera.AutoExposureTimeUpperLimit.SetValue(exposureautotimemax);
                }
                usbcamera.TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
                usbcamera.TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
                usbcamera.TriggerActivation.SetValue(Basler_UsbCameraParams::TriggerActivation_RisingEdge);

                //usbcamera.ShutterMode.SetIntValue(ShutterMode_Global);
                ///usbcamera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
                //usbcamera.TriggerSource.SetValue(TriggerSource_Line1);
                //usbcamera.TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                //usbcamera.TriggerDelay.SetValue(0.0); // unit is microseconds
                //usbcamera.LineDebouncerTime.SetValue(10.0); // unit is microseconds

                ////???????ú?É???í???Í¾ß??
                //GenApi::CIntegerPtr OffsetX = nodemap.GetNode("OffsetX");
                //GenApi::CIntegerPtr OffsetY = nodemap.GetNode("OffsetY");

                //GenApi::CIntegerPtr width = nodemap.GetNode("Width");
                //GenApi::CIntegerPtr height = nodemap.GetNode("Height");
                //width->SetValue(640);
                //height->SetValue(480);
                //OffsetX->SetValue(0);
                //OffsetY->SetValue(0);
        /**********************************************************************/
                GenApi::CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
                GenApi::CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
                GenApi::CIntegerPtr width(nodemap.GetNode("Width"));
                GenApi::CIntegerPtr height(nodemap.GetNode("Height"));

                // On some cameras the offsets are read-only,
                // so we check whether we can write a value. Otherwise, we would get an exception.
                // GenApi has some convenience predicates to check this easily.
                if (IsWritable(offsetX))
                {
                    offsetX->SetValue(offsetX->GetMin());
                }
                if (IsWritable(offsetY))
                {
                    offsetY->SetValue(offsetY->GetMin());
                }

                // Some properties have restrictions. Use GetInc/GetMin/GetMax to make sure you set a valid value.
                int64_t newWidth = width->GetMax();
                newWidth = Adjust(newWidth, width->GetMin(), width->GetMax(), width->GetInc());

                int64_t newHeight = height->GetMax();
                newHeight = Adjust(newHeight, height->GetMin(), height->GetMax(), height->GetInc());

                width->SetValue(newWidth);
                height->SetValue(newHeight);

                cout << "OffsetX 2          : " << offsetX->GetValue() << endl;
                cout << "OffsetY 2          : " << offsetY->GetValue() << endl;

                cout << "Width 2            : " << width->GetValue() << endl;
                cout << "Height 2           : " << height->GetValue() << endl;

                // Enable chunks in general.
                if (IsWritable(usbcamera.ChunkModeActive))
                {
                    usbcamera.ChunkModeActive.SetValue(true);
                }
                else
                {
                    throw RUNTIME_EXCEPTION("The camera doesn't support chunk features");
                }

                // Enable time stamp chunks.
                usbcamera.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
                usbcamera.ChunkEnable.SetValue(true);

                usbcamera.StartGrabbing(GrabStrategy_OneByOne);
                CImageFormatConverter   formatConverter;//¾???×???
                CPylonImage             pylonImage;//???úÔ­??Í???¾???
                formatConverter.OutputPixelFormat = PixelType_BGR8packed;
                auto start = std::chrono::high_resolution_clock::now();
                auto start1 = std::chrono::high_resolution_clock::now();
                int temp = 0;
                bool flag = false;
                while (usbcamera.IsGrabbing())
                {
                    if (!flag)
                        usbcamera.RetrieveResult(10000, ptrGrabResult, TimeoutHandling_ThrowException);
                    else {
                        usbcamera.RetrieveResult(100, ptrGrabResult, TimeoutHandling_ThrowException);
                    }
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        flag = true;
                        //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                        //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                        //Pylon::DisplayImage(1, ptrGrabResult);
                        //???äÁ?Ö?É?Ö?Î????????ä
                        //GetLocalTime(&sys_time);

                        //?ä?ö???ä
                        //printf("%4d/%02d/%02d %02d:%02d:%02d.%03d ?Ç?Ú%1d\n", sys_time.wYear,
                        //	sys_time.wMonth,
                        //	sys_time.wDay,
                        //	sys_time.wHour,
                        //	sys_time.wMinute,
                        //	sys_time.wSecond,
                        //	sys_time.wMilliseconds,
                        //	sys_time.wDayOfWeek);

                        //Pylon::DisplayImage(1, ptrGrabResult);

                        // Check to see if a buffer containing chunk data has been received.
                        if (PayloadType_ChunkData != ptrGrabResult->GetPayloadType())
                        {
                            throw RUNTIME_EXCEPTION("Unexpected payload type received.");
                        }

                        if (IsReadable(ptrGrabResult->ChunkTimestamp))
                        {
                            //cout << "TimeStamp (Result): " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;
                        }
                        //cout << "TimeStamp (Result) 2: " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;

                        char cam0[150] = {};
                        //sprintf_s(cam0, "./flow/2/%06d.png", i);
                        //cout << cam0 << endl;
                        //string str1 = cam0;
                        i++;
                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat src0 = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                        resize(src0, src0, Size(src0.cols * 0.47, src0.rows * 0.47));
                        cvtColor(src0, src0, COLOR_BGR2GRAY);
                        qcam2.push(src0);
                        //cout << cam0 << endl;
                        /*while (!qcam2.empty())
                        {
                            cout << "qcam2 size = " << qcam2.size() << endl;
                            sprintf_s(cam0, "./flow/2/%06d.png", q);
                            Mat img_cam2 = qcam2.front();
                            qcam2.pop();
                            cout << cam0 << endl;
                            imwrite(cam0, img_cam2);
                            Sleep(4);
                            q++;
                        }*/
                        //imwrite(cam0, src0);
                        //cv::imshow("cam2", src0);
                        //cv::waitKey(1);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);

                        auto end = std::chrono::high_resolution_clock::now();
                        auto end1 = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<float, std::milli> tm = end - start;
                        std::chrono::duration<float, std::milli> tm1 = end1 - start1;
                        temp++;
                        if (tm1.count() / 1000 > 1)
                        {
                            cout << "FPS 2 = " << temp << endl;
                            start1 = std::chrono::high_resolution_clock::now();
                            temp = 0;
                        }
                        int S = (int)log10(tm.count() / 1000);
                        if ((tm.count() / 1000) < 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) * 10 << "e-01" << endl;
                        }
                        if ((tm.count() / 1000) >= 1)
                        {
                            fout << setprecision(13) << (tm.count() / 1000) / pow(10, S) << "e+0" << S << endl;
                        }
                        //std::stringstream stemp1;
                        //stemp1 << i;
                        //string scount1 = stemp1.str();
                        //string str1 = "flow//2//cam2_";
                        //str1 += scount1;
                        //str1 += ".png";
                        //i++;
                        //str1.c_str();
                        ////CImagePersistence::Save(ImageFileFormat_Png, str3.c_str(), ptrGrabResult);
                        //CImagePersistence::Save(ImageFileFormat_Png, str1.c_str(), ptrGrabResult);
                        if (finishth)
                            return;
                    }
                    else
                    {
                        cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
                    }
                }

            }
            catch (const GenericException& e)
            {
                // Error handling.
                cerr << "An exception occurred." << endl
                    << e.GetDescription() << endl;
            }
        }
    }
}

void save0() {
    char cam0[150] = {};
    int q = 0;
    //Sleep(10);
    while (1) {
        while (qcam0.size()>0)
        {
            cout << "qcam0 size = " << qcam0.size() << endl;
            sprintf(cam0, "../flow/0/%06d.png", q);
            Mat img_cam0 = qcam0.front();
            cout << cam0 << endl;
            imwrite(cam0, img_cam0);
            usleep(5);
            q++;
            qcam0.pop();
        }
        if (finishsave && qcam0.empty()) break;
    }
    return;
}

void save1() {
    char cam0[150] = {};
    int q = 0;
    //Sleep(10);
    while (1) {
        while (qcam1.size()>0)
        {
            cout << "qcam1 size = " << qcam1.size() << endl;
            sprintf(cam0, "../flow/1/%06d.png", q);
            Mat img_cam0 = qcam1.front();
            cout << cam0 << endl;
            imwrite(cam0, img_cam0);
            usleep(5);
            q++;
            qcam1.pop();
        }
        if (finishsave && qcam1.empty()) break;
    }
    return;
}

void save2() {
    char cam0[150] = {};
    int q = 0;
    //Sleep(10);
    while (1) {
        while (qcam2.size()>0)
        {
            cout << "qcam2 size = " << qcam2.size() << endl;
            sprintf(cam0, "../flow/2/%06d.png", q);
            Mat img_cam0 = qcam2.front();
            cout << cam0 << endl;
            imwrite(cam0, img_cam0);
            usleep(5);
            q++;
            qcam2.pop();
        }
        if (finishsave && qcam2.empty()) break;
    }
    return;
}

//void finishgrab() {
//	while (1)
//	{
//		//Sleep(1000);
//		if (_kbhit())
//		{
//			int ch = _getch();
//			if (ch == 27)
//			{
//				finishth = true;
//				Sleep(5000);
//				while (1) {
//					if( qcam2.empty()&& qcam1.empty()&& qcam0.empty())
//						finishsave = true;
//					//Sleep(1000);
//					break;
//				}
//				return ;
//			}
//		}
//		if (finishsave)
//			return;
//	}
//}
