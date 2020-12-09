#include "CameraApi.h" //相机SDK的API头文件

#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

unsigned char           * g_pRgbBuffer1;     //处理后数据缓存区
unsigned char           * g_pRgbBuffer2;     //处理后数据缓存区

int main()
{

    int                     iCameraCounts = 2;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[2];
    vector<int>                     hCamera = {1, 1};
    tSdkCameraCapbility     tCapability1;      //设备描述信息
    tSdkCameraCapbility     tCapability2;      //设备描述信息
    tSdkFrameHead           sFrameInfo1;
    tSdkFrameHead           sFrameInfo2;
    BYTE*			        pbyBuffer1;
    BYTE*			        pbyBuffer2;
    int                     iDisplayFrames = 10000;
    //IplImage *iplImage = NULL;
    vector<int>                     channel={3, 3};

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList[0],-1,-1,&hCamera[0]);
    iStatus = CameraInit(&tCameraEnumList[1],-1,-1,&hCamera[1]);

    //初始化失败
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera[0],&tCapability1);
    CameraGetCapability(hCamera[1],&tCapability2);

    //
    g_pRgbBuffer1= (unsigned char*)malloc(tCapability1.sResolutionRange.iHeightMax*tCapability1.sResolutionRange.iWidthMax*3);
    g_pRgbBuffer2= (unsigned char*)malloc(tCapability2.sResolutionRange.iHeightMax*tCapability2.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera[0]);
    CameraPlay(hCamera[1]);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */

    if(tCapability1.sIspCapacity.bMonoSensor){
        channel[0]=1;
        CameraSetIspOutFormat(hCamera[0],CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel[0]=3;
        CameraSetIspOutFormat(hCamera[0],CAMERA_MEDIA_TYPE_BGR8);
    }

    if(tCapability2.sIspCapacity.bMonoSensor){
        channel[1]=1;
        CameraSetIspOutFormat(hCamera[1],CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel[1]=3;
        CameraSetIspOutFormat(hCamera[1],CAMERA_MEDIA_TYPE_BGR8);
    }

    //循环显示1000帧图像
    while(1)
    {
        if(CameraGetImageBuffer(hCamera[0],&sFrameInfo1,&pbyBuffer1,1000) == CAMERA_STATUS_SUCCESS&&
        CameraGetImageBuffer(hCamera[1],&sFrameInfo2,&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera[0], pbyBuffer1, g_pRgbBuffer1,&sFrameInfo1);
            CameraImageProcess(hCamera[1], pbyBuffer2, g_pRgbBuffer2,&sFrameInfo2);
		    
		    cv::Mat matImage1(
					Size(sFrameInfo1.iWidth,sFrameInfo1.iHeight), 
					sFrameInfo1.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer1
					);
                     
		    cv::Mat matImage2(
					Size(sFrameInfo2.iWidth,sFrameInfo2.iHeight), 
					sFrameInfo2.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer2
					);
			imshow("Opencv Demo1", matImage1);
            imshow("Opencv Demo2", matImage2);

            waitKey(5);

            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera[0],pbyBuffer1);
            CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);

		}
    }

    CameraUnInit(hCamera[0]);
    CameraUnInit(hCamera[1]);
    //注意，现反初始化后再free
    free(g_pRgbBuffer1);
    free(g_pRgbBuffer2);


    return 0;
}

