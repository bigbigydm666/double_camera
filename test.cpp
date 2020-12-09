/*
 * @Author: your name
 * @Date: 2020-12-09 20:51:39
 * @LastEditTime: 2020-12-09 20:57:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /OpenCv/test.cpp
 */
#include "DoubleCamera.hpp"
vector<Mat> imgs;
int main()
{
   DoubleCameraInit();
   while(1)
   {
      imgs = getImage();
      imshow("111", imgs[0]);
      imshow("222", imgs[1]);
      waitKey(5);
      ReleaseBuffers();
   }
   UnInit();
   return 0;
}