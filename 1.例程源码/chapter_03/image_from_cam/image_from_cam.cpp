/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::VideoCapture cap(0);

    cv::Mat frame;
    while(1)
    {
        cap>>frame;
        if(frame.empty()){break;}
        cv::imshow("[cam1]",frame);
        cv::waitKey(10);        
    }
    return 0;
}
