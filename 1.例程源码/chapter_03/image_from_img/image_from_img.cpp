/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::Mat img=cv::imread("1.jpg");
    cv::imshow("[img1]",img);
    cv::waitKey(0);

    return 0;
}
