/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

//please install opencv-3.x.x and opencv_contrib-3.x.x

#include <iostream>
#include <vector>
#include <ctime>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

int main(int argc, char** argv)
{
    //load image
    cv::Mat src_img=cv::imread("1.jpg");

    cv::Ptr<cv::Feature2D> detector=cv::xfeatures2d::SURF::create(700);
    std::clock_t start_time,end_time;

    start_time=clock();//start timer
    //detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(src_img,keypoints);
    //calculate descriptors
    cv::Mat descriptors;
    detector->compute(src_img,keypoints,descriptors);
    end_time=clock();//end timer

    std::stringstream run_time;
    run_time<<"surf run time="<<(double)(end_time-start_time)/CLOCKS_PER_SEC<<"s";
    std::cout<<run_time.str();
    
    //display sift features
    cv::putText(src_img,run_time.str(),cv::Point(30,30),cv::FONT_HERSHEY_TRIPLEX,1.5,cv::Scalar(0,0,0));
    cv::drawKeypoints(src_img,keypoints,src_img);
    cv::imshow("surf features",src_img);

    cv::waitKey(0);
}
