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
    cv::Mat gray_img;
    cv::imshow("[img1]",img);

    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY); 
    cv::imshow("[gray_img]",gray_img);
    
    //calc hist
    cv::MatND dst_hist;
    int dims=1;
    float hranges[]={0,255};
    const float *ranges[]={hranges};
    int size=256;
    int channels=0;

    cv::calcHist(&gray_img,1,&channels,cv::Mat(),dst_hist,dims,&size,ranges);  
    int scale=1;
    cv::Mat dst_img(size*scale, size, CV_8U, cv::Scalar(0));
    
    double minValue=0;
    double maxValue=0;
    cv::minMaxLoc(dst_hist,&minValue,&maxValue,0,0);

    int hpt = cv::saturate_cast<int>(0.9*size);
    for(int i=0;i<256;i++)
    {
        float binValue=dst_hist.at<float>(i);
	int realValue=cv::saturate_cast<int>(binValue*hpt/maxValue);
	cv::rectangle(dst_img,cv::Point(i*scale,size-1),cv::Point((i+1)*scale-1, size-realValue),cv::Scalar(255));
    }
    cv::imshow("[hist]", dst_img);
    
    //equalize
    cv::Mat equal_img;
    cv::equalizeHist(gray_img,equal_img);
    cv::imshow("[equal_img]",gray_img);
    //calc hist
    cv::calcHist(&equal_img,1,&channels,cv::Mat(),dst_hist,dims,&size,ranges);    
    cv::minMaxLoc(dst_hist,&minValue,&maxValue,0,0);
    for(int i=0;i<256;i++)
    {
        float binValue=dst_hist.at<float>(i);
	int realValue=cv::saturate_cast<int>(binValue*hpt/maxValue);
	cv::rectangle(dst_img,cv::Point(i*scale,size-1),cv::Point((i+1)*scale-1, size-realValue),cv::Scalar(255));
    }
    cv::imshow("[equal_hist]", dst_img);

    cv::waitKey(0);

    return 0;
}
