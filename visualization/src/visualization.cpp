// visualization.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat srcImage;
	srcImage = cv::imread("D:\\work\\association_track\\test_video\\images\\testvideo060.jpg");
	cv::imshow("显示图像", srcImage);
	cv::waitKey();
	return 0;
    std::cout << "Hello World!\n";
}
