#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat srcImage;
	srcImage = cv::imread("D:\\work\\association_track\\test_video\\images\\testvideo060.jpg");
	cv::imshow("��ʾͼ��", srcImage);
	cv::waitKey();
	return 0;
	std::cout << "Hello World!\n";
}
