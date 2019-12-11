// visualization.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "def.h"
#include "utils.h"

std::vector<BoundingBox> readDetections(std::ifstream& box_file)
{
	// Read boxex from file.
	std::vector<BoundingBox> boxes;
	std::string line;
	std::getline(box_file, line);
	// TODO: check if the first line == file name.
	std::getline(box_file, line);
	int box_nums = std::stoi(line);
	for (int i = 0; i < box_nums; i++)
	{
		BoundingBox box;
		std::getline(box_file, line);
		std::vector<std::string> strlist = utils::split(line);
		assert(strlist.size() == 17);
		box.x = std::stof(strlist[0]);
		box.y = std::stof(strlist[1]);
		box.w = std::stof(strlist[2]);
		box.h = std::stof(strlist[3]);
		box.score = std::stof(strlist[4]);
		box.left_eye_x = std::stof(strlist[5]);
		box.left_eye_y = std::stof(strlist[6]);
		box.right_eye_x = std::stof(strlist[7]);
		box.right_eye_y = std::stof(strlist[8]);
		box.nose_x = std::stof(strlist[9]);
		box.nose_y = std::stof(strlist[10]);
		box.mouse_x = std::stof(strlist[11]);
		box.mouse_y = std::stof(strlist[12]);
		box.pitch = std::stof(strlist[13]);
		box.roll = std::stof(strlist[14]);
		box.yaw = std::stof(strlist[15]);
		box.angle_confidence = std::stof(strlist[16]);
		boxes.push_back(box);
	}
	return boxes;
}

std::string float2string(float f)
{
	std::stringstream ss;
	ss << f;
	return ss.str();
}

int main()
{
	std::vector<cv::String> img_paths;	// Save all input image path.
	cv::glob("D:\\work\\association_track\\test_video\\images", img_paths);
	std::cout << "Start reading person data." << std::endl;
	for (auto img_path : img_paths)
	{
		std::vector<BoundingBox> boxes;	// Detection bounding boxes of the corresponding image.

		// Open corresponding detection boxex file.
		std::string box_path = utils::replace_all(img_path, "images", "results");
		box_path = utils::replace_all(box_path, ".jpg", ".txt");
		std::ifstream box_file;
		box_file.open(box_path);
		assert(box_file.is_open());
		boxes = readDetections(box_file);
		box_file.close();

		// Draw the detection box on input image, then show the picture.
		cv::Mat image = cv::imread(img_path);
		for (auto box : boxes) 
		{
			if (box.score < 0.35) continue;
			cv::Point pt1(box.x, box.y);
			cv::Point pt2(box.x + box.w, box.y + box.h);

			std::stringstream ss;
			ss << "" <<box.score << " " << box.pitch << " " << box.roll << " " \
				<< box.yaw << " " << box.angle_confidence;
			std::string text = ss.str();

			cv::rectangle(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			cv::putText(image, text, cv::Point(box.x, box.y - 20) , 1, 1, cv::Scalar(255, 5, 25), 2);

			if (box.angle_confidence > 0.6)
			{
				cv::circle(image, cv::Point(box.left_eye_x, box.left_eye_y), 3, cv::Scalar(255, 0, 0), -1);
				cv::circle(image, cv::Point(box.right_eye_x, box.right_eye_y), 3, cv::Scalar(255, 0, 0), -1);
				cv::circle(image, cv::Point(box.nose_x, box.nose_y), 3, cv::Scalar(255, 0, 0), -1);
				cv::circle(image, cv::Point(box.mouse_x, box.mouse_y), 3, cv::Scalar(255, 0, 0), -1);
			}
		}
		cv::putText(image, img_path, cv::Point(50, 50), 1, 1, cv::Scalar(0, 255, 255));
		cv::imshow("Show detection", image);
		cv::waitKey();
	}

	return 0;
}
