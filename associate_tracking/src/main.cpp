/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019,
*  XIAOSHU Technology  Co., Ltd
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the institute nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: WQ.Tang
*********************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "iou_tracker.h"
#include "utils.h"
#include "auto_color.h"
#include <direct.h>
#include <io.h>
//#define VISUALIZE_DETECTION
//#define VISUALIZE

std::vector<iou_tracker::BoundingBox> readDetections(std::ifstream& box_file)
{
	// Read boxex from file.
	std::vector<iou_tracker::BoundingBox> boxes;
	std::string line;
	std::getline(box_file, line);
	// TODO: check if the first line == file name.
	std::getline(box_file, line);
	int box_nums = std::stoi(line);
	for (int i = 0; i < box_nums; i++)
	{
		iou_tracker::BoundingBox box;
		std::getline(box_file, line);
		std::vector<std::string> strlist = utils::split(line);
		assert(strlist.size() == 18);
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
		box.blur = std::stof(strlist[17]);
		boxes.push_back(box);
	}
	return boxes;
}

int main()
{
	// Save video.
	int isColor = 1, fps = 10, frameWidth = 1280, frameHeight = 720;
	cv::VideoWriter video_writer("track_result.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight), isColor);
	assert(video_writer.isOpened());
	std::vector<cv::Mat> image_list;

	// Create iou_tracker instance.
	float sigma_l = 0.35, sigma_h = 0.85, sigma_iou = 0.15, t_min = 4, t_max = 100;
	float boxInflactRatio = 1.3;
	iou_tracker::IOUTracker* tracker = new iou_tracker::IOUTracker(sigma_l, sigma_h, sigma_iou, t_min, t_max);

	// Start tracking by frame.
	std::vector<cv::String> img_paths;	// Save all input image path.
	cv::glob("D:\\work\\association_track\\test_video_with_key_points\\images", img_paths);
	std::vector<iou_tracker::BoundingBox> last_frame_boxes;
	for (int i = 0; i < img_paths.size(); i++)
	{
		cv::String img_path = img_paths[i];
		std::vector<iou_tracker::BoundingBox> boxes;	// Detection bounding boxes of the corresponding image.

		// Open corresponding detection boxex file.
		std::string box_path = utils::replace_all(img_path, "images", "results");
		box_path = utils::replace_all(box_path, ".jpg", ".txt");
		std::ifstream box_file;
		box_file.open(box_path);
		assert(box_file.is_open());
		boxes = readDetections(box_file);
		box_file.close();

		// Inflact boxes
		for (int i = 0; i < boxes.size(); i++)
		{
			boxes[i].x = boxes[i].x - boxes[i].w * (boxInflactRatio - 1) / 2;
			if (boxes[i].x < 0) boxes[i].x = 0;
			boxes[i].y = boxes[i].y - boxes[i].h * (boxInflactRatio - 1) / 2;
			if (boxes[i].y < 0) boxes[i].y = 0;
			boxes[i].w = boxes[i].w + boxes[i].w * (boxInflactRatio - 1);
			if (boxes[i].x + boxes[i].w > frameWidth) boxes[i].w = frameWidth - boxes[i].x - 1;
			boxes[i].h = boxes[i].h + boxes[i].h * (boxInflactRatio - 1);
			if (boxes[i].y + boxes[i].h > frameHeight) boxes[i].h = frameHeight - boxes[i].y - 1;
		}

		// Track one frame
		std::vector<iou_tracker::Trajectory> results = tracker->Track1Frame(boxes);
		if(i == img_paths.size() - 1) results = tracker->TrackLastFrame(boxes);

#ifdef VISUALIZE
		std::vector<iou_tracker::Trajectory> active_trajs = tracker->getActiveTrajectorys();

		// Draw the detection box on input image, then show the picture.
		cv::Mat image = cv::imread(img_path);

		// Draw the last frame result. This should be drawn BEFORE the trajectory!
		for (auto box : last_frame_boxes)
		{
			cv::Point pt1(box.x, box.y);
			cv::Point pt2(box.x + box.w, box.y + box.h);
			cv::rectangle(image, pt1, pt2, cv::Scalar(150, 154, 150), 2);
			last_frame_boxes.clear();
		}

		// Draw active trajectory
		for (auto traj : active_trajs)
		{
			if (traj.max_score < sigma_h) continue;
			iou_tracker::BoundingBox box = traj.boxes.back();
			cv::Point pt1(box.x, box.y);
			cv::Point pt2(box.x + box.w, box.y + box.h);
			cv::rectangle(image, pt1, pt2, auto_color::GetColor(traj.id, 10), 2);

			last_frame_boxes.push_back(box);

			std::stringstream ss;
			ss << traj.id;
			std::string id = ss.str();
			cv::putText(image, id, pt1, 1, 1, cv::Scalar(200, 25, 5), 2);
			//if (traj.max_score < sigma_h) cv::putText(image, id, pt1, 1, 1, cv::Scalar(200, 200, 200), 2);;
		}

#ifdef VISUALIZE_DETECTION
		// Visualize detection boxes.
		for (auto box : boxes)
		{
			if (box.score < sigma_l) continue;
			cv::Point pt1(box.x, box.y);
			cv::Point pt2(box.x + box.w, box.y + box.h);
			cv::rectangle(image, pt1, pt2, cv::Scalar(200, 0, 2), 2);
		}
#endif

		// Visualize active tracker
		cv::putText(image, img_path, cv::Point(50, 50), 1, 1, cv::Scalar(0, 255, 255));
		cv::imshow("Associate tracking", image);
		cv::waitKey(1);

		// Save video
		video_writer<<image;
#endif

		// Save result pictures
		cv::Mat image_orig = cv::imread(img_path);
		image_list.push_back(image_orig);

		for (auto result : results)
		{
			std::string dir_string = "D:\\work\\association_track\\result\\" + utils::float2string(result.id);

			int start_frame = result.start_frame;
			for (int i = 0; i < result.boxes.size(); i++)
			{
				if (result.boxes[i].angle_confidence < 0.6) continue;
				if (result.boxes[i].score < 0.7) continue;
				if (_access(dir_string.c_str(), 0))
				{
					int flag_dir = _mkdir(dir_string.c_str());
					assert(flag_dir == 0);
				}
				cv::Rect face_rect(result.boxes[i].x, result.boxes[i].y, result.boxes[i].w, result.boxes[i].h);
				cv::Mat face_roi = image_list[start_frame + i](face_rect);
				std::string file_name = dir_string + "\\" + utils::float2string(i) + ".jpg";
				std::cout << file_name << std::endl;
				cv::imwrite(file_name, face_roi);
				// write key point info
				// std::string kp_file_name = dir_string + "\\" + utils::float2string(i) + ".txt";
				// std::ofstream kp_file(kp_file_name);
				// kp_file << result.boxes[i].score << " " << result.boxes[i].roll << " " << result.boxes[i].pitch \
					<< " " << result.boxes[i].yaw << " " << result.boxes[i].angle_confidence << " " << result.boxes[i].blur << std::endl;
				// kp_file.close();
			}
		} // End saving
	}
}
