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

#pragma once
#include "trajectory_filter.h"
#include <vector>
#include <algorithm>
#include <assert.h>

namespace iou_tracker
{
	TrajectoryFilter::TrajectoryFilter(float sigma_det_score, float sigma_angle_conf, float sigma_roll, float sigma_pitch, float sigma_yaw, float sigma_blur)
	{
		Initialize(sigma_det_score, sigma_angle_conf, sigma_roll, sigma_pitch, sigma_yaw, sigma_blur);
	}

	void TrajectoryFilter::Initialize(float sigma_det_score, float sigma_angle_conf, float sigma_roll, float sigma_pitch, float sigma_yaw, float sigma_blur)
	{
		initialized_ = true;
		
		sigma_det_score_ = sigma_det_score;
		sigma_angle_conf_ = sigma_angle_conf;
		sigma_roll_ = sigma_roll;
		sigma_pitch_ = sigma_pitch;
		sigma_yaw_ = sigma_yaw;
		sigma_blur_ = sigma_blur;
		hist_frame_.clear();
	}

	void TrajectoryFilter::StoreFrame(cv::Mat image_orig, long frame_count)
	{
		assert(initialized_);
		hist_frame_[frame_count] = image_orig;
		if (hist_frame_.size() > 100)
		{
			auto it = hist_frame_.find(frame_count-100);
			hist_frame_.erase(hist_frame_.begin(), it);
		}
	}

	cv::Mat TrajectoryFilter::GetMostCenteredFace(Trajectory traj)
	{
		assert(initialized_);
		float max_score = -10000;
		int best_index = -1;
		int start_frame = traj.start_frame;
		for (int i = 0; i < traj.boxes.size(); i++)
		{
			if (traj.boxes[i].angle_confidence < sigma_angle_conf_) continue;
			if (traj.boxes[i].score < sigma_det_score_) continue;
			if (fabs(traj.boxes[i].yaw) > sigma_yaw_) continue;

			float score = -(fabs(traj.boxes[i].roll) + fabs(traj.boxes[i].pitch) + fabs(traj.boxes[i].yaw)) \
								+ traj.boxes[i].angle_confidence*10 + traj.boxes[i].blur/100;
			if (score > max_score)
			{
				max_score = score;
				best_index = i;
			}
		}
		if (best_index == -1)  return cv::Mat();

		cv::Rect face_rect(traj.boxes[best_index].x, traj.boxes[best_index].y, traj.boxes[best_index].w, traj.boxes[best_index].h);
		cv::Mat face_roi = hist_frame_[start_frame + best_index](face_rect);
		return face_roi;
	}
};