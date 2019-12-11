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
#include <vector>
#include <opencv2/opencv.hpp>
#include "iou_tracker.h"

namespace iou_tracker
{

	class TrajectoryFilter
	{
	public:
		TrajectoryFilter() : initialized_(false) {}
		TrajectoryFilter(float sigma_det_score, float sigma_angle_conf, float sigma_roll, float sigma_pitch, float sigma_yaw, float sigma_blur);
		void Initialize(float sigma_det_score, float sigma_angle_conf, float sigma_roll, float sigma_pitch, float sigma_yaw, float sigma_blur);
		virtual ~TrajectoryFilter() = default;
		void StoreFrame(cv::Mat image_orig, long frame_count);
		cv::Mat GetMostCenteredFace(Trajectory traj);
	
	private:

	private:
		bool initialized_;
		std::map<long, cv::Mat> hist_frame_;

		// Filter parameters
		float sigma_det_score_;
		float sigma_angle_conf_;
		float sigma_roll_;
		float sigma_pitch_;
		float sigma_yaw_;
		float sigma_blur_;
	};

};