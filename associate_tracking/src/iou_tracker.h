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

namespace iou_tracker
{
	struct BoundingBox
	{
		// x-component of top left coordinate
		float x;
		// y-component of top left coordinate
		float y;
		// width of the box
		float w;
		// height of the box
		float h;
		// score of the box;
		float score;
		// key point information
		float left_eye_x;
		float left_eye_y;
		float right_eye_x;
		float right_eye_y;
		float nose_x;
		float nose_y;
		float mouse_x;
		float mouse_y;
		float pitch;
		float roll;
		float yaw;
		float angle_confidence;

		// blur info
		float blur;
	};

	struct Trajectory
	{
		std::vector<BoundingBox> boxes;
		float max_score;
		int start_frame;
		int id;
	};

	class FrameCounter
	{
	public:
		FrameCounter() : counter_(0) {}
		FrameCounter(int init_count) : counter_(init_count) {}
		void setCounter(int count) { counter_ = count; }
		FrameCounter operator ++ (int)	{
			FrameCounter ret(counter_);
			counter_++;
			return counter_;
		}
		long count() { return counter_; }

	private:
		long counter_;
	};

	class IOUTracker
	{
	public:
		IOUTracker() : initialized_(false) {}
		IOUTracker(FrameCounter* counter_ptr, float sigma_l, float sigma_h,
							float sigma_iou, float t_min, float t_max, float inflact_ratio,
							float width, float height);
		virtual ~IOUTracker() = default;

		// The param explaination could be seen at end of the file.
		void Initialize(FrameCounter* counter_ptr, float sigma_l, float sigma_h,
							float sigma_iou,	 float t_min,	float t_max, float inflact_ratio,	
							float width, float height);

		// Input: detection boxes of current frame.
		// Output: several tracking trajectorys.
		// This function must be called at every frame.
		std::vector<Trajectory> Track1Frame(std::vector<BoundingBox> det_boxes);

		// Input: detection boxes of current frame.
		// Output: several tracking trajectorys.
		// This function is used at the last frame. The active trajectory will all be finished.
		std::vector<Trajectory> TrackLastFrame(std::vector<BoundingBox> det_boxes);

		// Used to display the tracking result.
		std::vector<Trajectory> getActiveTrajectorys() { return active_trajectorys_; };

		long getFrameCounter(){ return frame_counter_ptr_->count(); }

	private:
		// Return the IoU between two boxes
		float intersectionOverUnion(BoundingBox box1, BoundingBox box2);
		// Returns the index of the bounding box with the highest IoU
		int highestIOU(BoundingBox box, std::vector<BoundingBox> boxes);
		// Inflact input boxes for more iou
		void InflactBoxes(std::vector<iou_tracker::BoundingBox>& boxes);
	
	private:
		bool initialized_;	// Keeps track about the correct initialization of this class
		// long frame_counter_;	// Keeps counting the frame number.
		FrameCounter* frame_counter_ptr_;	// Keeps counting the frame number. Use ptr to keep consistence of different object.
		long id_counter_;	 // Keeps counting for generating the right id number for each trajectory.
		std::vector<Trajectory> active_trajectorys_;	
		std::vector<long> long_traj_ids_;	// Preserves ids of the trajectorys that exceeded the t_max.

		// Tracking parameteres
		float sigma_l_;	// low detection threshold
		float sigma_h_;	// high detection threshold
		float sigma_iou_;	// IOU threshold
		float t_min_;	// minimum track length in frames
		float t_max_;	// maximum track length in frames

		float frame_width_;
		float frame_height_;
		float box_inflact_ratio_; // Inflact input dection boxes
	};
};