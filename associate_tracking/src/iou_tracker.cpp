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
#include "iou_tracker.h"
#include <vector>
#include <algorithm>
#include <assert.h>

namespace iou_tracker
{
	IOUTracker::IOUTracker(float sigma_l, float sigma_h, float sigma_iou, float t_min, float t_max)
	{
		Initialize(sigma_l, sigma_h, sigma_iou, t_min, t_max);
	}

	void IOUTracker::Initialize(float sigma_l, float sigma_h, float sigma_iou, float t_min, float t_max)
	{
		sigma_l_ = sigma_l;
		sigma_h_ = sigma_h;
		sigma_iou_ = sigma_iou;
		t_min_ = t_min;
		t_max_ = t_max;

		frame_counter_ = 0;
		id_counter_ = 0;

		active_trajectorys_.clear();
		long_traj_ids_.clear();

		initialized_ = true;
	}

	float IOUTracker::intersectionOverUnion(BoundingBox box1, BoundingBox box2)
	{
		float minx1 = box1.x;
		float maxx1 = box1.x + box1.w;
		float miny1 = box1.y;
		float maxy1 = box1.y + box1.h;

		float minx2 = box2.x;
		float maxx2 = box2.x + box2.w;
		float miny2 = box2.y;
		float maxy2 = box2.y + box2.h;

		if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
			return 0.0f;
		else
		{
			float dx = std::min(maxx2, maxx1) - std::max(minx2, minx1);
			float dy = std::min(maxy2, maxy1) - std::max(miny2, miny1);
			float area1 = (maxx1 - minx1) * (maxy1 - miny1);
			float area2 = (maxx2 - minx2) * (maxy2 - miny2);
			float inter = dx * dy; // Intersection
			float uni = area1 + area2 - inter; // Union
			float IoU = inter / uni;
			return IoU;
		}
		//	return 0.0f;
	}

	int IOUTracker::highestIOU(BoundingBox box, std::vector<BoundingBox> boxes)
	{
		float iou = 0, highest = 0;
		int index = -1;
		for (int i = 0; i < boxes.size(); i++)
		{
			if (boxes[i].score < sigma_l_) continue;
			iou = intersectionOverUnion(box, boxes[i]);
			if (iou >= highest)
			{
				highest = iou;
				index = i;
			}
		}
		return index;
	}

	std::vector<Trajectory>  IOUTracker::Track1Frame(std::vector<BoundingBox> det_boxes)
	{
		assert(initialized_);
		std::vector<Trajectory> result;
		
		// Update active tracks
		for (int i = 0; i < active_trajectorys_.size(); i++)
		{
			Trajectory track = active_trajectorys_[i];

			// Get the index of the detection with the highest IOU
			int index = highestIOU(track.boxes.back(), det_boxes);
			// Check is above the IOU threshold
//			std::cout << " --- IOU Best match = " << intersectionOverUnion(track.boxes.back(), frameBoxes[index]) << std::endl;
			if (index != -1 && intersectionOverUnion(track.boxes.back(), det_boxes[index]) >= sigma_iou_)
			{
				track.boxes.push_back(det_boxes[index]);

				if (track.max_score < det_boxes[index].score)
					track.max_score = det_boxes[index].score;
				// Remove the best matching detection from the frame detections
				det_boxes.erase(det_boxes.begin() + index);
				active_trajectorys_[i] = track;
			}
			// If the track was not updated...
			else
			{
				// Check the conditions to finish the track
				if (track.max_score >= sigma_h_ && track.boxes.size() >= t_min_)
				{
					//track.id = id_counter_++;
					result.push_back(track);
				}
				active_trajectorys_.erase(active_trajectorys_.begin() + i);
				// Workaround used because of the previous line "erase" call
				i--;
			}
		} // End for active tracks

		// Delete the overlapping tracks. We don't need uncertain trajectory.
		for (int i = 0; i < active_trajectorys_.size(); i++)
		{
			Trajectory track1 = active_trajectorys_[i];
			for (int j = i + 1; j < active_trajectorys_.size(); j++)
			{
				Trajectory track2 = active_trajectorys_[j];
				float iou = intersectionOverUnion(track1.boxes.back(), track2.boxes.back());
				if (iou > 0.01)
				{
					// Check the conditions to finish the track
					if (track1.max_score >= sigma_h_ && track1.boxes.size() >= t_min_)
					{
						//track.id = id_counter_++;
						result.push_back(track1);
					}
					if (track2.max_score >= sigma_h_ && track2.boxes.size() >= t_min_)
					{
						//track.id = id_counter_++;
						result.push_back(track2);
					}
					active_trajectorys_.erase(active_trajectorys_.begin() + j);
					active_trajectorys_.erase(active_trajectorys_.begin() + i);
					j--;
					i--;
				}
			}
		} // End for deleting overlap tracks

		// Delete the tracks over time.
		for (int i = 0; i < active_trajectorys_.size(); i++)
		{
			Trajectory track = active_trajectorys_[i];
			if (track.boxes.size() > t_max_)
			{
				// Check the conditions to finish the track
				if (track.max_score >= sigma_h_)
				{
					//track.id = id_counter_++;
					result.push_back(track);
				}
				active_trajectorys_.erase(active_trajectorys_.begin() + i);
				i--;
			}
		} // End for deleting

		// Create new tracks
		for (auto box : det_boxes)
		{
			// Skip the detections with a too low score.
			if (box.score < sigma_l_) continue;
			// Regist new trajectory for each bounding box.
			std::vector<BoundingBox> b;
			b.push_back(box);
			Trajectory t = { b, box.score, frame_counter_, id_counter_++ };
			// Trajectory t = { b, box.score, frame_counter_, 0 };		// To prevent generating too many ids, we try to generate id when track finish.
			active_trajectorys_.push_back(t);
		} // End for creating

		frame_counter_++;
		return result;
	}

	std::vector<Trajectory> IOUTracker::TrackLastFrame(std::vector<BoundingBox> det_boxes)
	{
		assert(initialized_);
		std::vector<BoundingBox> empty;
		//std::vector<Trajectory> result = Track1Frame(det_boxes);
		std::vector<Trajectory> result2 = Track1Frame(empty);
		//if (result.empty()) result = result2;
		//else if (result2.empty()) result = result;
		//else result.insert(result.end(), result2.begin(), result2.end());

		return result2;
	}
};