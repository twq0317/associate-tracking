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
#include <opencv2/opencv.hpp>

namespace auto_color
{
	cv::Scalar HSV2RGB(const float h, const float s, const float v) {
		const int h_i = static_cast<int>(h * 6);
		const float f = h * 6 - h_i;
		const float p = v * (1 - s);
		const float q = v * (1 - f * s);
		const float t = v * (1 - (1 - f) * s);
		float r, g, b;
		switch (h_i) {
		case 0:
			r = v; g = t; b = p;
			break;
		case 1:
			r = q; g = v; b = p;
			break;
		case 2:
			r = p; g = v; b = t;
			break;
		case 3:
			r = p; g = q; b = v;
			break;
		case 4:
			r = t; g = p; b = v;
			break;
		case 5:
			r = v; g = p; b = q;
			break;
		default:
			r = 1; g = 1; b = 1;
			break;
		}
		return cv::Scalar(r * 255, g * 255, b * 255);
	}

	cv::Scalar GetColor(const int n, const int max) {
		// Convert
		int converted_n;
		if (n % 2 == 0) converted_n = (n%max) / 2;
		else converted_n = max / 2 + (n%max) / 2 + 1;
		cv::Scalar color;
		cv::RNG rng(12345);
		const float golden_ratio_conjugate = 0.618033988749895;
		const float s = 0.3;
		const float v = 0.99;

		const float h = std::fmod((float)converted_n/ (float)max + golden_ratio_conjugate, 1.f);
		color = HSV2RGB(h, s, v);
		
		return color;
	}
};