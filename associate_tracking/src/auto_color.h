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