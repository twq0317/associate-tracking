
#pragma once
#include <vector>

/******************************************************************************
* STRUCTS
******************************************************************************/
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
};

struct Track
{
	std::vector<BoundingBox> boxes;
	float max_score;
	int start_frame;
	int id;
};