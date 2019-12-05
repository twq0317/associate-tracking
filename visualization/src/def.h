
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
};

struct Track
{
	std::vector<BoundingBox> boxes;
	float max_score;
	int start_frame;
	int id;
};