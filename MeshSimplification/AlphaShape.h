#pragma once

#include "Utils.h"

class AlphaShape
{
public:
	AlphaShape();
	~AlphaShape();

	void construct(std::vector<Point_3> points, Plane_3 plane);

};

