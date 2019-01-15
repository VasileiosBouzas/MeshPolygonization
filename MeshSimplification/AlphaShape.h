#pragma once

#include "Utils.h"

class AlphaShape
{
public:
	AlphaShape();
	~AlphaShape();

	std::vector<Point_3> construct(std::set<Point_3> points, Plane_3 plane);

};

