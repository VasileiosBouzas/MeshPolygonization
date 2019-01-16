#pragma once

#include "Utils.h"

class AlphaShape
{
public:
	AlphaShape();
	~AlphaShape();

	std::vector<Point_3> construct(std::set<Point_3> points, Plane_3 plane);

private:
	Segment_2 get_incident_edge(std::vector<Segment_2>* segments, Point_2 target);

};

