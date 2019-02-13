#pragma once

#include "Utils.h"

// Convert 3D segments to 2D segments
inline std::vector<Segment_2> project_segments(Plane_3* plane, std::vector<Segment_3>* segments) {
	std::vector<Segment_2> segments_2d;

	// Project segment to 2D
	Point_2 source, target;
	for (auto segment : *segments) {
		// Project source-target on plane
		source = plane->to_2d(segment.source());
		target = plane->to_2d(segment.target());

		// Add 2D segment
		segments_2d.push_back(Segment_2(source, target));
	}

	return segments_2d;
}


inline void segments_to_polygons(Plane_3* plane, std::vector<Segment_3>* segments) {
	// Project segments on plane
	std::vector<Segment_2> segments_2d = project_segments(plane, segments);

}
