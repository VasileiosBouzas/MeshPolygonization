#pragma once

#include "Utils.h"

inline std::vector<Point_2> compute_line_intersections(Plane_3* plane, std::vector<Line_3>* lines) {
	std::vector<Point_2> points;

	// Compute line intersections
	for (auto curr_line : *lines) {
		for (auto other_line : *lines) {
			// Compute intersecting point
			if (curr_line == other_line) { continue; }
			auto intersection = CGAL::intersection(curr_line, other_line);

			// Handle intersection
			if (intersection != boost::none) {
				std::cout << "INTERSECTION!" << std::endl;
				if (const Point_3* pt = boost::get<Point_3>(&(*intersection))) { 
					std::cout << *pt << std::endl; 
				}
			}
		}
	}

	return points;
}
