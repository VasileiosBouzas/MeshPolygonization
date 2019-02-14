#pragma once

#include "Utils.h"
#include "Draw.h"


// Convert 3D segments to 2D segments
inline std::vector<Segment_2> project_segments(Plane_3* plane, std::vector<Segment_3>* segments) {
	std::vector<Segment_2> segments_2d;

	// Project segment to 2D
	Point_2 source, target;
	std::vector<Point_2> existing_points;
	for (auto segment : *segments) {
		// Project source-target on plane
		source = plane->to_2d(segment.source());
		target = plane->to_2d(segment.target());

		// Check for overlaps
		int exists = 0;
		for (auto point : existing_points) {
			if (CGAL::squared_distance(source, point) < 0.01) { exists++; }
			if (CGAL::squared_distance(target, point) < 0.01) { exists++; }
		}

		// Add 2D segment
		if (exists != 2) {
			segments_2d.push_back(Segment_2(source, target));
			existing_points.push_back(source);
			existing_points.push_back(target);
		}
	}

	return segments_2d;
}


// Divide segments to subsegments
inline std::vector<Segment_2> split_segments(std::vector<Segment_2>* segments) {
	std::vector<Segment_2> sub_segments;

	// Compute segment intersections
	std::list<Point_2> segment_points;
	std::vector<Point_2> existing_points;
	Point_2 source, target;
	for (auto segment_1 : *segments) {
		for (auto segment_2 : *segments) {
			// Compute intersection
			auto intersection = CGAL::intersection(segment_1, segment_2);

			// Handle intersection
			bool not_exists = true;
			if (intersection != boost::none) {
				if (const Point_2* point = boost::get<Point_2>(&(*intersection))) {
					// Check distance to existing intersections
					for (auto pt : existing_points) {
						if (CGAL::squared_distance(*point, pt) < 1) {
							segment_points.push_back(pt);
							not_exists = false; break;
						}
					}

					// If not existing
					if (not_exists) {
						segment_points.push_back(*point);
						existing_points.push_back(*point);
					}
				}
			}
		}

		// Sort segment points by distance to source
		source = segment_1.source();
		target = segment_1.target();
		segment_points.sort([&source](const Point_2 &pt1, const Point_2 &pt2) {
			if (CGAL::squared_distance(source, pt1) < CGAL::squared_distance(source, pt2))
				return pt1 < pt2;
			return pt2 > pt1;
		});

		// Construct subsegments
		sub_segments.push_back(Segment_2(source, segment_points.front()));
		for (std::list<Point_2>::iterator it = segment_points.begin(); it != segment_points.end(); ++it) {
			if (std::next(it, 1) == segment_points.end()) { break; }
			sub_segments.push_back(Segment_2(*it, *std::next(it, 1)));
		}
		sub_segments.push_back(Segment_2(segment_points.back(), target));

		/*std::cout << "SUB-SEGMENTS" << std::endl;
		for (auto segment : sub_segments) {
			std::cout << segment.source() << ", " << segment.target() << std::endl;
		}*/

		// Clear
		segment_points.clear();
	}

	return sub_segments;
}


inline void segments_to_polygons(Plane_3* plane, std::vector<Segment_3>* segments, unsigned int id) {
	// Project segments on plane
	std::vector<Segment_2> segments_2d = project_segments(plane, segments);

	for (auto segment_1 : segments_2d) {
		for (auto segment_2 : segments_2d) {
			if (segment_1 != segment_2) {
				if (CGAL::squared_distance(segment_1.source(), segment_2.source()) < 0.01 &&
					CGAL::squared_distance(segment_1.target(), segment_2.target()) < 0.01)
					std::cout << "FOUND" << std::endl;
			}
		}
	}

	//std::vector<Segment_2> sub_segments = split_segments(&segments_2d);
}
