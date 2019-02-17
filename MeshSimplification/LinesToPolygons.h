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
	Point_2 source, target;
	for (auto segment_1 : *segments) {
		for (auto segment_2 : *segments) {
			// Compute intersection
			auto intersection = CGAL::intersection(segment_1, segment_2);

			// Handle intersection
			if (intersection != boost::none) {
				if (const Point_2* point = boost::get<Point_2>(&(*intersection))) {
						segment_points.push_back(*point);
				}
			}
		}

		// Sort segment points by distance to source
		source = segment_1.source();
		target = segment_1.target();
		segment_points.sort([&](const Point_2 &pt1, const Point_2 &pt2) {
			return CGAL::squared_distance(source, pt1) < CGAL::squared_distance(source, pt2);
		});

		// Construct subsegments
		for (auto point : segment_points) {
			sub_segments.push_back(Segment_2(source, point));
			source = point;
		}
		sub_segments.push_back(Segment_2(source, target));

		// Clear
		segment_points.clear();
	}

	return sub_segments;
}


inline std::vector<Segment_2> filter_segments(std::vector<Segment_2>* segments) {
	std::vector<Segment_2> filtered;

	Point_2 source, target;
	for (auto segment_1 : *segments) {
		source = segment_1.source();
		target = segment_1.target();

		bool source_connected = false;
		bool target_connected = false;
		for (auto segment_2: *segments) {
			if (segment_1 == segment_2) { continue; }
			if (segment_2.source() == source || segment_2.target() == source) { source_connected = true; }
			if (segment_2.source() == target || segment_2.target() == target) { target_connected = true; }
			if (source_connected && target_connected) { filtered.push_back(segment_1); break; }
		}
	}

	return filtered;
}


inline void segments_to_polygons(Plane_3* plane, std::vector<Segment_3>* segments, unsigned int id) {
	// Project segments on plane
	std::vector<Segment_2> segments_2d = project_segments(plane, segments);

	// Split segments into subsegments
	std::vector<Segment_2> sub_segments = split_segments(&segments_2d);
	
	// Construct 2D arrangement
	Arrangement_2 arr;
	CGAL::insert_non_intersecting_curves(arr, sub_segments.begin(), sub_segments.end());

	// Iterate arrangement faces
	X_monotone_curve_2 cv;
	std::vector<Point_2> points;
	for (Face_iterator f = arr.faces_begin(); f != arr.faces_end(); ++f) {
		// Check if face is unbounded
		if (f->is_unbounded()) { continue; }

		// Traverse outer boundary of face
		Ccb_halfedge_circulator cc = f->outer_ccb();
		do {
			// Retrieve original curve
			cv = cc->curve();

			// Check curve orientation to original segment
			bool curve_has_same_direction = (cc->curve().source() == cc->source()->point());

			// If same orientation
			if (curve_has_same_direction) { points.push_back(cv.source()); } 
			else { points.push_back(cv.target()); }

		} while (++cc != f->outer_ccb());
	}

	std::cout << std::endl;
}
