#pragma once

#include "Utils.h"
#include "TriangulationOverlap.h"


// Retrieve incident edge
inline Segment_2 get_incident_edge(std::vector<Segment_2>* segments, Point_2 target) {
	for (auto segment : *segments) {
		if (segment.source() == target) { return segment; }
	}
}


// Construct AlphaShape
inline std::vector<Point_3> AlphaShape(const Mesh* mesh, unsigned int id, Plane_3* plane, std::vector<Point_3>* points) {
	//// Construct Alpha Shape
	//Alpha_shape_2 as(points_2d.begin(), points_2d.end());

	//// Find optimal alpha
	//Alpha_iterator opt = as.find_optimal_alpha(1);
	//as.set_alpha((*opt)*2.0f);
	//as.set_mode(Alpha_shape_2::REGULARIZED);

	//// Retrieve alpha boundary
	//std::vector<Segment_2> segments;
	//for (Alpha_shape_edges_iterator it = as.alpha_shape_edges_begin(); it != as.alpha_shape_edges_end(); ++it) {
	//	if (as.classify(*it) == Alpha_shape_2::REGULAR) { segments.push_back(as.segment(*it)); }
	//}
	
	//std::cout << "Segment " << id << std::endl;
	///*for (auto segment : segments) {
	//	std::cout << segment.source() << ", " << segment.target() << std::endl;
	//}
	//std::cout << std::endl;*
	
	// Convert from 3D to 2D
	std::vector<Point_2> points_2d;
	for (auto point : *points) {
		points_2d.push_back(plane->to_2d(point));
	}

	// Construct alpha shape
	std::vector<Segment_2> segments = construct_alpha_shape(mesh, id, plane, &points_2d);

	// Draw alpha shape
	draw_alpha_shape(&segments, id);

	// Order points
	std::vector<Point_2> sorted_points;
	if (segments.size() > 0) {
		Segment_2 next = segments[0];
		Point_2 start = next.source();
		do {
			sorted_points.push_back(next.source());
			next = get_incident_edge(&segments, next.target());
		} while (start != next.source());
	}

	// Convert to 3D
	std::vector<Point_3> points_3d;
	for (auto point : sorted_points) {
		points_3d.push_back(plane->to_3d(point));
	}

	return points_3d;
}


