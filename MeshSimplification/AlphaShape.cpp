#include "AlphaShape.h"



AlphaShape::AlphaShape()
{
}


AlphaShape::~AlphaShape()
{
}


std::vector<Point_3> AlphaShape::construct(std::set<Point_3> points, Plane_3 plane) {
	// Convert from 3D to 2D
	std::vector<Point_2> points_2d;
	for (auto point : points) {
		points_2d.push_back(plane.to_2d(point));
	}

	// Construct Alpha Shape
	Alpha_shape_2 as(points_2d.begin(), points_2d.end());

	// Find optimal alpha
	Alpha_iterator opt = as.find_optimal_alpha(1);
	as.set_alpha((*opt)*(*opt));
	as.set_mode(Alpha_shape_2::GENERAL);

	// Retrieve alpha boundary
	std::vector<Segment_2> segments;
	for (Alpha_shape_edges_iterator it = as.alpha_shape_edges_begin(); it != as.alpha_shape_edges_end(); ++it) {
		if (as.classify(*it) == Alpha_shape_2::REGULAR) { segments.push_back(as.segment(*it)); }
	}

	// Order points
	std::vector<Point_2> sorted_points;
	Segment_2 next = segments[0];
	Point_2 start = next.source();
	do {
		sorted_points.push_back(next.source());
		next = get_incident_edge(&segments, next.target());
	} while (start != next.source());

	// Convert to 3D
	std::vector<Point_3> points_3d;
	for (auto point : sorted_points) {
		points_3d.push_back(plane.to_3d(point));
	}

	return points_3d;
}


Segment_2 AlphaShape::get_incident_edge(std::vector<Segment_2>* segments, Point_2 target) {
	for (auto segment : *segments) {
		if (segment.source() == target) { return segment; }
	}
}

