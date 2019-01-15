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
	as.set_mode(Alpha_shape_2::REGULARIZED);

	// Retrieve alpha boundary
	std::vector<Segment_2> segments;
	for (Alpha_shape_edges_iterator it = as.alpha_shape_edges_begin(); it != as.alpha_shape_edges_end(); ++it) {
		segments.push_back(as.segment(*it));
	}

	// Boundary segment map
	std::map<Point_2, Point_2> boundary;
	for (auto segment : segments) {
		boundary[segment.vertex(1)] = segment.vertex(2);
	}

	// Order boundary vertices
	std::vector<Point_2> sorted_points;
	Point_2 start = boundary.begin()->first, end;
	while (boundary.size() > 0) {
		sorted_points.push_back(start);
		end = boundary[start];
		boundary.erase(start);
		start = end;
	}

	// Convert to 3D
	std::vector<Point_3> points_3d;
	for (auto point : sorted_points) {
		points_3d.push_back(plane.to_3d(point));
	}

	return points_3d;
}
