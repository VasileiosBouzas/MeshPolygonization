#pragma once

#include "Utils.h"
#include "Segment.h"
#include "Draw.h"


// Project segment faces to 2D
inline std::vector<Polygon_2> project_segment_faces(const Mesh* mesh, unsigned int id, Plane_3* plane) {
	std::vector<Polygon_2> polygons;

	std::vector<Vertex> vertices;
	VProp_geom geom = mesh->points();
	std::vector<Point_2> points;

	// Iterate segment faces
	for (auto face : select_segment(mesh, id)) {
		// Retrieve face vertices
		vertices = vertex_around_face(mesh, face);

		// Project face points to 2D
		for (auto vertex : vertices) {
			points.push_back(plane->to_2d(geom[vertex]));
		}

		// Construct 2D polygon
		Polygon_2 polygon(points.begin(), points.end());

		// Ensure COUNTERCLOCKWISE order
		if (polygon.is_clockwise_oriented()) { polygon.reverse_orientation(); }

		// Add triangle
		polygons.push_back(polygon);
		points.clear();
	}

	return polygons;
}


// Check overlap
inline bool check_overlap(Polygon_2* polygon, std::vector<Polygon_2>* faces) {
	// Iterate segment faces
	double area = 0;
	Pwh_list_2 inter;
	for (auto face : *faces) {
		// Compute intersection
		//CGAL::do_intersect(*polygon, face);
		//CGAL::intersection(*polygon, face, std::back_inserter(inter));

		/*Pwh_list_2::const_iterator it;
		for (it = inter.begin(); it != inter.end(); ++it) {
			area += it->outer_boundary().area();
		}*/
	}

	if (area / std::abs(polygon->area()) >= 0.8) { return true; }
	return false;
}


// Define simplified face
inline void define_face(const Mesh* mesh, unsigned int id, Plane_3* plane, std::vector<Polygon_2>* polygons) {
	// Project segment faces to 2D polygons
	std::vector<Polygon_2> faces = project_segment_faces(mesh, id, plane);
	//draw_mesh_segment(&faces, id);

	// Check overlapping
	for (auto polygon : *polygons) {
		// If face overlaps with segment
		check_overlap(&polygon, &faces);
	}
}


