#pragma once

#include "Utils.h"


// Select segment by id
inline std::set<Face> select_segment(const Mesh* mesh, unsigned int id) {
	std::set<Face> segment;

	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	for (auto face : mesh->faces()) {
		if (chart[face] == id) { segment.insert(face); }
	}

	return segment;
}


// Retrieve segment vertices
inline std::set<Vertex> get_segment_vertices(const Mesh* mesh, unsigned int id) {
	std::set<Vertex> vertices;

	// Select segment by id
	std::set<Face> segment = select_segment(mesh, id);

	// Iterate faces
	std::vector<Vertex> new_vertices;
	for (auto face : segment) {
		// Retrieve face vertices
		new_vertices = vertex_around_face(mesh, face);
		vertices.insert(new_vertices.begin(), new_vertices.end());
	}

	return vertices;
}


//// Compute segment bbox
//inline Bbox_3 get_segment_bbox(const Mesh* mesh, unsigned int id) {
//	// Retrieve segmet vertices
//}


// Retrieve boundary points
inline std::vector<Point_3> get_boundary_points(const Mesh* mesh, unsigned int id) {
	std::vector<Point_3> boundary;

	// Retrieve segment vertices
	std::set<Vertex> vertices = get_segment_vertices(mesh, id);

	// Retrieve points
	std::vector<Face> neighbors;
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	VProp_geom geom = mesh->points();
	for (auto vertex : vertices) {
		bool is_out = false;

		// Retrieve neighboring vertices
		neighbors = face_around_vertex(mesh, vertex);

		// Check neighbors
		for (auto neighbor : neighbors) {
			if (chart[neighbor] != id) { is_out = true; break; }
		}

		// Add point
		if (is_out) { boundary.push_back(geom[vertex]); }
	}

	return boundary;
}
