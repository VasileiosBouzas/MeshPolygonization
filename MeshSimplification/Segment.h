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


// Retrieve segment edges
inline std::vector<Segment_3> get_segment_edges(const Mesh* mesh, unsigned int id) {
	std::vector<Segment_3> edges;

	// Select segment by id
	std::set<Face> segment = select_segment(mesh, id);

	// Iterate faces
	Point_3 source, target;
	VProp_geom geom = mesh->points();
	for (auto face : segment) {
		Halfedge hf = mesh->halfedge(face);
		for (Halfedge h : halfedges_around_face(hf, *mesh)) {
			source = geom[mesh->source(h)];
			target = geom[mesh->target(h)];
			edges.push_back(Segment_3(source, target));
		}
	}

	return edges;
}


// Retrieve interior points
inline std::vector<Point_3> get_interior_points(const Mesh* mesh, unsigned int id) {
	std::vector<Point_3> interior;

	// Retrieve segment vertices
	std::set<Vertex> vertices = get_segment_vertices(mesh, id);

	// Retrieve points
	std::vector<Face> neighbors;
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	VProp_geom geom = mesh->points();
	for (auto vertex : vertices) {
		bool is_in = true;

		// Retrieve neighboring vertices
		neighbors = face_around_vertex(mesh, vertex);

		// Check neighbors
		for (auto neighbor : neighbors) {
			if (chart[neighbor] != id) { is_in = false; break; }
		}

		// Add point
		if (is_in) { interior.push_back(geom[vertex]); }
	}

	return interior;
}
