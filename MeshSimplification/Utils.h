#pragma once

#include "CGALTypes.h"


// CIRCULATORS //
// Vertex around face circulator
inline std::vector<Vertex> vertex_around_face(const Mesh* mesh, Face face) {
	std::vector<Vertex> vertices;

	Halfedge hf = mesh->halfedge(face);
	for (Halfedge h : halfedges_around_face(hf, *mesh)) {
		Vertex vertex = target(h, *mesh);
		vertices.push_back(vertex);
	}

	return vertices;
}
// CIRCULATORS //


// GENERAL FUNCTIONS //
// K-Ring Neighboring Vertices
inline std::set<Vertex> get_k_ring_vertices(const Mesh* mesh, const Vertex vertex, unsigned int k) {
	std::set<Vertex> neighbors, new_neighbors;
	Vertex neighbor;

	neighbors.insert(vertex);
	for (unsigned int i = 1; i < k + 1; i++) {
		for (auto neighbor : neighbors) {
			// Vertex_around_target_circulator
			Vertex_around_target_circulator vbegin(mesh->halfedge(neighbor), *mesh), done(vbegin);
			do {
				neighbor = *vbegin++;
				new_neighbors.insert(neighbor);
			} while (vbegin != done);
		}

		// Update neighbors
		neighbors.insert(new_neighbors.begin(), new_neighbors.end());

		// Clear unordered_set
		new_neighbors.clear();
	}

	return neighbors;
}

// K-Ring Neighboring Faces
inline std::set<Face> get_k_ring_faces(const Mesh* mesh, const Face face, unsigned int k) {
	std::set<Face> neighbors, new_neighbors;
	Face opp_face;

	neighbors.insert(face);
	for (unsigned int i = 1; i < k + 1; i++) {
		for (auto neighbor : neighbors) {
			Halfedge_around_face_circulator hbegin(mesh->halfedge(neighbor), *mesh), done(hbegin);
			do {
				opp_face = mesh->face(mesh->opposite(*hbegin++));
				if (opp_face != mesh->null_face()) { new_neighbors.insert(opp_face); }
			} while (hbegin != done);
		}

		// Update neighbors
		neighbors.insert(new_neighbors.begin(), new_neighbors.end());

		// Clear unordered_set
		new_neighbors.clear();
	}

	return neighbors;
}

// Fit plane to faces
inline Plane fit_plane_to_faces(const Mesh* mesh, std::set<Face>* faces) {
	std::vector<Vertex> vertices;
	std::vector<Point> points;
	VProp_geom geom = mesh->points();

	// Retrieve geometry for points of current region
	for (auto face : *faces) {
		vertices = vertex_around_face(mesh, face);
		for (auto vertex : vertices) {
			points.push_back(geom[vertex]);
		}
	}

	// Calculate plane
	Plane plane;
	linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

	return plane;
}
// GENERAL FUNCTIONS //