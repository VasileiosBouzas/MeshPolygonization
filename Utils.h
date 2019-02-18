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

// Face around target circulator
inline std::vector<Face> face_around_vertex(const Mesh* mesh, Vertex vertex) {
	std::vector<Face> faces;
	Face face;

	Face_around_target_circulator fbegin(mesh->halfedge(vertex), *mesh), done(fbegin);
	do {
		face = *fbegin++;
		if (face != mesh->null_face()) {faces.push_back(face);}
	} while (fbegin != done);

	return faces;
}
// CIRCULATORS //


// GENERAL FUNCTIONS //
// K-Ring Neighboring Vertices (Vertex)
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

// K-Ring Neighboring Faces (Vertex)
inline std::set<Face> get_k_ring_faces(const Mesh* mesh, const Vertex vertex, unsigned int k) {
	std::set<Vertex> neighbors;
	std::vector<Face> new_faces;
	std::set<Face> faces;

	if (k == 1) {
		new_faces = face_around_vertex(mesh, vertex);
		faces.insert(new_faces.begin(), new_faces.end());
	} else {
		neighbors = get_k_ring_vertices(mesh, vertex, k - 1);
		for (auto neighbor : neighbors) {
			// Face around target circulator
			new_faces = face_around_vertex(mesh, neighbor);
			faces.insert(new_faces.begin(), new_faces.end());
		}
	}

	return faces;
}

// K-Ring Neighboring Faces (Face)
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
inline Plane_3 fit_plane_to_faces(const Mesh* mesh, std::set<Face>* faces) {
	std::vector<Vertex> vertices;
	std::vector<Point_3> points;
	VProp_geom geom = mesh->points();

	// Retrieve geometry for points of current region
	for (auto face : *faces) {
		vertices = vertex_around_face(mesh, face);
		for (auto vertex : vertices) {
			points.push_back(geom[vertex]);
		}
	}

	// Calculate plane
	Plane_3 plane;
	linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

	return plane;
}

// Point in Bbox
inline bool is_in_bbox(const Bbox_3* bbox, const Point_3* pt) {
	// Extend bbox
	double offset = 1e-04; // Arithmetic precision
	double xmin = bbox->xmin() - offset;
	double xmax = bbox->xmax() + offset;
	double ymin = bbox->ymin() - offset;
	double ymax = bbox->ymax() + offset;
	double zmin = bbox->zmin() - offset;
	double zmax = bbox->zmax() + offset;

	if (pt->x() >= xmin && pt->x() <= xmax) {
		if (pt->y() >= ymin && pt->y() <= ymax) {
			if (pt->z() >= zmin && pt->z() <= zmax) {
				return true;
			}
		}
	}

	return false;
}

// Bbox Planes
inline std::vector<Plane_3> compute_bbox_planes(const Bbox_3* bbox) {
	std::vector<Plane_3> planes;

	// Define bbox vertices
	Point_3 p0(bbox->xmin(), bbox->ymin(), bbox->zmin()); // 0
	Point_3 p1(bbox->xmin(), bbox->ymax(), bbox->zmin()); // 1
	Point_3 p2(bbox->xmin(), bbox->ymin(), bbox->zmax()); // 2
	Point_3 p3(bbox->xmin(), bbox->ymax(), bbox->zmax()); // 3
	Point_3 p4(bbox->xmax(), bbox->ymin(), bbox->zmin()); // 4
	Point_3 p5(bbox->xmax(), bbox->ymax(), bbox->zmin()); // 5
	Point_3 p6(bbox->xmax(), bbox->ymin(), bbox->zmax()); // 6
	Point_3 p7(bbox->xmax(), bbox->ymax(), bbox->zmax()); // 7

	// Construct planes
	planes.push_back(Plane_3(p0, p1, p4)); // XY-min (zmin)
	planes.push_back(Plane_3(p2, p3, p6)); // XY-max (zmax)
	planes.push_back(Plane_3(p0, p2, p4)); // XZ-min (ymin)
	planes.push_back(Plane_3(p1, p3, p5)); // XZ-max (ymax)
	planes.push_back(Plane_3(p0, p1, p2)); // YZ-min (xmin)
	planes.push_back(Plane_3(p4, p5, p6)); // YZ-max (xmax)

	return planes;
}