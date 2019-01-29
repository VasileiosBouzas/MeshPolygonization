#pragma once

#include "Utils.h"
#include "Segment.h"
#include "Draw.h"


// Project segment faces to 2D
inline std::vector<Triangle_2> project_segment_faces(const Mesh* mesh, unsigned int id, Plane_3* plane) {
	std::vector<Triangle_2> triangles;

	std::vector<Vertex> vertices;
	VProp_geom geom = mesh->points();
	Point_2 p, q, r;

	// Iterate segment faces
	for (auto face : select_segment(mesh, id)) {
		// Retrieve face vertices
		vertices = vertex_around_face(mesh, face);

		// Project points to 2D
		p = plane->to_2d(geom[vertices[0]]);
		q = plane->to_2d(geom[vertices[1]]);
		r = plane->to_2d(geom[vertices[2]]);

		// Construct triangle
		Triangle_2 triangle(p, q, r);

		// Ensure COUNTERCLOCKWISE order
		if (triangle.orientation() == CGAL::NEGATIVE) { triangle.opposite(); }

		// Add triangle
		triangles.push_back(triangle);
	}

	return triangles;
}


// Check overlap
inline bool check_overlap(Finite_faces_iterator it, std::vector<Triangle_2>* triangles) {
	// Retrieve face vertices
	Point_2 p, q, r;
	p = it->vertex(0)->point();
	q = it->vertex(1)->point();
	r = it->vertex(2)->point();

	// Construct triangle
	Triangle_2 T(p, q, r);

	// Ensure COUNTERCLOCKWISE order
	if (T.orientation() == CGAL::NEGATIVE) { T.opposite(); }

	// Iterate segment triangles
	double area = 0;
	Polygon_2 poly;
	for (auto triangle : *triangles) {
		// Compute intersection
		auto intersection = CGAL::intersection(triangle, T);

		// Handle intersection
		if (intersection != boost::none) {
			// If triangle intersects
			if (const std::vector<Point_2>* points = boost::get<std::vector<Point_2>>(&(*intersection))) { 
				poly = Polygon_2(points->begin(), points->end());
				area += std::abs(poly.area());
			}
			// If triangle lies inside
			else if (const Triangle_2* tri = boost::get<Triangle_2>(&(*intersection))) {
				area += std::abs(tri->area());
			}
		}
	}

	if (area / std::abs(T.area()) >= 0.5) { return true; }
	return false;
}


// Retrieve alpha shape
inline std::vector<Segment_2> construct_alpha_shape(const Mesh* mesh, unsigned int id, Plane_3* plane, std::vector<Point_2>* points) {
	// Construct 2D Delaunay Triangulation
	Triangulation_2 DT(points->begin(), points->end());
	// Draw triangulation
	draw_triangulation(&DT, id);
	
	// Project segment faces to 2D triangles
	std::vector<Triangle_2> triangles = project_segment_faces(mesh, id, plane);
	// Draw segment projection
	draw_segment(&triangles, id);

	// Iterate triangulation faces
	std::set<Point_2> new_points;
	std::set<Triangulation_2::Face_handle> in_domain;
	for (Finite_faces_iterator it = DT.finite_faces_begin(); it != DT.finite_faces_end(); ++it) {
		// If face overlaps with segment, mark it in domain
		if (check_overlap(it, &triangles)) { in_domain.insert(it); }
	}

	// Iterate triangulation edges
	std::vector<Segment_2> segments;
	for (Triangulation_2::Finite_edges_iterator eit = DT.finite_edges_begin(); eit != DT.finite_edges_end(); ++eit) {
		const Triangulation_2::Face_handle& fh = eit->first;

		int ctr = 0;
		bool opposite = false;
		
		// If face in domain
		if (in_domain.find(fh) != in_domain.end()) { ctr++; }
		// If neighboring face in domain
		if (in_domain.find(fh->neighbor(eit->second)) != in_domain.end()) { ctr++; opposite = true; }
		
		// Check either face or neighboring face in domain
		// Not both of them!
		if (ctr == 1) {
			// If face, include edge
			if (opposite == false) { segments.push_back(DT.segment(*eit)); }
			// If neighboring face, include opposite edge
			else { segments.push_back(DT.segment(*eit).opposite()); }
		}
	}

	return segments;
}


