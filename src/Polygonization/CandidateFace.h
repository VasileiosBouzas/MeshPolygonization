#pragma once

#include "Utils.h"


// Convert 3D segments to 2D segments
inline std::vector<Segment_2> project_segments(Plane_3* plane, std::vector<Plane_intersection>* edges, std::vector<int>* plane_edges) {
	std::vector<Segment_2> segments;

	// Project segment to 2D
	Segment_3 segment;
	Point_2 source, target;
	for (auto i : *plane_edges) {
		// Retrieve geometry
		segment = (*edges)[i].segment;

		// Project source-target on plane
		source = plane->to_2d(segment.source());
		target = plane->to_2d(segment.target());

		// Add 2D segment
		segments.push_back(Segment_2(source, target));
	}

	return segments;
}


// Construct simple polygons
inline std::vector<Candidate_face> define_faces(unsigned int id, std::vector<Segment_2>* segments, std::vector<Plane_intersection>* edges, std::vector<int>* plane_edges, Plane_3* plane) {
	std::vector<Candidate_face> candidate_faces;

	// Construct 2D arrangement
	Arrangement_2 arr;
	CGAL::insert_non_intersecting_curves(arr, segments->begin(), segments->end());

	// Iterate arrangement faces
	for (Face_iterator f = arr.faces_begin(); f != arr.faces_end(); ++f) {
		// Check if face is unbounded
		if (f->is_unbounded()) { continue; }

		// Define candidate face
		std::vector<Point_2> points;
		Candidate_face candidate_face;

		// Traverse outer boundary of face
		Ccb_halfedge_circulator cc = f->outer_ccb();
		do {
			// Retrieve curve
			Point_2 source = cc->curve().source();
			Point_2 target = cc->curve().target();
			Segment_2 curve(source, target);

			// Find original segment
			int pos;
			for (auto i = 0; i < segments->size(); i++) {
				Segment_2 segment = (*segments)[i];
				if (curve == segment) { pos = i; break;}
			}

			// Retrieve edge
			auto e = (*plane_edges)[pos];
			// Add to face
			candidate_face.edges.push_back(e);

			// Check curve orientation to original segment
			bool curve_has_same_direction = (cc->curve().source() == cc->source()->point());

			// Add vertices to face (CCW)
			int vertex;
			// If same direction
			if (curve_has_same_direction) {
				// Add edge source
				vertex = (*edges)[e].vertices[0];
				points.push_back(curve.source());
			}
			// If opposite
			else {
				// Add edge target
				vertex = (*edges)[e].vertices[1];
				points.push_back(curve.target());
			}
			candidate_face.vertices.push_back(vertex);

		} while (++cc != f->outer_ccb());

		// Assign 2D projection
		candidate_face.polygon = Polygon_2(points.begin(), points.end());

		// Assign supporting plane to face
		candidate_face.plane = id;

		// Update
		candidate_faces.push_back(candidate_face);
	}

	return candidate_faces;
}


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


// Compute confidence
inline std::pair<std::size_t, double> compute_confidence(Polygon_2* polygon, std::vector<Polygon_2>* faces) {
	std::size_t num = 0;
	double area = 0;

	// Iterate segment faces
	for (auto face : *faces) {
		// Check for bbox overlap
		if (do_overlap(polygon->bbox(), face.bbox())) {
			bool is_inside = true;
			std::vector<Point_2> points;

			// Check if face is inside polygon
			Polygon_2::Vertex_const_iterator v;
			// For every face vertex
			for (v = face.vertices_begin(); v != face.vertices_end(); ++v) {
				// Check its position with respect to polygon
				auto check = CGAL::bounded_side_2(polygon->vertices_begin(), polygon->vertices_end(), *v);

				// If it is inside or on the boundary, store it
				if (check == CGAL::ON_BOUNDED_SIDE || check == CGAL::ON_BOUNDARY) { points.push_back(*v); }

				// If a vertex is out of polygon, mark whole face as outside
				else { is_inside = false; }
			}

			// If face is inside
			if (is_inside) {
				// Include in total area
				num++; // Mark the face as supporting
				area += std::abs(face.area());
			}
			// If face intersects (at least one vertex is inside polygon)
			else {
				// Compute edge intersections of polygon and face
				Polygon_2::Edge_const_iterator ep, ef;
				// Polygon edge iterator
				for (ep = polygon->edges_begin(); ep != polygon->edges_end(); ++ep) {
					// Face edge iterator
					for (ef = face.edges_begin(); ef != face.edges_end(); ++ef) {
						// Compute intersection
						auto intersection = CGAL::intersection(*ep, *ef);

						// Handle intersection
						if (intersection != boost::none) {
							if (const Point_2* pt = boost::get<Point_2>(&(*intersection))) {
								// Store with existent points
								points.push_back(*pt);
							}
						}
					}
				}

				// Order points (either CW or CCW)
				std::vector<Point_2> ordered;
				CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(ordered));

				// Construct polygon and include in total area
				Polygon_2 pol(ordered.begin(), ordered.end());
				area += std::abs(pol.area());
			}
		}
	}

	return std::make_pair(num, area);
}


inline std::vector<Candidate_face> compute_candidate_faces(const Mesh* mesh, unsigned int id, Plane_3* plane, std::vector<Plane_intersection>* edges, std::vector<int>* plane_edges) {
	// Project segments on plane
	std::vector<Segment_2> segments = project_segments(plane, edges, plane_edges);

	// Construct simple polygons
	std::vector<Candidate_face> candidate_faces = define_faces(id, &segments, edges, plane_edges, plane);

	// Project segment faces to 2D polygons
	std::vector<Polygon_2> faces = project_segment_faces(mesh, id, plane);

	// Compute face confidences
	std::vector<Polygon_2> polygons;
	for (auto i = 0; i < candidate_faces.size(); i++) {
		Polygon_2 polygon = candidate_faces[i].polygon;
		polygons.push_back(polygon);
		auto pair = compute_confidence(&polygon, &faces);

		// Number of supporting faces
		candidate_faces[i].supporting_face_num = pair.first;

		// Covered area
		candidate_faces[i].covered_area = pair.second;

		// Total area
		candidate_faces[i].area = polygon.area();
	}

	return candidate_faces;
}