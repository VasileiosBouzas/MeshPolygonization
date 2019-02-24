#pragma once

#include "Utils.h"
#include "Segment.h"
#include "LinesToPolygons.h"
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

	if (area / std::abs(polygon->area()) >= 0.4) { return true; }
	return false;
}


// Simplify polygon
inline std::vector<Point_2> simplify_polygon(Polygon_2* polygon) {
	std::vector<Point_2> points;

	double pi = 4 * std::atan(1);
	double threshold = 5 * pi / 180;

	// Set initial direction
	Vector_2 direction = polygon->edges_begin()->to_vector();
	double angle = std::atan(direction.y() / direction.x()), curr_angle;

	// Check edge direction
	points.push_back(polygon->vertex(0)); // Add first polygon vertex
	// For each edge
	for (auto e = polygon->edges_begin(); e != polygon->edges_end(); ++e) {
		// If edge is small, skip it
		// if (e->squared_length() < 0.01) { continue; }

		// Compute its direction
		direction = e->to_vector();
		curr_angle = std::atan(direction.y() / direction.x());

		// Compare direction to initial
		double diff = std::abs(angle - curr_angle);
		// If within threshold, continue
		if (diff < threshold) { continue; }
		// Else, set new direction and add edge source
		else { angle = curr_angle; points.push_back(e->source()); }
	}

	return points;
}


// Merge candidate polygons
inline std::vector<Point_2> merge_polygons(std::vector<Polygon_2>* polygons, std::vector<Polygon_2>* faces) {
	// Check overlapping
	std::vector<Segment_2> edges;
	for (auto polygon : *polygons) {
		// If face overlaps with segment
		if (check_overlap(&polygon, faces)) {
			// Store its edges
			for (auto e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
				edges.push_back(*e);
			}
		}
	}

	// Recover non-common edges
	std::vector<Segment_2>::iterator it;
	std::vector<Segment_2> selected;
	for (auto edge : edges) {
		// Check if opposite edge exists
		it = std::find(edges.begin(), edges.end(), edge.opposite());
		// If not...
		if (it == edges.end()) {
			// Add edge
			selected.push_back(edge);
		}
	}

	// Recover all rings with HDS
	std::vector<Polygon_2> rings = construct_polygons(&selected);

	// Recover ring of maximum area
	std::vector<Point_2> points_2d;
	if (!rings.empty()) {
		// Maximum area ring
		auto max_ring = std::max_element(rings.begin(), rings.end(),
			                             [&](const Polygon_2 &a, const Polygon_2 &b)
		                                 { return a.area() < b.area(); });

		// Simplify polygon
		points_2d = simplify_polygon(&*max_ring);
	}

	return points_2d;
}


// Define simplified face
inline std::vector<Point_3> define_face(const Mesh* mesh, unsigned int id, Plane_3* plane, std::vector<Polygon_2>* polygons) {
	// Project segment faces to 2D polygons
	std::vector<Polygon_2> faces = project_segment_faces(mesh, id, plane);
	//draw_mesh_segment(&faces, id);

	// Merge candidate polygons
	std::vector<Point_2> points_2d = merge_polygons(polygons, &faces);
	// Polygon_2 face_polygon(points_2d.begin(), points_2d.end());
	// draw_face(&face_polygon, id);

	// Project to 3D
	std::vector<Point_3> points_3d;
	for (auto point : points_2d) {
		points_3d.push_back(plane->to_3d(point));
	}

	return points_3d;
}


