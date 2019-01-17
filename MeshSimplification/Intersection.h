#pragma once

#include "Utils.h"
#include "Segment.h"


// Compute supporting planes of segments
inline std::map<unsigned int, Plane_3> compute_planes(const Mesh* mesh, const Graph* G) {
	std::map<unsigned int, Plane_3> plane_map;

	// Compute planes for segments
	Graph_vertex_iterator vb, ve;
	unsigned int id;
	std::set<Face> segment;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;

		// Select segment by id
		segment = select_segment(mesh, id);

		// Compute plane for segment
		plane_map[id] = fit_plane_to_faces(mesh, &segment);
	}

	return plane_map;
}


// Compute plane intersections
inline std::set<Point_3> compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::set<Point_3> points;

	unsigned int curr_id, adj_id, other_id;
	Plane_3 curr_plane, adj_plane, other_plane;

	// Vertex to plane
	curr_id = (*G)[v].segment;
	curr_plane = (*plane_map)[curr_id];

	// Find adjacent vertices
	auto adjacent = boost::adjacent_vertices(v, *G);

	// For every adjacent
	for (auto adj : make_iterator_range(adjacent)) {
		// Vertex to plane
		adj_id = (*G)[adj].segment;
		adj_plane = (*plane_map)[adj_id];

		// Find adjacent segments of adjacent
		auto others = boost::adjacent_vertices(adj, *G);

		// For every other plane
		for (auto other : make_iterator_range(others)) {
			// Check if edge exists
			if (adj < other && boost::edge(v, other, *G).second) {
				// Vertex to plane
				other_id = (*G)[other].segment;
				other_plane = (*plane_map)[other_id];

				// Compute intersection fo three planes
				auto inter = CGAL::intersection(curr_plane, adj_plane, other_plane);

				// Handle intersection
				if (inter != boost::none) {
					if (const Point_3* pt = boost::get<Point_3>(&(*inter))) { points.insert(*pt); }
				}
			}
		}
	}

	return points;
}