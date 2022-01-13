/**
 * MeshPolygonization is the implementation of the MVS (Multi-view Stereo) building mesh simplification method
 * described in the following paper:
 *      Vasileios Bouzas, Hugo Ledoux, and  Liangliang Nan.
 *      Structure-aware Building Mesh Polygonization.
 *      ISPRS Journal of Photogrammetry and Remote Sensing. 167(2020), 432-442, 2020.
 * Please cite the above paper if you use the code/program (or part of it).
 *
 * LICENSE:
 *      MeshPolygonization is free for academic use. If you are interested in a commercial license please contact
 *      the 3D Geoinformation group. You can find the information here: https://3d.bk.tudelft.nl/
 */

#pragma once

#include "Utils.h"
#include "Segment.h"


// Compute supporting planes of segments
inline std::map<unsigned int, Plane_3> compute_supporting_planes(const Mesh* mesh, const Graph* G) {
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


// Compute triple plane intersections
inline std::vector<Triple_intersection> compute_triple_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::vector<Triple_intersection> intersections;

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
				auto intersection = CGAL::intersection(curr_plane, adj_plane, other_plane);

				// Handle intersection
				if (intersection != boost::none) {
					if (const Point_3* pt = boost::get<Point_3>(&(*intersection))) {
						// Construct triple intersection
						Triple_intersection intersection;

						// Add geometry
						intersection.point = *pt;

						// Add planes
						intersection.planes.insert(curr_id);
						intersection.planes.insert(adj_id);
						intersection.planes.insert(other_id);

						// Update
						intersections.push_back(intersection); 
					}
				}
			}
		}
	}

	return intersections;
}


// Clip line with bbox planes
inline Segment_3 clip_line(const Line_3* line, const Bbox_3* bbox) {
	Segment_3 segment;

	// Compute bbox planes
	std::vector<Plane_3> bbox_planes = compute_bbox_planes(bbox);

	// Compute intersection with bbox
	std::vector<Point_3> points;
	for (auto plane : bbox_planes) {
		auto intersection = CGAL::intersection(*line, plane);

		// Handle intersection
		if (intersection != boost::none) {
			if (const Point_3* pt = boost::get<Point_3>(&(*intersection))) {
				if (is_in_bbox(bbox, pt)) { points.push_back(*pt); }
			}
		}
	}

	// Construct segment
	if (points.size() == 2) {
		return Segment_3(points[0], points[1]);
	}
	return segment;
}


// Compute plane intersections
inline std::vector<Plane_intersection> compute_intersections(const Bbox_3* bbox, const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::vector<Plane_intersection> intersections;

	unsigned int curr_id, adj_id;
	Plane_3 curr_plane, adj_plane;

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

		// Compute intersecting line
		auto intersection = CGAL::intersection(curr_plane, adj_plane);

		// Handle intersection
		if (intersection != boost::none) {
			if (const Line_3* line = boost::get<Line_3>(&(*intersection))) {
				// Clip line with bbox
				Segment_3 segment = clip_line(line, bbox);

				// Define plane intersection
				Plane_intersection intersection;

				// Add geometry
				intersection.segment = segment;

				// Add supporting planes
				intersection.planes.insert(curr_id);
				intersection.planes.insert(adj_id);

				// Update
				intersections.push_back(intersection);
			}
		}
	}

	return intersections;
}

