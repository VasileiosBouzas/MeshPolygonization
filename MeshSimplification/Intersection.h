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
inline std::vector<Line_3> compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::vector<Line_3> lines;

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
			if (const Line_3* line = boost::get<Line_3>(&(*intersection))) { lines.push_back(*line); }
		}
	}

	return lines;
}


// Compute bbox - supporting plane intesections
inline std::vector<Line_3> compute_bbox_intersections(const Bbox_3* bbox, Plane_3* plane) {
	std::vector<Line_3> lines;

	// Compute bbox planes
	std::vector<Plane_3> bbox_planes = compute_bbox_planes(bbox);

	// For every plane
	for (auto bbox_plane : bbox_planes) {
		// Compute intersecting line
		auto intersection = CGAL::intersection(*plane, bbox_plane);

		// Handle intersection
		if (intersection != boost::none) {
			if (const Line_3* line = boost::get<Line_3>(&(*intersection))) { lines.push_back(*line); }
		}
	}

	return lines;
}


// Clip lines with bbox planes
inline std::vector<Segment_3> clip_lines(std::vector<Line_3>* lines, const Bbox_3* bbox) {
	std::vector<Segment_3> segments;

	// Compute bbox planes
	std::vector<Plane_3> bbox_planes = compute_bbox_planes(bbox);

	// For each line
	std::vector<Point_3> points;
	for (auto line : *lines) {
		// Compute intersection with bbox
		for (auto plane : bbox_planes) {
			auto intersection = CGAL::intersection(line, plane);

			// Handle intersection
			if (intersection != boost::none) {
				if (const Point_3* pt = boost::get<Point_3>(&(*intersection))) {
					if (is_in_bbox(bbox, pt)) { points.push_back(*pt); }
				}
			}
		}

		segments.push_back(Segment_3(points[0], points[1]));
		points.clear();
	}

	return segments;
}