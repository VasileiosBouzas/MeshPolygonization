#pragma once

#include "Utils.h"
#include "Draw.h"


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
inline std::vector<Candidate_face> construct_candidate_faces(std::vector<Segment_2>* segments, std::vector<int>* plane_edges) {
	std::vector<Candidate_face> candidate_faces;

	// Construct 2D arrangement
	Arrangement_2 arr;
	CGAL::insert_non_intersecting_curves(arr, segments->begin(), segments->end());

	// Iterate arrangement faces
	for (Face_iterator f = arr.faces_begin(); f != arr.faces_end(); ++f) {
		// Check if face is unbounded
		if (f->is_unbounded()) { continue; }

		// Define candidate face
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
				if (curve == segment || curve == segment.opposite()) {
					pos = i; break;
				}
			}

			// Retrieve edge
			auto edge = (*plane_edges)[pos];
			// Add to face
			candidate_face.edges.push_back(edge);

		} while (++cc != f->outer_ccb());

		// Update
		candidate_faces.push_back(candidate_face);
	}

	return candidate_faces;
}


inline std::vector<Candidate_face> segments_to_polygons(Plane_3* plane, std::vector<Plane_intersection>* edges, std::vector<int>* plane_edges) {
	// Project segments on plane
	std::vector<Segment_2> segments = project_segments(plane, edges, plane_edges);

	// Construct simple polygons
	std::vector<Candidate_face> candidate_faces = construct_candidate_faces(&segments, plane_edges);

	return candidate_faces;
}