#pragma once

#include "Utils.h"


// Compute segment orientation
inline Vector_3 compute_segment_orientation(const Mesh* mesh, unsigned int id) {
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	for (auto face : mesh->faces()) {
		if (chart[face] == id) {
			return CGAL::Polygon_mesh_processing::compute_face_normal(face, *mesh);
		}
	}
}


// Reverse orientation
inline void reverse_orientation(Mesh* mesh, Halfedge first) {
	if (first == Halfedge()) return;
	Halfedge last = first;
	Halfedge prev = first;
	Halfedge start = first;
	first = mesh->next(first);
	Vertex  new_v = mesh->target(start);
	while (first != last) {
		Vertex  tmp_v = mesh->target(first);
		mesh->set_target(first, new_v);
		mesh->set_halfedge(new_v, first);
		new_v = tmp_v;
		Halfedge n = mesh->next(first);
		mesh->set_next(first, prev);
		prev = first;
		first = n;
	}
	mesh->set_target(start, new_v);
	mesh->set_halfedge(new_v, start);
	mesh->set_next(start, prev);
}


// Reverse face orientations
inline void reverse_face_orientations(Mesh* mesh, Face face) {
	reverse_orientation(mesh, mesh->halfedge(face));

	for (auto hd : mesh->halfedges_around_face(mesh->halfedge(face))) {
		Halfedge ohd = mesh->opposite(hd);
		if (mesh->is_border(ohd) && mesh->target(hd) == mesh->target(ohd)) {
			reverse_orientation(mesh, ohd);
		}
	}
}
