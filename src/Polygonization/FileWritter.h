#pragma once

#include <fstream>

#include "Segment.h"
#include "rply.h"



// Write graph
inline void writeGraph(const Mesh* mesh, const Graph* G, std::string file) {
	unsigned int id;
	Point_3 centroid;

	// Open file
	std::ofstream fout(file.c_str());

	// Iterate graph vertices
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;

		// Retrieve segment centroid
		centroid = get_segment_centroid(mesh, id);

		// Write vertex
		fout << "v " << centroid.x() << " " << centroid.y() << " " << centroid.z() << std::endl;
	}

	// Iterate graph edges
	Graph_edge_iterator eb, ee;
	for (boost::tie(eb, ee) = edges(*G); eb != ee; ++eb) {
		fout << "l " << boost::source(*eb, *G) + 1 << " " << boost::target(*eb, *G) + 1 << std::endl;
	}

	fout.close();
}


// Write .ply output
inline void writeMesh(const Mesh* mesh, std::string file) {
	// Initialize .ply file
	p_ply ply = ply_create(file.c_str(), PLY_LITTLE_ENDIAN, NULL, 0, NULL);

	// Vertices
	const char* name = "vertex";
	long ninstances = mesh->num_vertices();
	ply_add_element(ply, name, ninstances);

	ply_add_scalar_property(ply, "x", PLY_DOUBLE);
	ply_add_scalar_property(ply, "y", PLY_DOUBLE);
	ply_add_scalar_property(ply, "z", PLY_DOUBLE);
	ply_add_scalar_property(ply, "planarity", PLY_DOUBLE);

	// Faces
	name = "face";
	ninstances = mesh->num_faces();
	ply_add_element(ply, name, ninstances);

	ply_add_list_property(ply, "vertex_indices", PLY_UCHAR, PLY_INT);
	ply_add_scalar_property(ply, "planarity", PLY_DOUBLE);
	ply_add_scalar_property(ply, "chart", PLY_INT);
	ply_add_scalar_property(ply, "imp", PLY_DOUBLE);
	ply_add_scalar_property(ply, "red", PLY_UCHAR);
	ply_add_scalar_property(ply, "green", PLY_UCHAR);
	ply_add_scalar_property(ply, "blue", PLY_UCHAR);

	// Header
	ply_write_header(ply);

	// Vertex properties
	VProp_geom geom = mesh->points();
	VProp_double v_planar = mesh-> property_map<Vertex, double>("v:planarity").first;

	for (auto v : mesh->vertices()) {
		Point_3 point = geom[v];
		ply_write(ply, point.x());
		ply_write(ply, point.y());
		ply_write(ply, point.z());
		ply_write(ply, v_planar[v]);
	}

	// Face properties
	FProp_double f_planar = mesh->property_map<Face, double>("f:planarity").first;
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	FProp_double imp = mesh->property_map<Face, double>("f:imp").first;
	FProp_color color = mesh->property_map<Face, Point_3>("f:color").first;

	std::vector<Vertex> vertices;
	for (auto f : mesh->faces()) {
		// Collect face vertices
		vertices = vertex_around_face(mesh, f);
		
		ply_write(ply, vertices.size());
		for (auto v : vertices) {
			ply_write(ply, v);
		}

		// Add planarity attribute
		ply_write(ply, f_planar[f]);

		// Add chart attribute
		ply_write(ply, chart[f]);

		// Add importance attribute
		ply_write(ply, imp[f]);

		// Add color attribute
		ply_write(ply, color[f].x());
		ply_write(ply, color[f].y());
		ply_write(ply, color[f].z());
	}

	// Close
	ply_close(ply);
}


// Write simplified mesh
inline void writeSimplified(const Mesh* mesh, std::string file) {
	// Initialize .ply file
	p_ply ply = ply_create(file.c_str(), PLY_ASCII, NULL, 0, NULL);

	// Vertices
	const char* name = "vertex";
	long ninstances = mesh->num_vertices();
	ply_add_element(ply, name, ninstances);

	ply_add_scalar_property(ply, "x", PLY_DOUBLE);
	ply_add_scalar_property(ply, "y", PLY_DOUBLE);
	ply_add_scalar_property(ply, "z", PLY_DOUBLE);

	// Faces
	name = "face";
	ninstances = mesh->num_faces();
	ply_add_element(ply, name, ninstances);

	ply_add_list_property(ply, "vertex_indices", PLY_UCHAR, PLY_INT);
	/*ply_add_scalar_property(ply, "supporting_face_num", PLY_DOUBLE);
	ply_add_scalar_property(ply, "covered_area", PLY_DOUBLE);*/

	// Header
	ply_write_header(ply);

	// Vertex properties
	VProp_geom geom = mesh->points();

	// Write vertices
	for (auto v : mesh->vertices()) {
		Point_3 point = geom[v];
		ply_write(ply, point.x());
		ply_write(ply, point.y());
		ply_write(ply, point.z());
	}

	// Write faces
	std::vector<Vertex> vertices;

	for (auto f : mesh->faces()) {
		// Collect face vertices
		vertices = vertex_around_face(mesh, f);

		ply_write(ply, vertices.size());
		for (auto v : vertices) {
			ply_write(ply, v);
		}

		/*ply_write(ply, supporting_face_num[f]);
		ply_write(ply, covered_area[f]);*/
	}

	// Close
	ply_close(ply);
}
