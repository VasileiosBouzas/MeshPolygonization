#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include "../RPly/rply.h"
#include "Planarity.h"
#include "PlanarSegmentation.h"
#include "StructureGraph.h"
#include "Simplification.h"

// Functions
void writeGraph(const Mesh* mesh, const Graph* G, std::string filename);
void writeMesh(const Mesh* mesh, std::string filename);
void writeSimplified(const Mesh* mesh, std::string filename);

int main() {
	srand(time(NULL));

	// File inputs
	std::string filename;
	std::cout << "Insert file name: ";
	std::cin >> filename;

	// Planarity inputs
	unsigned int num_rings;
	std::cout << "Insert order of k-ring neighborhood: ";
	std::cin >> num_rings;

	// Segmentation inputs
	double dist_thres;
	std::cout << "Insert distance threshold: ";
	std::cin >> dist_thres;

	// StructureGraph inputs
	double imp_thres;
	std::cout << "Insert importance threshold: ";
	std::cin >> imp_thres;

	// Read mesh
	Mesh mesh;
	std::string file = "data//" + filename + ".off";
	std::ifstream input(file.c_str());
	if (!input || !(input >> mesh)) {
		std::cerr << "Input is not a triangle mesh" << std::endl;
		return EXIT_FAILURE;
	}

	// Calculate planarity
	time_t start = time(NULL);
	Planarity plan;
	plan.compute(&mesh, num_rings);
	// Execution time
	time_t end;
	end = time(NULL);
	std::cout << "Planarity: " << end - start << " secs" << std::endl;

	// Initialize segmentation
	start = time(NULL);
	PlanarSegmentation seg;
	std::size_t seg_number = seg.apply(&mesh, dist_thres, num_rings);
	// Execution time
	end = time(NULL);
	std::cout << "Segmentation: " << end - start << " secs" << std::endl;

	// StructureGraph
	start = time(NULL);
	StructureGraph graph;
	Graph structure_graph = graph.construct(&mesh, seg_number, imp_thres);
	// Execution time
	end = time(NULL);
	std::cout << "Structure Graph: " << end - start << " secs" << std::endl;

	// Simplification
	start = time(NULL);
	Simplification simpl;
	Mesh simplified = simpl.apply(&mesh, &structure_graph);
	// Execution time
	end = time(NULL);
	std::cout << "Simplification: " << end - start << " secs" << std::endl;

	// Visualize graph
	writeGraph(&mesh, &structure_graph, filename);

	// Write original mesh
	writeMesh(&mesh, filename);

	// Write simplified mesh
	writeSimplified(&simplified, filename);

	return EXIT_SUCCESS;
}


// Write graph
void writeGraph(const Mesh* mesh, const Graph* G, std::string filename) {
	unsigned int id;
	std::set<Face> segment;
	std::vector<Vertex> vs;
	std::set<Point_3> points;
	VProp_geom geom = mesh->points();
	Point_3 centroid;

	// Open file
	std::ofstream fout;
	std::string file = "graphs/" + filename + ".obj";
	fout.open(file.c_str());

	// Iterate graph vertices
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;

		// Select segment by id
		segment = StructureGraph::select_segment(mesh, id);

		// Collect segment points
		for (auto face : segment) {
			// Collect face vertices
			vs = vertex_around_face(mesh, face);

			// Collect face points
			for (auto v : vs) {
				points.insert(geom[v]);
			}
		}

		// Compute centroid
		centroid = CGAL::centroid(points.begin(), points.end(), CGAL::Dimension_tag<0>());

		// Write vertex
		fout << "v " << centroid.x() << " " << centroid.y() << " " << centroid.z() << std::endl;

		points.clear();
	}

	// Iterate graph edges
	Graph_edge_iterator eb, ee;
	for (boost::tie(eb, ee) = edges(*G); eb != ee; ++eb) {
		fout << "l " << boost::source(*eb, *G) + 1 << " " << boost::target(*eb, *G) + 1 << std::endl;
	}

	fout.close();
}


// Write .ply output
void writeMesh(const Mesh* mesh, std::string filename) {
	// Initialize .ply file
	std::string file = "outputs/" + filename + ".ply";
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
void writeSimplified(const Mesh* mesh, std::string filename) {
	// Initialize .ply file
	std::string file = "simplified/" + filename + ".ply";
	p_ply ply = ply_create(file.c_str(), PLY_LITTLE_ENDIAN, NULL, 0, NULL);

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
	}

	// Close
	ply_close(ply);
}