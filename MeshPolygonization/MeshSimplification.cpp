#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include "../RPly/rply.h"
#include "Planarity.h"
#include "PlanarSegmentation.h"
#include "StructureGraph.h"
#include "Segment.h"
#include "Simplification.h"
#include <chrono>

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

	// Read mesh
	Mesh mesh;
	std::string file = "data//" + filename + ".off";
	std::ifstream input(file.c_str());
	if (!input || !(input >> mesh)) {
		std::cerr << "Input is not a triangle mesh" << std::endl;
		return EXIT_FAILURE;
	}
	
	// Planarity inputs
	unsigned int num_rings = 3;
	/*std::cout << "Insert order of k-ring neighborhood: ";
	std::cin >> num_rings;*/

	// Segmentation inputs
	double dist_thres = 0;
	VProp_geom geom = mesh.points();
	for (auto h : mesh.halfedges()) {
		auto source = geom[mesh.source(h)];
		auto target = geom[mesh.target(h)];
		dist_thres += std::sqrt(CGAL::squared_distance(source, target));
	}
	dist_thres /= mesh.number_of_halfedges();
	std::cout << "Distance threshold: " << std::setprecision(2) << dist_thres << std::endl;
	std::cout << "Insert distance threshold: ";
	std::cin >> dist_thres;

	// StructureGraph inputs
	double imp_thres;
	std::cout << "Insert importance threshold: ";
	std::cin >> imp_thres;

	// Calculate planarity
	auto start = std::chrono::steady_clock::now();
	Planarity plan;
	plan.compute(&mesh, num_rings);
	// Execution time
	auto end = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Planarity: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Initialize segmentation
	start = std::chrono::steady_clock::now();
	PlanarSegmentation seg;
	std::size_t seg_number = seg.apply(&mesh, dist_thres, num_rings);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Segmentation: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// StructureGraph
	start = std::chrono::steady_clock::now();
	StructureGraph graph;
	Graph structure_graph = graph.construct(&mesh, seg_number, imp_thres);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Structure Graph: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Write original mesh
	writeMesh(&mesh, filename);

	// Visualize graph
	writeGraph(&mesh, &structure_graph, filename);

	// Simplification
	start = std::chrono::steady_clock::now();
	Simplification simpl;
	Mesh simplified = simpl.apply(&mesh, &structure_graph);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Simplification: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Write simplified mesh
	writeSimplified(&simplified, filename);

	return EXIT_SUCCESS;
}


// Write graph
void writeGraph(const Mesh* mesh, const Graph* G, std::string filename) {
	unsigned int id;
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
	Mesh::Property_map<Face, std::size_t> supporting_face_num = mesh->property_map<Face, std::size_t>("f:supporting_face_num").first;
	Mesh::Property_map<Face, double> covered_area = mesh->property_map<Face, double>("f:covered_area").first;
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