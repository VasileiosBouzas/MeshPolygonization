#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include "Planarity.h"
#include "PlanarSegmentation.h"
#include "StructureGraph.h"
#include "Simplification.h"
#include "FileWritter.h"

#include <chrono>


int main(int argc, char *argv[]) {
	srand(time(NULL));

    std::string input_file = "../../../data/scene.off";
    if (argc == 2)
        input_file = argv[1];

	// Read mesh
	Mesh mesh;
	std::ifstream input(input_file.c_str());
	if (input.fail()) {
	    std::cerr << "Failed to open file \'" << input_file << "\'." << std::endl;
	    return EXIT_FAILURE;
	}
	if (!(input >> mesh)) {
		std::cerr << "Failed loading model from the file." << std::endl;
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

//	// Write original mesh
//	writeMesh(&mesh, input_file + "-input.ply");
//
//	// Write graph
//	writeGraph(&mesh, &structure_graph, input_file + "-graph.obj");

	// Simplification
	start = std::chrono::steady_clock::now();
	Simplification simpl;
	Mesh simplified = simpl.apply(&mesh, &structure_graph);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Simplification: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Write simplified mesh
	writeSimplified(&simplified, input_file + "-result.ply");

	return EXIT_SUCCESS;
}