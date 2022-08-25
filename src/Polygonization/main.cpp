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
 *      the 3D Geoinformation group.
 *
 * Copyright (C) 2019 3D Geoinformation Research Group
 * https://3d.bk.tudelft.nl/
 */

#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <chrono>

#include "Planarity.h"
#include "PlanarSegmentation.h"
#include "StructureGraph.h"
#include "Simplification.h"
#include "FileWritter.h"

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>


int main(int argc, char *argv[]) {
	srand(time(NULL));

    std::string input_file = std::string(POLYGONIZATION_ROOT_DIR) + "/../data/arc.off";
    std::cout << "input model: " << input_file << std::endl;
    if (argc == 2)
        input_file = argv[1];

	// Read mesh
	Mesh mesh;
    if (CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(input_file, mesh) == false) {
	    std::cerr << "Failed to load input model from file \'" << input_file << "\'." << std::endl;
	    return EXIT_FAILURE;
	}

    // Planarity inputs
	unsigned int num_rings = 3;
	/*std::cout << "Insert order of k-ring neighborhood: ";
	std::cin >> num_rings;*/

    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "------- Parameters (You may need to modify some of them) -------" << std::endl;

	// Segmentation inputs
#if 1
    double dist_threshold = 0.8;    // NOTE: you can modify this parameter here
#else
    double dist_threshold = 0.0;
	VProp_geom geom = mesh.points();
	for (auto e : mesh.edges()) {
		auto s = mesh.vertex(e, 0);
		auto t = mesh.vertex(e, 1);
        dist_threshold += std::sqrt(CGAL::squared_distance(geom[s], geom[t]));
	}
    dist_threshold /= mesh.num_edges();
#endif
    std::cout << "\tDistance threshold: " << std::setprecision(2) << dist_threshold << std::endl;

	// StructureGraph inputs
	double importance_threshold = 0.0;    // NOTE: you can modify this parameter here
	std::cout << "\tImportance threshold: " << std::setprecision(2) << importance_threshold << std::endl;

    auto solver = LinearProgramSolver::GUROBI;    // NOTE: you can modify this parameter here (available solvers are Gurobi and SCIP)
#ifdef HAS_GUROBI
    std::cout << "\tSolver: " << (solver == LinearProgramSolver::GUROBI ? "Gurobi" : "SCIP") << " " << std::endl;
#else
    std::cout << "\tSolver requested: " << (solver == LinearProgramSolver::GUROBI ? "Gurobi (Not available, use SCIP instead)" : "SCIP") << " " << std::endl;
#endif

    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

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
	std::size_t seg_number = seg.apply(&mesh, dist_threshold, num_rings);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Segmentation: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// StructureGraph
	start = std::chrono::steady_clock::now();
	StructureGraph graph;
	Graph structure_graph = graph.construct(&mesh, seg_number, importance_threshold);
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
	Mesh simplified = simpl.apply(&mesh, &structure_graph, solver);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Simplification: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Write simplified mesh
	const std::string result_file = input_file + "-result.ply";
	writeSimplified(&simplified, result_file);
	std::cout << "Done. Result saved to file \'" << result_file << std::endl;

	return EXIT_SUCCESS;
}