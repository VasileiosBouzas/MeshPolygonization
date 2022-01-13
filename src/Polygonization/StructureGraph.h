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

#pragma once

#include "Utils.h"

class StructureGraph
{
public:
	StructureGraph();
	~StructureGraph();

	Graph construct(Mesh* mesh, std::size_t seg_number, double imp_thres);

private:
	std::set<unsigned int> compute_importance(Mesh* mesh, std::size_t seg_number, double imp_thres);
	std::set<unsigned int> get_adjacent_segments(const Mesh* mesh, unsigned int id);
	std::size_t segment_to_vertex(const Graph* G, unsigned int id);
	Graph construct_structure_graph(const Mesh* mesh, std::set<unsigned int>* segments);
};

