#pragma once

#include "Utils.h"
#include "StructureGraph.h"

class Simplification
{
public:
	Simplification();
	~Simplification();

	Mesh apply(const Mesh* mesh, const Graph* G);

private:
	std::vector<Triple_intersection> compute_mesh_vertices(const Bbox_3* bbox, const Graph* G, std::map<unsigned int, Plane_3>* plane_map);
	std::vector<Plane_intersection> compute_mesh_edges(const Bbox_3* bbox, const Graph* G, std::map<unsigned int, Plane_3>* plane_map);
	std::vector<Plane_intersection> split_edges(std::vector<Plane_intersection>* segments, std::vector<Triple_intersection>* points);
};

