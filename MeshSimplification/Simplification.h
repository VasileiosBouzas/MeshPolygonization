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
	std::map<unsigned int, Plane_3> compute_planes(const Mesh* mesh, const Graph* G);
	std::vector<Point_3> compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map);
};

