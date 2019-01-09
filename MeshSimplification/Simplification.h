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
	std::map<unsigned int, Plane> compute_planes(const Mesh* mesh, const Graph* G);
	std::vector<Point> compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane>* plane_map);
};

