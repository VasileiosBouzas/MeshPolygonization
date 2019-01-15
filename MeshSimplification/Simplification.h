#pragma once

#include "Utils.h"
#include "StructureGraph.h"

class Simplification
{
public:
	Simplification();
	~Simplification();

	Mesh apply(const Mesh* mesh, const Graph* G, double dist_thres);

private:
	std::map<unsigned int, Plane_3> compute_planes(const Mesh* mesh, const Graph* G);
	std::set<Point_3> compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map);
	std::vector<Point_3> get_interior_points(const Mesh* mesh, unsigned int id);
};

