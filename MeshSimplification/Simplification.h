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
	std::vector<Point_3> get_interior_points(const Mesh* mesh, unsigned int id);
};

