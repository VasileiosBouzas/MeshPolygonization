#pragma once

#include "Utils.h"

class PlanarSegmentation
{
public:
	PlanarSegmentation();
	~PlanarSegmentation();

	std::size_t apply(Mesh* mesh, double dist_thres, unsigned int num_rings);

private:
	Face get_max_planarity_face(const Mesh* mesh, std::set<Face>* faces);
	bool check_distance(const Mesh* mesh, Face face, Plane plane, double dist);
	Point random_color();
};

