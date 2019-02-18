#pragma once

#include "Utils.h"

class Planarity
{
public:
	Planarity();
	~Planarity();

	void compute(Mesh* mesh, unsigned int num_rings = 1);

private:
	double compute_k_ring_planarity(const Mesh* mesh, const Vertex vertex, unsigned int num_rings);
	void planarity_to_faces(Mesh* mesh);
};

