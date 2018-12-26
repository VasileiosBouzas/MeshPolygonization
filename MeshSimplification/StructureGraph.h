#pragma once

#include "Utils.h"

class StructureGraph
{
public:
	StructureGraph();
	~StructureGraph();

	Graph construct(Mesh* mesh, std::size_t seg_number, double imp_thres);
	static std::set<Face> select_segment(const Mesh* mesh, unsigned int id);

private:
	std::set<unsigned int> compute_importance(Mesh* mesh, std::size_t seg_number, double imp_thres);
	std::set<unsigned int> get_adjacent_segments(const Mesh* mesh, unsigned int id);
	std::size_t segment_to_vertex(const Graph* G, unsigned int id);
	Graph construct_structure_graph(const Mesh* mesh, std::set<unsigned int>* segments);
};

