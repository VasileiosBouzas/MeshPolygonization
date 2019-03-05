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
	bool check_distance(const Mesh* mesh, Face face, Plane_3 plane, double dist);
	Point_3 random_color();
	std::size_t refine_segmentation(Mesh* mesh, std::size_t seg_number, std::map<int, Plane_3>* plane_map, std::map<int, std::set<Face>>* segment_map, double dist);
	bool check_fitting(Mesh* mesh, std::set<Face>* segment, Plane_3 * plane, double dist);
	int merge_segments(Mesh* mesh, std::size_t seg_number, std::set<Face>* seg_1, std::set<Face>* seg_2);
};

