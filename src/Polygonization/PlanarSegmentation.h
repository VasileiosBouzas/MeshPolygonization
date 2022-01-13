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

class PlanarSegmentation
{
public:
	PlanarSegmentation();
	~PlanarSegmentation();

	std::size_t apply(Mesh* mesh, double dist_thres, unsigned int num_rings);

private:
	Face get_max_planarity_face(const Mesh* mesh, std::set<Face>* faces);
	bool check_distance(const Mesh* mesh, Face* face, Plane_3* plane, double dist);
	Point_3 random_color();
	std::size_t refine_segmentation(Mesh* mesh, std::size_t seg_number, std::map<int, Plane_3>* plane_map, std::map<int, std::set<Face>>* segment_map, double dist);
	bool check_fitting(Mesh* mesh, std::set<Face>* segment, Plane_3 * plane, double dist);
	int merge_segments(Mesh* mesh, std::size_t seg_number, std::set<Face>* seg_1, std::set<Face>* seg_2);
};

