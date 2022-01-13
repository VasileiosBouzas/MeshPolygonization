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
 *      the 3D Geoinformation group. You can find the information here: https://3d.bk.tudelft.nl/
 */

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
	bool do_intersect(Segment_3* segment, Plane_3* plane);
	void cross_section_split(std::vector<Plane_intersection>* edges, Plane_intersection* e, const Point_3* pt, int idx);
	void refine_edges(std::vector<Plane_intersection>* edges, std::vector<Triple_intersection>* vertices, std::map<unsigned int, Plane_3>* plane_map);
	std::vector<Candidate_face> compute_mesh_faces(const Mesh* mesh, const Graph* G, std::map<unsigned int, Plane_3>* plane_map, std::vector<Plane_intersection>* edges);
	Mesh simplify(std::vector<Triple_intersection>* vertices, std::vector<Plane_intersection>* edges, std::vector<Candidate_face>* faces);
};

