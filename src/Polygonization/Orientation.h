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
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

void orient(Mesh* mesh, std::vector<Triple_intersection>* vertices) {
	// Retrieve mesh points
	std::vector<Point_3> points;
	for (auto vertex : *vertices) {
		points.push_back(vertex.point);
	}

	// Construct polygons to be oriented
	Mesh::Property_map<Face, std::vector<int>> vertex_indices = mesh->property_map<Face, std::vector<int>>("f:vertices").first;
	std::vector<std::vector<int>> polygons;
	for (auto f : mesh->faces()) {
		// Retrieve vertex indices
		std::vector<int> polygon = vertex_indices[f];

		// Add to polygons
		polygons.push_back(polygon);
	}

	CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
	mesh->clear();
	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, *mesh);
	
	for (auto v : mesh->vertices()) {
		if (mesh->is_isolated(v))
			mesh->remove_vertex(v);
	}
	mesh->collect_garbage();

	if (CGAL::is_closed(*mesh) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(*mesh)))
		CGAL::Polygon_mesh_processing::reverse_face_orientations(*mesh);

	CGAL::Polygon_mesh_processing::triangulate_faces(*mesh);
}