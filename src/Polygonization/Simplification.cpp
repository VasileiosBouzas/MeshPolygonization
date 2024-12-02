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

#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
#include "CandidateFace.h"
#include "Optimization.h"
#include "Orientation.h"


Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G, LinearProgramSolver::SolverName solver_name) {
	// Compute bbox of original mesh
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Supporting plane map
	std::map<unsigned int, Plane_3> plane_map = compute_supporting_planes(mesh, G);

	// Compute mesh vertices
	std::vector<Triple_intersection> vertices = compute_mesh_vertices(&bbox, G, &plane_map);

	// Compute plane intersections
	std::vector<Plane_intersection> segments = compute_mesh_edges(&bbox, G, &plane_map);

	// Split & refine mesh edges
	std::vector<Plane_intersection> edges = split_edges(&segments, &vertices);
	refine_edges(&edges, &vertices, &plane_map);

	// Compute mesh faces
	std::vector<Candidate_face> faces = compute_mesh_faces(mesh, G, &plane_map, &edges);

	// Optimize
	return simplify(&vertices, &edges, &faces, solver_name);;
}


// Compute mesh vertices
std::vector<Triple_intersection> Simplification::compute_mesh_vertices(const Bbox_3* bbox, const Graph* G, std::map<unsigned int, Plane_3>* plane_map) {
	// Traverse structure graph
	unsigned int id;
	Plane_3 plane;
	std::vector<Triple_intersection> new_points, points;

	// For each segment
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = (*plane_map)[id];

		// Compute triple plane intersections
		new_points = compute_triple_intersections(G, *vb, plane_map);
		for (auto point : new_points) {
			// If inside bbox
			if (is_in_bbox(bbox, &point.point)) {
				bool found = false;

				// Check if exists
				for (auto existing : points) {
					// Triple intersection equality ==
					// Supporting planes are equal
					// For each triple of planes, only one vertex should exist!
					if (point.planes == existing.planes) {
						found = true; break;
					}
				}

				// If not, add
				if (!found) { points.push_back(point); }
			}
		}
	}

	return points;
}


// Compute mesh edges
std::vector<Plane_intersection> Simplification::compute_mesh_edges(const Bbox_3* bbox, const Graph* G, std::map<unsigned int, Plane_3>* plane_map) {
	// Traverse structure graph
	unsigned int id;
	Plane_3 plane;
	std::vector<Plane_intersection> new_segments, segments;

	// For each segment
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = (*plane_map)[id];

		// Compute triple plane intersections
		new_segments = compute_intersections(bbox, G, *vb, plane_map);
		for (auto segment : new_segments) {
			bool found = false;

			// Check if exists
			for (auto existing : segments) {
				// Plane intersection equality ==
				// Supporting planes are equal
				// For each plane pair, only one intersection should exist!
				if (segment.planes == existing.planes) {
					found = true; break;
				}
			}

			// If not, add
			if (!found) { 
				segments.push_back(segment); 
			}
		}
	}

	return segments;
}


// Split mesh edges with mesh vertices
std::vector<Plane_intersection> Simplification::split_edges(std::vector<Plane_intersection>* segments, std::vector<Triple_intersection>* vertices) {
	std::vector<Plane_intersection> edges;

	// For each segment
	for (auto segment : *segments) {
		// Retrieve supporting planes
		auto segment_planes = segment.planes;

		// Find vertices to split edge
		std::vector<int> splitters;
		// For each vertex
		for (auto i = 0; i < vertices->size(); i++) {
			// Retrieve supporting planes
			auto vertex_planes = (*vertices)[i].planes;

			// Find common supporting planes with edge
			std::vector<int> common_planes;
			std::set_intersection(segment_planes.begin(), segment_planes.end(), 
								  vertex_planes.begin(), vertex_planes.end(),
				                  std::back_inserter(common_planes));

			// Two common planes ==
			// Vertex lies on edge => split edge!
			if (common_planes.size() == 2) { splitters.push_back(i); }
		}

		// Retrieve edge endpoints
		Point_3 source = segment.segment.source();
		Point_3 target = segment.segment.target();

		// Sort split points by distance to source
		std::sort(splitters.begin(), splitters.end(), 
			      [&](const int &v1, const int &v2) {
			      return CGAL::squared_distance(source, (*vertices)[v1].point) < 
				         CGAL::squared_distance(source, (*vertices)[v2].point); });

		// Split edge into edges
		for (auto i = 0; i < splitters.size(); i++) {
			int j = i + 1;
			if (j >= splitters.size()) { break; }

			// Recover vertices
			int v1 = splitters[i];
			int v2 = splitters[j];

			// Define plane intersection
			Plane_intersection edge;

			// Add geometry
			source = (*vertices)[v1].point;
			target = (*vertices)[v2].point;
			edge.segment = Segment_3(source, target);

			// Add vertices
			edge.vertices.push_back(v1);
			edge.vertices.push_back(v2);

			// Add supporting plane ==
			// The same as of the original segment!
			edge.planes.insert(segment_planes.begin(), segment_planes.end());

			// Update
			edges.push_back(edge);
		}
	}

	return edges;
}


// 3D segment intersection
bool Simplification::do_intersect(Segment_3* segment, Plane_3* plane) {
	// Retrieve segment endpoints
	Point_3 s = segment->source();
	Point_3 t = segment->target();

	// Oriented sides
	auto ss = plane->oriented_side(s);
	auto st = plane->oriented_side(t);

	// Either the endpoints must lie on the same oriented side
	// or one on the oriented boundary and the other on any side
	if ((ss == CGAL::ON_POSITIVE_SIDE && st == CGAL::ON_NEGATIVE_SIDE) ||
		(ss == CGAL::ON_NEGATIVE_SIDE && st == CGAL::ON_POSITIVE_SIDE))
		return true;
		
	return false;
}


// Cross-section split
void Simplification::cross_section_split(std::vector<Plane_intersection>* edges, Plane_intersection* e, const Point_3* pt, int idx) {
	// Create two edges out of the old edge
    Plane_intersection e1, e2;

	// Add geometry
	e1.segment = Segment_3(e->segment.source(), *pt);
	e2.segment = Segment_3(*pt, e->segment.target());

	// Add vertices
	e1.vertices.push_back(e->vertices[0]);
	e1.vertices.push_back(idx);

	e2.vertices.push_back(idx);
	e2.vertices.push_back(e->vertices[1]);

	// Add supporting planes == equal to the original
	e1.planes = e->planes;
	e2.planes = e->planes;

	// Delete original segment
	for (auto i = 0; i < edges->size(); i++) {
		auto edge = (*edges)[i];
		// Plane intersection equality
		if (e->segment == edge.segment && e->planes == edge.planes) {
			edges->erase(edges->begin() + i);
			break;
		}
	}

	// Add new edges
	edges->push_back(e1);
	edges->push_back(e2);
}


// Refine edges
void Simplification::refine_edges(std::vector<Plane_intersection>* edges, std::vector<Triple_intersection>* vertices, std::map<unsigned int, Plane_3>* plane_map) {
	bool splitted = true;
	do {
		splitted = false;

		// Check for edge intersections
		for (auto i = 0; i < edges->size(); i++) {
			// Retrieve segment
			auto ei = (*edges)[i];
			auto seg_i = ei.segment;
			auto pl_i = ei.planes;

			// Check all other segments
			for (auto j = i + 1; j < edges->size(); j++) {
				// Retrieve segment
				auto ej = (*edges)[j];
				auto seg_j = ej.segment;
				auto pl_j = ej.planes;

				// If edges share point, continue
				if (seg_i.has_on(seg_j.source()) ||
					seg_i.has_on(seg_j.target()))
					continue;

				// Check common planes
				std::vector<int> common_planes;
				std::set_intersection(pl_i.begin(), pl_i.end(),
					pl_j.begin(), pl_j.end(),
					std::back_inserter(common_planes));

				// Two edges can have at most two common planes!
				// 0 common planes == irrelevant
				// 1 common planes == edges must intersect either at endpoints (good!) or inside (very bad...)
				// 2 common planes == edges are part of the same intersection, continue
				if (common_planes.size() != 1) { continue; }

				// Retrieve all other planes that the common one
				std::vector<int> diff_i, diff_j;
				std::set_difference(pl_i.begin(), pl_i.end(), common_planes.begin(), common_planes.end(), std::back_inserter(diff_i));
				std::set_difference(pl_j.begin(), pl_j.end(), common_planes.begin(), common_planes.end(), std::back_inserter(diff_j));
				auto plane_i = (*plane_map)[diff_i[0]];
				auto plane_j = (*plane_map)[diff_j[0]];

				// Check if they intersect inside
				if (do_intersect(&seg_i, &plane_j) && do_intersect(&seg_j, &plane_i)) {
					// Compute intersection
					auto intersection = CGAL::intersection(seg_i.supporting_line(), plane_j);

					if (const Point_3* pt = boost::get<Point_3>(&(*intersection))) {
						// Create triple intersection
						Triple_intersection vertex;
						vertex.point = *pt;
						vertex.planes.insert(ei.planes.begin(), ei.planes.end());
						vertex.planes.insert(ej.planes.begin(), ej.planes.end());
						vertices->push_back(vertex);

						// New intersection index
						int idx = int(vertices->size()) - 1;

						// Split edges
						cross_section_split(edges, &ei, pt, idx); // First edge
						cross_section_split(edges, &ej, pt, idx); // Second edge

						splitted = true; break;
					}
				}
			}

			if (splitted) break;
		}
	} while (splitted);
}


// Compute mesh faces
std::vector<Candidate_face> Simplification::compute_mesh_faces(const Mesh* mesh, const Graph* G, 
															   std::map<unsigned int, Plane_3>* plane_map, 
	                                                           std::vector<Plane_intersection>* edges)
{
	std::vector<Candidate_face> candidate_faces;

	// Iterate segments
	unsigned int id;
	Plane_3 plane;
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = (*plane_map)[id];

		// Retrieve segment edges
		std::vector<int> plane_edges;
		for (auto i = 0; i < edges->size(); i++) {
			// Retrieve edge planes
			auto planes = (*edges)[i].planes;

			// Check for segment plane
			if (planes.find(id) != planes.end()) {
				plane_edges.push_back(i);
			}
		}

		// Construct candidate faces of segment
		std::vector<Candidate_face> faces = compute_candidate_faces(mesh, id, &plane, edges, &plane_edges);

		// Update
		candidate_faces.insert(candidate_faces.end(), faces.begin(), faces.end());
	}

	// Update edges
	for (auto i = 0; i < edges->size(); i++) {
		// Check all faces
		for (auto j = 0; j < candidate_faces.size(); j++) {
			// Retrieve face edges
			auto e = candidate_faces[j].edges;

			// If edge found, update
			auto pos = std::find(e.begin(), e.end(), i);
			if (pos != e.end()) { (*edges)[i].faces.push_back(j); }
		}
	}

	return candidate_faces;
}


// Simplify
Mesh Simplification::simplify(std::vector<Triple_intersection>* vertices, std::vector<Plane_intersection>* edges, std::vector<Candidate_face>* faces, LinearProgramSolver::SolverName solver_name) {
	// Construct proxy mesh
	Mesh proxy_mesh;

	// Face attributes //
	// Face index
	Mesh::Property_map<Face, std::size_t> face_indices = proxy_mesh.add_property_map<Face, std::size_t>("f:index").first;

	// Vertex indices
	Mesh::Property_map<Face, std::vector<int>> vertex_indices = proxy_mesh.add_property_map<Face, std::vector<int>>("f:vertices").first;

	// Number of supporting faces
	Mesh::Property_map<Face, std::size_t> supporting_face_num = proxy_mesh.add_property_map<Face, std::size_t>("f:supporting_face_num").first;

	// Covered area
	Mesh::Property_map<Face, double> covered_area = proxy_mesh.add_property_map<Face, double>("f:covered_area").first;

	// Total area
	Mesh::Property_map<Face, double> area = proxy_mesh.add_property_map<Face, double>("f:area").first;
	// Face attributes //

	// Define faces
	for (auto face : *faces) {
		// Add topology
		std::vector<Vertex> face_vertices;
		for (auto i : face.vertices) {
			Triple_intersection vertex = (*vertices)[i];
			Vertex v = proxy_mesh.add_vertex(vertex.point);
			face_vertices.push_back(v);
		}
		Face f = proxy_mesh.add_face(face_vertices);

		// Add attributes
		vertex_indices[f] = face.vertices;                 // Vertex indices
		supporting_face_num[f] = face.supporting_face_num; // Number of supporting faces
		covered_area[f] = face.covered_area;               // Covered area
		area[f] = face.area;                               // Total area

		// Update edges
		// Retrieve face edges
		auto face_edges = face.edges;
		for (auto i = 0; i < edges->size(); i++) {
			// Check if edge is in face edges
			auto pos = std::find(face_edges.begin(), face_edges.end(), i);

			// If so, add to edge fan
			if (pos != face_edges.end()) { 
				(*edges)[i].fan.push_back(f);
			}
		}
	}

	// Optimize
	std::vector<double> X = optimize(&proxy_mesh, edges, solver_name);

	// Faces to delete
	std::vector<Face> to_delete;
	std::size_t f_idx(0);
	for (auto f : proxy_mesh.faces()) {
		if (static_cast<int>(std::round(X[f_idx])) == 0)
			to_delete.push_back(f);
		++f_idx;
	}

	// Face deletion
	for (std::size_t i = 0; i < to_delete.size(); ++i) {
	    Face f = to_delete[i];
		Halfedge h = proxy_mesh.halfedge(f);
		CGAL::Euler::remove_face(h, proxy_mesh);
	}

	// Ensure consistent orientation
	orient(proxy_mesh, *vertices);
	return proxy_mesh;
}