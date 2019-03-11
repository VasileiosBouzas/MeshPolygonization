#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
#include "CandidateFace.h"
#include "Draw.h"

Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G) {
	// Compute bbox of original mesh
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Supporting plane map
	std::map<unsigned int, Plane_3> plane_map = compute_supporting_planes(mesh, G);

	// Compute mesh vertices
	std::vector<Triple_intersection> vertices = compute_mesh_vertices(&bbox, G, &plane_map);

	// Compute plane intersections
	std::vector<Plane_intersection> segments = compute_mesh_edges(&bbox, G, &plane_map);
	// Split edges
	std::vector<Plane_intersection> edges = split_edges(&segments, &vertices);

	// Compute mesh faces
	std::vector<Candidate_face> faces = compute_mesh_faces(mesh, G, &plane_map, &edges);

	for (auto face : faces) {
		std::cout << face.conf << std::endl;
	}

	// Define simplified mesh
	Mesh simplified_mesh;
	return simplified_mesh;
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
			if (!found) { segments.push_back(segment); }
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
			for (auto plane : vertex_planes) {
				auto pos = std::find(segment_planes.begin(), segment_planes.end(), plane);
				if (pos != segment_planes.end()) { common_planes.push_back(plane); }
			}

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


// Compute mesh faces
std::vector<Candidate_face> Simplification::compute_mesh_faces(const Mesh* mesh, const Graph* G, std::map<unsigned int, Plane_3>* plane_map, std::vector<Plane_intersection>* edges) {
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