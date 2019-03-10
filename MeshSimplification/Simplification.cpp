#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
#include "LinesToPolygons.h"
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
	std::vector<Triple_intersection> mesh_vertices = compute_mesh_vertices(&bbox, G, &plane_map);

	// Compute plane intersections
	std::vector<Plane_intersection> edges = compute_mesh_edges(&bbox, G, &plane_map);
	// Split edges
	edges = split_edges(&edges, &mesh_vertices);

	draw_frame(&mesh_vertices, &edges);

	unsigned int id;
	Plane_3 plane;
	Graph_vertex_iterator vb, ve;
	int n = 0;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = plane_map[id];

		std::vector<Segment_3> segments;
		for (auto edge : edges) {
			auto planes = edge.planes;
			if (planes.find(id) != planes.end()) { segments.push_back(edge.segment); }
		}

		std::vector<Polygon_2> polygons = segments_to_polygons(&plane, &segments);
		n += int(polygons.size());
	}

	std::cout << "VERTICES: " << mesh_vertices.size() << std::endl;
	std::cout << "EDGES: " << edges.size() << std::endl;
	std::cout << "FACES: " << n << std::endl;

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
std::vector<Plane_intersection> Simplification::split_edges(std::vector<Plane_intersection>* segments, std::vector<Triple_intersection>* points) {
	std::vector<Plane_intersection> edges;

	// For each segment
	for (auto segment : *segments) {
		// Retrieve supporting planes
		auto segment_planes = segment.planes;

		// Find vertices to split edge
		std::vector<Point_3> splitters;
		// For each vertex
		for (auto point : *points) {
			// Retrieve supporting planes
			auto point_planes = point.planes;

			// Find common supporting planes with edge
			std::vector<int> common_planes;
			for (auto plane : point_planes) {
				auto pos = std::find(segment_planes.begin(), segment_planes.end(), plane);
				if (pos != segment_planes.end()) { common_planes.push_back(plane); }
			}

			// Two common planes ==
			// Vertex lies on edge => split edge!
			if (common_planes.size() == 2) { splitters.push_back(point.point); }
		}

		// Retrieve edge endpoints
		Point_3 source = segment.segment.source();
		Point_3 target = segment.segment.source();

		// Sort split points by distance to source
		std::sort(splitters.begin(), splitters.end(), 
			      [&](const Point_3 &pt1, const Point_3 &pt2) {
			      return CGAL::squared_distance(source, pt1) < 
				         CGAL::squared_distance(source, pt2); });

		// Split edge into edges
		for (auto i = 0; i < splitters.size(); i++) {
			int j = i + 1;
			if (j >= splitters.size()) { break; }

			// Define plane intersection
			Plane_intersection edge;

			// Add geometry
			source = splitters[i];
			target = splitters[j];
			edge.segment = Segment_3(splitters[i], splitters[j]);

			// Add supporting plane ==
			// The same as of the original segment!
			edge.planes.insert(segment_planes.begin(), segment_planes.end());

			edges.push_back(edge);
		}
	}

	return edges;
}