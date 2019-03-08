#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
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
	std::vector<Plane_intersection> segments = compute_mesh_edges(&bbox, G, &plane_map, &vertices);

	draw_frame(&vertices, &segments);

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
std::vector<Plane_intersection> Simplification::compute_mesh_edges(const Bbox_3* bbox, const Graph* G, std::map<unsigned int, Plane_3>* plane_map, std::vector<Triple_intersection>* points) {
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
				if (segment.planes == existing.planes) {
					found = true; break;
				}
			}

			// If not, add
			if (!found) { segments.push_back(segment); }
		}
	}

	std::cout << segments.size() << std::endl;
	for (auto segment : segments) {
		for (auto plane : segment.planes) {
			std::cout << plane << ", ";
		}
		std::cout << std::endl;
	}

	std::vector<Plane_intersection> finales;
	for (auto segment : segments) {
		auto segment_planes = segment.planes;
		std::vector<Point_3> splitters;
		for (auto point : *points) {
			auto point_planes = point.planes;

			std::vector<int> common_planes;
			for (auto plane : point_planes) {
				auto pos = std::find(segment_planes.begin(), segment_planes.end(), plane);
				if (pos != segment_planes.end()) { common_planes.push_back(plane); }
			}

			if (common_planes.size() == 2) { splitters.push_back(point.point); }
		}

		Point_3 source = segment.segment.source();
		Point_3 target = segment.segment.source();

		std::sort(splitters.begin(), splitters.end(), [&](const Point_3 &pt1, const Point_3 &pt2) {
			return CGAL::squared_distance(source, pt1) < CGAL::squared_distance(source, pt2);
		});

		for (auto i = 0; i < splitters.size(); i++) {
			for (auto j = i + 1; j < splitters.size(); j++) {
				Plane_intersection finale;

				source = splitters[i];
				target = splitters[j];
				finale.segment = Segment_3(splitters[i], splitters[j]);

				finale.planes.insert(segment_planes.begin(), segment_planes.end());

				finales.push_back(finale);
			}
		}
	}
	std::cout << finales.size() << std::endl;
	return finales;
}