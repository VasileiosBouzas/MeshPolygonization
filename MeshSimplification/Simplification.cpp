#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
#include "LinesToPolygons.h"
#include "Overlap.h"
#include "Draw.h"

Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G) {
	// Plane map
	std::map<unsigned int, Plane_3> plane_map = compute_planes(mesh, G);

	// Compute bbox of original mesh
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Define simplified mesh
	Mesh simplified_mesh;
	Vertex simplified_vertex;
	std::vector<Vertex> simplified_vertices;

	// Traverse structure graph
	unsigned int id;
	Plane_3 plane;
	std::vector<Line_3> lines, bbox_lines;
	std::vector<Segment_3> segments;
	std::vector<Polygon_2> polygons;
	std::vector<Point_3> points;

	// For each segment
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = plane_map[id];

		// Supporting-adjacent plane intersections
		lines = compute_intersections(G, *vb, &plane_map);

		// Supporting-bbox plane intersections
		//bbox_lines = compute_bbox_intersections(&bbox, &plane);
		//lines.insert(lines.end(), bbox_lines.begin(), bbox_lines.end());

		// Clip lines with bbox
		segments = clip_lines(&lines, &bbox);

		// Segments to 2D polygons
		polygons = segments_to_polygons(&plane, &segments, id);

		// Define simplified face
		points = define_face(mesh, id, &plane, &polygons);

		if (points.size() >= 3) {
			for (auto point : points) {
				simplified_vertex = simplified_mesh.add_vertex(point);
				simplified_vertices.push_back(simplified_vertex);
			}
			simplified_mesh.add_face(simplified_vertices);
		}

		simplified_vertices.clear();
	}

	return simplified_mesh;
}