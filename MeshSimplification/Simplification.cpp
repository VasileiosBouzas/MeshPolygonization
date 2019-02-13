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
	// Plane map
	std::map<unsigned int, Plane_3> plane_map = compute_planes(mesh, G);
	
	// Compute mesh bbox
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Traverse structure graph
	unsigned int id;
	Plane_3 plane;
	std::vector<Line_3> lines, bbox_lines;
	std::vector<Segment_3> segments;

	// For each segment
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;
		plane = plane_map[id];

		// Supporting-adjacent plane intersections
		lines = compute_intersections(G, *vb, &plane_map);

		// Supporting-bbox plane intersections
		bbox_lines = compute_bbox_intersections(&bbox, &plane);
		lines.insert(lines.end(), bbox_lines.begin(), bbox_lines.end());

		// Clip lines with bbox
		segments = clip_lines(&lines, &bbox);

		// Segments to polygons
		//segments_to_polygons(&plane, &segments);

		// Draw segments
		draw_segments(&segments, id);
	}
	
	// Define simplified mesh
	Mesh simplified_mesh;

	return simplified_mesh;
}