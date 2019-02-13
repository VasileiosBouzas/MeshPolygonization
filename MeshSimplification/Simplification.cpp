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
	std::vector<Line_3> lines;
	std::vector<Segment_3> segments;
	Graph_vertex_iterator vb, ve;

	// For each segment
	unsigned int id;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;

		// Supporting-adjacent plane intersections
		lines = compute_intersections(G, *vb, &plane_map);

		// Clip lines with bbox
		segments = clip_lines(&lines, &bbox);

		// Draw segments
		draw_segments(&segments, id);
	}
	
	// Define simplified mesh
	Mesh simplified_mesh;

	return simplified_mesh;
}