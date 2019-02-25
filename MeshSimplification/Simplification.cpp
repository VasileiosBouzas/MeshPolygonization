#include "Simplification.h"
#include "Segment.h"
#include "Intersection.h"
#include "LinesToPolygons.h"
#include "Overlap.h"
#include "Orientation.h"
#include "Draw.h"

Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G, std::string filename) {
	// Plane map
	std::map<unsigned int, Plane_3> plane_map = compute_planes(mesh, G);

	// Compute bbox of original mesh
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Define simplified mesh
	std::vector<Point_3> existing;
	std::vector<std::size_t> face;
	std::vector<std::vector<std::size_t>> faces;
	std::map<std::size_t, unsigned int> face_to_segment;

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
		// lines.insert(lines.end(), bbox_lines.begin(), bbox_lines.end());

		// Clip lines with bbox
		segments = clip_lines(&lines, &bbox);

		// Segments to 2D polygons
		polygons = segments_to_polygons(&plane, &segments, id);

		// Define simplified face
		points = define_face(mesh, id, &plane, &polygons);

		// At least 3 points to define a face
		if (points.size() >= 3) {
			for (auto point : points) {
				// Check if point already exists
				bool found = false;
				for (std::size_t i = 0; i < existing.size(); i++) {
					// Check in vicinity
					if (point == existing[i]) {
						//If exists
						face.push_back(i);
						found = true; break;
					}
				}

				// If not
				if (!found) {
					// Add to existing
					existing.push_back(point);

					// Add to face the last existing point
					face.push_back(existing.size() - 1);
				}
			}

			// Add to faces
			faces.push_back(face);
			face_to_segment[faces.size() - 1] = id;
			face.clear();
		}
	}

	// Draw mesh skeleton
	// draw_skeleton(&existing, &faces, filename);

	// Define simplified mesh
	Mesh simplified_mesh;

	// Define vertices
	Vertex vertex;
	std::map<std::size_t, Vertex> point_to_vertex;
	for (std::size_t i = 0; i < existing.size(); i++) {
		vertex = simplified_mesh.add_vertex(existing[i]);
		point_to_vertex[i] = vertex;
	}

	// Define faces
	std::vector<Vertex> vertices;
	for (std::size_t i = 0; i < faces.size(); i++) {
		face = faces[i];

		// Recover face triangle
		Point_3 p = existing[face[0]];
		Point_3 q = existing[face[1]];
		Point_3 r = existing[face[2]];

		// Compute triangle normal
		Vector_3 normal = CGAL::normal(p, q, r);

		// Compute segment normal
		Vector_3 seg_normal = compute_segment_orientation(mesh, face_to_segment[i]);

		// Check face orientation
		if (CGAL::angle(normal, seg_normal) == CGAL::OBTUSE) {
			// If needed, reverse
			std::reverse(face.begin(), face.end());
		}

		// Collect vertices
		for (auto idx : face) {
			vertices.push_back(point_to_vertex[idx]);
		}

		// Define face
		simplified_mesh.add_face(vertices);
		vertices.clear();
	}

	return simplified_mesh;
}