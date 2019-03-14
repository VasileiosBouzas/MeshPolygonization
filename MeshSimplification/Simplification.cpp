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

	// Split & refine mesh edges
	std::vector<Plane_intersection> edges = split_edges(&segments, &vertices);
	refine_edges(&edges, &vertices, &plane_map);

	draw_frame(&vertices, &edges);

	// Compute mesh faces
	std::vector<Candidate_face> faces = compute_mesh_faces(mesh, G, &plane_map, &edges);

	Mesh simplified_mesh;
	for (auto face : faces) {
		std::vector<Vertex> fv;
		for (auto i : face.vertices) {
			auto v = simplified_mesh.add_vertex(vertices[i].point);
			fv.push_back(v);
		}
		simplified_mesh.add_face(fv);
	}

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

	e2.vertices.push_back(e->vertices[1]);
	e2.vertices.push_back(idx);

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
	/*bool splitted = true;
	do {
		splitted = false;
*/
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

						break;

						/*splitted = true; break;*/
					}
				}
			}

			/*if (splitted) break;*/
		}
	/*} while (splitted);*/
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