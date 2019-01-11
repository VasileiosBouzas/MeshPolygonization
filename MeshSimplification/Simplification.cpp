#include "Simplification.h"



Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G) {
	// Plane map
	std::map<unsigned int, Plane_3> plane_map = compute_planes(mesh, G);

	// Define mesh bbox
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Define simplified mesh
	Mesh simplified_mesh;
	std::vector<Point_3> points;
	Vertex simplified_vertex;
	std::vector<Vertex> simplified_vertices;
	
	// Traverse structure graph
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		points = compute_intersections(G, *vb, &plane_map);

		// Define face vertices
		for (auto point : points) {
			if (is_in_bbox(bbox, point)) {
				simplified_vertex = simplified_mesh.add_vertex(point);
				simplified_vertices.push_back(simplified_vertex);
			}
		}

		// Define face
		// Assert the face is at least rectangular
		if (simplified_vertices.size() >= 4) { 
			simplified_mesh.add_face(simplified_vertices); 
		}

		simplified_vertices.clear();
	}

	//// Remove any isolated vertices
	//for (auto vertex : simplified_mesh.vertices()) {
	//	if (simplified_mesh.is_isolated(vertex)) {
	//		simplified_mesh.remove_vertex(vertex);
	//	}
	//}

	std::cout << "#vertices: " << simplified_mesh.num_vertices() << std::endl;
	std::cout << "#faces: " << simplified_mesh.num_faces() << std::endl;

	return simplified_mesh;
}


// Compute supporting planes of segments
std::map<unsigned int, Plane_3> Simplification::compute_planes(const Mesh* mesh, const Graph* G) {
	std::map<unsigned int, Plane_3> plane_map;

	// Compute planes for segments
	Graph_vertex_iterator vb, ve;
	unsigned int id;
	std::set<Face> segment;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Vertex to segment
		id = (*G)[*vb].segment;

		// Select segment by id
		segment = StructureGraph::select_segment(mesh, id);

		// Compute plane for segment
		plane_map[id] = fit_plane_to_faces(mesh, &segment);
	}

	return plane_map;
}


// Compute plane intersections
std::vector<Point_3> Simplification::compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::vector<Point_3> points;

	unsigned int curr_id, adj_id, other_id;
	Plane_3 curr_plane, adj_plane, other_plane;

	// Vertex to plane
	curr_id = (*G)[v].segment;
	curr_plane = (*plane_map)[curr_id];

	// Find adjacent vertices
	auto adjacent = boost::adjacent_vertices(v, *G);

	// For every adjacent
	for (auto adj : make_iterator_range(adjacent)) {
		// Vertex to plane
		adj_id = (*G)[adj].segment;
		adj_plane = (*plane_map)[adj_id];

		// Find adjacent segments of adjacent
		auto others = boost::adjacent_vertices(adj, *G);

		// For every other plane
		for (auto other : make_iterator_range(others)) {
			// Check if edge exists
			if (adj < other && boost::edge(v, other, *G).second) {
				// Vertex to plane
				other_id = (*G)[other].segment;
				other_plane = (*plane_map)[other_id];

				// Compute intersection fo three planes
				auto inter = CGAL::intersection(curr_plane, adj_plane, other_plane);

				// Handle intersection
				if (inter != boost::none) {
					if (const Point_3* pt = boost::get<Point_3>(&(*inter))) {points.push_back(*pt);}
				}
			}
		}
	}

	return points;
}