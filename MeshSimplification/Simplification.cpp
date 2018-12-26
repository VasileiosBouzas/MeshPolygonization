#include "Simplification.h"



Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G) {
	// Plane map
	std::map<unsigned int, Plane> plane_map = compute_planes(mesh, G);

	// Define simplified mesh
	Mesh simplified;
	std::set<Point> points;
	Vertex simplified_vertex; std::set<Vertex> simplified_vertices;
	
	// Traverse structure graph
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		points = compute_intersections(G, *vb, &plane_map);
		
		/*std::cout << "Segment " << curr_id << std::endl;
		std::cout << "Number of points: " << points.size() << std::endl;
		for (auto point : points) {
			std::cout << "Point: " << point.x() << ", " << point.y() << ", " << point.z() << std::endl;
		} 
		std::cout << std::endl;*/

		// Define face vertices
		for (auto point : points) {
			simplified_vertex = simplified.add_vertex(point);
			simplified_vertices.insert(simplified_vertex);
		}

		// Define face
		// simplified.add_face(simplified_vertices);
		simplified_vertices.clear();
	}

	std::cout << "#vertices: " << simplified.num_vertices() << std::endl;
	std::cout << "#faces: " << simplified.num_faces() << std::endl;

	return simplified;
}


std::map<unsigned int, Plane> Simplification::compute_planes(const Mesh* mesh, const Graph* G) {
	std::map<unsigned int, Plane> plane_map;

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


std::set<Point> Simplification::compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane>* plane_map) {
	std::set<Point> points;

	unsigned int curr_id, adj_id, other_id;
	Plane curr_plane, adj_plane, other_plane;

	// Vertex to segment
	curr_id = (*G)[v].segment;

	// Find adjacent vertices
	auto adjacent = boost::adjacent_vertices(v, *G);

	// For every adjacent
	for (auto adj : make_iterator_range(adjacent)) {
		// Vertex to segment
		adj_id = (*G)[adj].segment;

		// Recover planes
		curr_plane = (*plane_map)[curr_id];
		adj_plane = (*plane_map)[adj_id];

		// Find adjacent segments of adjacent
		auto others = boost::adjacent_vertices(adj, *G);

		// For every other plane
		for (auto other : make_iterator_range(others)) {
			// Check if edge exists
			if (adj < other && boost::edge(v, other, *G).second) {
				// Vertex to segment
				other_id = (*G)[other].segment;

				// Recover third plane
				other_plane = (*plane_map)[other_id];

				// Compute intersection fo three planes
				auto inter = CGAL::intersection(curr_plane, adj_plane, other_plane);

				// Handle intersection
				if (inter != boost::none) {
					if (const Point* pt = boost::get<Point>(&(*inter))) {points.insert(*pt);}
				}
			}
		}
	}

	return points;
}