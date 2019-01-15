#include "Simplification.h"
#include "AlphaShape.h"


Simplification::Simplification()
{
}


Simplification::~Simplification()
{
}


Mesh Simplification::apply(const Mesh* mesh, const Graph* G, double dist_thres) {
	// Plane map
	std::map<unsigned int, Plane_3> plane_map = compute_planes(mesh, G);

	// Define mesh bbox
	Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(*mesh);

	// Define simplified mesh
	Mesh simplified_mesh;
	std::set<Point_3> points;
	Vertex simplified_vertex;
	std::vector<Vertex> simplified_vertices;
	
	// Traverse structure graph
	unsigned int id;
	Plane_3 plane;
	std::vector<Point_3> sorted, interior;
	VProp_geom geom = simplified_mesh.points();
	double dist = dist_thres * dist_thres;
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		// Compute intersections
		points = compute_intersections(G, *vb, &plane_map);

		// Check intersections
		for (auto point : points) {
			if (!is_in_bbox(bbox, point)) { points.erase(point); }
		}
		if (points.size() <= 2) { continue; }

		// Retrieve segment
		id = (*G)[*vb].segment;
		plane = (plane_map)[id];

		//// Retrieve interior points
		//interior = get_interior_points(mesh, id);
		//points.insert(interior.begin(), interior.end());

		// Sort points
		AlphaShape as;
		sorted = as.construct(points, plane);

		// Define face vertices
		for (auto point : sorted) {
			// Check if vertex already exists
			for (auto vertex : simplified_mesh.vertices()) {
				// If there is a nearby vertex
				if (CGAL::squared_distance(geom[vertex], point) < dist_thres) {
					// Use existent vertex
					point = geom[vertex]; break;
				}
			} 
			simplified_vertex = simplified_mesh.add_vertex(point);
			simplified_vertices.push_back(simplified_vertex);
		}

		// Define face
		// Assert the face is at least triangular
		if (simplified_vertices.size() >= 3) { 
			simplified_mesh.add_face(simplified_vertices); 
		}

		simplified_vertices.clear();
	}

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
std::set<Point_3> Simplification::compute_intersections(const Graph* G, const Graph_vertex v, std::map<unsigned int, Plane_3>* plane_map) {
	std::set<Point_3> points;

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
					if (const Point_3* pt = boost::get<Point_3>(&(*inter))) {points.insert(*pt);}
				}
			}
		}
	}

	return points;
}


//// Retrieve interior points
//std::vector<Point_3> Simplification::get_interior_points(const Mesh* mesh, unsigned int id) {
//	std::vector<Point_3> interior;
//
//	// Select segment by id
//	std::set<Face> segment = StructureGraph::select_segment(mesh, id);
//
//	// Iterate faces
//	std::vector<Vertex> new_vertices;
//	std::set<Vertex> vertices;
//	for (auto face : segment) {
//		// Retrieve face vertices
//		new_vertices = vertex_around_face(mesh, face);
//		vertices.insert(new_vertices.begin(), new_vertices.end());
//	}
//
//	// Retrieve points
//	std::vector<Face> neighbors;
//	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
//	VProp_geom geom = mesh->points();
//	for (auto vertex : vertices) {
//		bool is_in = true;
//
//		// Retrieve neighboring vertices
//		neighbors = face_around_vertex(mesh, vertex);
//
//		// Check neighbors
//		for (auto neighbor : neighbors) {
//			if (chart[neighbor] != id) {is_in = false; break;}
//		}
//
//		// Add point
//		if (is_in) { interior.push_back(geom[vertex]); }
//	}
//
//	return interior;
//}