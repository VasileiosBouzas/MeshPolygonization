#include "StructureGraph.h"
#include "Segment.h"


StructureGraph::StructureGraph()
{
}


StructureGraph::~StructureGraph()
{
}


Graph StructureGraph::construct(Mesh* mesh, std::size_t seg_number, double imp_thres) {
	// Select important segments
	std::set<unsigned int> important_segments = compute_importance(mesh, seg_number, imp_thres);

	// Construct structure graph
	Graph structure_graph = construct_structure_graph(mesh, &important_segments);

	return structure_graph;
}


// Compute importance
std::set<unsigned int> StructureGraph::compute_importance(Mesh* mesh, std::size_t seg_number, double imp_thres) {
	std::set<unsigned int> important_segments;

	// Define importance attribute
	FProp_double imp = mesh->add_property_map<Face, double>("f:imp", -1).first;

	// Calculate total area of mesh
	double total_area = CGAL::Polygon_mesh_processing::area(*mesh);

	// Calculate segment importance
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	Filtered_graph segment_graph(*mesh, 0, chart);
	double area, importance;
	std::set<Face> segment;
	for (std::size_t id = 0; id < seg_number; id++) {
		// Select segment
		segment_graph.set_selected_faces(id, chart);
		
		// Compute area
		area = CGAL::Polygon_mesh_processing::area(segment_graph);

		// Compute importance
		importance = area / total_area * 100;

		// Select segment by id
		segment = select_segment(mesh, id);

		// Assign importance to faces
		for (auto face : segment) {
			imp[face] = importance;
		}

		// Check if important
		if (importance > imp_thres) { important_segments.insert(id); }
	}

	return important_segments;
}


// Collect adjacent segments
std::set<unsigned int> StructureGraph::get_adjacent_segments(const Mesh* mesh, unsigned int id) {
	std::set<unsigned int> adjacent;

	// Select segment
	std::set<Face> segment = select_segment(mesh, id);

	// Define chart attribute
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;

	// Iterate faces
	std::vector<Vertex> vertices;
	std::set<Face> new_faces, faces;
	for (auto face : segment) {
		// Collect vertices
		vertices = vertex_around_face(mesh, face);

		// Collect neighboring faces
		for (auto vertex : vertices) {
			new_faces = get_k_ring_faces(mesh, vertex, 1);
			faces.insert(new_faces.begin(), new_faces.end());
		}

		// Iterate faces
		for (auto opp_face : faces) {
			// Check chart
			if (chart[face] != chart[opp_face]) {
				// Different charts = different segment!
				adjacent.insert(chart[opp_face]);
			}
		}

		faces.clear();
	}

	return adjacent;
}


// Convert segment to vertex
std::size_t StructureGraph::segment_to_vertex(const Graph* G, unsigned int id) {
	Graph_vertex_iterator vb, ve;
	for (boost::tie(vb, ve) = vertices(*G); vb != ve; ++vb) {
		if ((*G)[*vb].segment == id) { return *vb; }
	}
}


// Construct structure graph
Graph StructureGraph::construct_structure_graph(const Mesh* mesh, std::set<unsigned int>* segments) {
	Graph G;

	// Assign segment to vertex
	for (auto segment : *segments) {
		// Add vertex to graph
		boost::add_vertex(GraphVertex{segment}, G);
	}

	// Add graph edges
	std::set<unsigned int> adjacent;
	for (auto segment : *segments) {
		adjacent = get_adjacent_segments(mesh, segment);
		for (auto adj : adjacent) {
			// Check if edge exists and adjacent is important
			if (segment < adj && segments->find(adj) != segments->end()) {
				boost::add_edge(segment_to_vertex(&G, segment),
					            segment_to_vertex(&G, adj),
					            G);
			}
		}
	}

	return G;
}
