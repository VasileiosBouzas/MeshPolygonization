#include "Planarity.h"


Planarity::Planarity()
{
}


Planarity::~Planarity()
{
}


void Planarity::compute(Mesh* mesh, unsigned int num_rings) {
	// Create planarity attribute
	VProp_double planarity = mesh->add_property_map<Vertex, double>("v:planarity", -9999).first;

	// Compute planarity for vertices
	for (auto v : mesh->vertices()) {
		planarity[v] = compute_k_ring_planarity(mesh, v, num_rings);
	}

	// Assign planarity to faces
	planarity_to_faces(mesh);
}


// K-Ring Neighborhood Planarity
double Planarity::compute_k_ring_planarity(const Mesh* mesh, const Vertex vertex, unsigned int k) {
	// Collect k-ring neighbors
	std::set<Vertex> vertices = get_k_ring_vertices(mesh, vertex, k);

	// Retrieve geometry of neighbors
	std::vector<Point> points;
	VProp_geom geom = mesh->points();
	for (auto v : vertices) {
		points.push_back(geom[v]);
	}

	// Calculate planarity
	Plane plane;
	double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

	return planarity;
}


// Assign planarity to faces
void Planarity::planarity_to_faces(Mesh* mesh) {
	// Retrieve planarity property for vertices
	VProp_double v_planar = mesh->property_map<Vertex, double>("v:planarity").first;

	// Create planarity property for faces
	FProp_double f_planar = mesh->add_property_map<Face, double>("f:planarity").first;

	std::vector<Vertex> vertices;
	for (auto face : mesh->faces()) {
		// Retrieve face vertices
		vertices = vertex_around_face(mesh, face);

		// Iterate vertices
		for (auto vertex : vertices) {
			f_planar[face] += v_planar[vertex];
		}

		// Assign planarity to face
		f_planar[face] /= vertices.size();
	}
}