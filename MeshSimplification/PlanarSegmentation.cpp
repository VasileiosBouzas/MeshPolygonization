#include "PlanarSegmentation.h"


PlanarSegmentation::PlanarSegmentation()
{
}


PlanarSegmentation::~PlanarSegmentation()
{
}


std::size_t PlanarSegmentation::apply(Mesh* mesh, double dist_thres, unsigned int num_rings) {
	// Collect mesh vertices
	std::set<Face> faces;
	for (auto f : mesh->faces()) {
		faces.insert(f);
	}

	// Create chart attribute
	FProp_int chart = mesh->add_property_map<Face, int>("f:chart", -1).first;
	FProp_color color = mesh->add_property_map<Face, Point>("f:color", Point(0, 0, 0)).first;

	double dist = std::pow(dist_thres, 2);
	Point current_color = random_color();
	int current_index = 1;
	std::size_t seg_number = 0;
	while (faces.size() != 0) {
		// Locate vertex with highest planarity
		Face max_face = get_max_planarity_face(mesh, &faces);

		// Initialize current region
		std::set<Face> current_region;
		std::set<Face> seeds;

		// Calculate initial plane
		std::set<Face> neighbors = get_k_ring_faces(mesh, max_face, num_rings);
		Plane plane = fit_plane_to_faces(mesh, &neighbors);

		// Update
		faces.erase(max_face);
		seeds.insert(max_face);

		while (seeds.size() != 0) {
			// Collect 1-ring faces for all seeds
			neighbors.clear();
			std::set<Face> new_neighbors;
			for (auto seed : seeds) {
				// Collect faces
				new_neighbors = get_k_ring_faces(mesh, seed, 1);
				neighbors.insert(new_neighbors.begin(), new_neighbors.end());

				// End search for current seed
				seeds.erase(seed);
			}

			// Check neighboring faces
			for (auto neighbor : neighbors) {
				// Check visited
				bool is_visited = chart[neighbor] != -1;
				if (is_visited) continue;

				// Check distance
				bool is_fitting = check_distance(mesh, neighbor, plane, dist);
				if (is_fitting) {
					// Add face to current region
					current_region.insert(neighbor);
					chart[neighbor] = current_index;
					color[neighbor] = current_color;

					// Update
					faces.erase(neighbor);
					seeds.insert(neighbor);
				}
			}

			// Calculate plane for current region
			plane = fit_plane_to_faces(mesh, &current_region);
		}

		// Project vertices on supporting plane
		/*VProp_geom geom = mesh->points();
		for (auto face : current_region) {
			std::vector<Vertex> fc = vertex_around_face(mesh, face);
			for (auto v : fc) {
				Point proj = plane.projection(geom[v]);
				geom[v] = proj;
			}
		}*/

		// Initialize search for next region
		current_region.clear();
		current_index += 1;
		seg_number += 1;
		current_color = random_color();
	}

	return seg_number;
}


// Locate face with highest planarity
Face PlanarSegmentation::get_max_planarity_face(const Mesh* mesh, std::set<Face>* faces) {
	FProp_double planarity = mesh->property_map<Face, double>("f:planarity").first;
	auto max_face = std::max_element(faces->begin(), faces->end(),
		                             [&](const Face &a, const Face &b)
	                                 {return planarity[a] < planarity[b];});

	return *max_face;
}


// Check distance
bool PlanarSegmentation::check_distance(const Mesh* mesh, Face face, Plane plane, double dist) {
	// Collect vertices for face
	std::vector<Vertex> vertices = vertex_around_face(mesh, face);

	bool is_fitting;
	VProp_geom geom = mesh->points();
	for (auto vertex : vertices) {
		is_fitting = squared_distance(geom[vertex], plane) < dist;
		if (is_fitting == false) break;
	}

	return is_fitting;
}


// Assign color to each region
Point PlanarSegmentation::random_color() {
	int R = -1; int G = -1; int B = -1;
	while (R < 100) { R = rand() % 256; }
	while (G < 100) { G = rand() % 256; }
	while (B < 100) { B = rand() % 256; }
	return Point(R, G, B);
}