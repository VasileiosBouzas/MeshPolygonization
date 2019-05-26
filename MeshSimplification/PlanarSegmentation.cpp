#include "PlanarSegmentation.h"
#include "Segment.h"


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
	FProp_color color = mesh->add_property_map<Face, Point_3>("f:color", Point_3(0, 0, 0)).first;

	double dist = std::pow(dist_thres, 2);
	Point_3 current_color = random_color();
	int current_index = 0;
	std::size_t seg_number = 0;
	std::map<int, Plane_3> plane_map;
	std::map<int, std::set<Face>> segment_map;

	while (faces.size() != 0) {
		// Locate vertex with highest planarity
		Face max_face = get_max_planarity_face(mesh, &faces);

		// Initialize current region
		std::set<Face> current_region;
		std::set<Face> seeds;

		// Calculate initial plane
		std::set<Face> neighbors = get_k_ring_faces(mesh, max_face, num_rings);
		Plane_3 plane = fit_plane_to_faces(mesh, &neighbors);

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
				bool is_fitting = check_distance(mesh, &neighbor, &plane, dist);
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
		VProp_geom geom = mesh->points();
		for (auto face : current_region) {
			std::vector<Vertex> fc = vertex_around_face(mesh, face);
			for (auto v : fc) {
				Point_3 proj = plane.projection(geom[v]);
				geom[v] = proj;
			}
		}

		// Store
		plane_map[current_index] = plane;
		segment_map[current_index] = current_region;

		// Initialize search for next region
		current_region.clear();
		current_index += 1;
		seg_number += 1;
		current_color = random_color();
	}

	// Refine segmentation
	std::cout << "Number of planes: " << seg_number << std::endl;
	seg_number = refine_segmentation(mesh, seg_number, &plane_map, &segment_map, dist);

	return seg_number;
}


// Locate face with highest planarity
Face PlanarSegmentation::get_max_planarity_face(const Mesh* mesh, std::set<Face>* faces) {
	FProp_double planarity = mesh->property_map<Face, double>("f:planarity").first;
	auto max_face = std::max_element(faces->begin(), faces->end(),
					[&](const Face &a, const Face &b)
					{return planarity[a] < planarity[b]; });

	return *max_face;
}


// Check distance
bool PlanarSegmentation::check_distance(const Mesh* mesh, Face* face, Plane_3* plane, double dist) {
	// Collect vertices for face
	std::vector<Vertex> vertices = vertex_around_face(mesh, *face);

	VProp_geom geom = mesh->points();
	for (auto vertex : vertices) {
		if (squared_distance(geom[vertex], *plane) > dist) { return false; }
	}

	return true;
}


// Assign color to each region
Point_3 PlanarSegmentation::random_color() {
	int R = -1; int G = -1; int B = -1;
	while (R < 100) { R = rand() % 256; }
	while (G < 100) { G = rand() % 256; }
	while (B < 100) { B = rand() % 256; }
	return Point_3(R, G, B);
}


// Check segment fitting
bool PlanarSegmentation::check_fitting(Mesh* mesh, std::set<Face>* segment, Plane_3* plane, double dist) {
	// Iterate segment faces
	int n = 0;
	for (auto face : *segment) {
		if (check_distance(mesh, &face, plane, dist)) n++;
	}

	// Good fitting >= 20%
	double fitting = n / double(segment->size());
	if (fitting >= 0.2) { return true; }
	return false;
}


// Merge segments
int PlanarSegmentation::merge_segments(Mesh* mesh, std::size_t seg_number, std::set<Face>* seg_1, std::set<Face>* seg_2) {
	// Retrieve all face properties
	FProp_int chart = mesh->property_map<Face, int>("f:chart").first;
	FProp_color color = mesh->property_map<Face, Point_3>("f:color").first;

	// Introduce new segment
	int new_id = int(seg_number);
	Point_3 new_color = random_color();

	// Change first segment
	for (auto face : *seg_1) {
		// Change chart
		chart[face] = new_id;

		// Change color
		color[face] = new_color;
	}

	// Change second segment
	for (auto face : *seg_2) {
		chart[face] = new_id;
		color[face] = new_color;
	}

	return new_id;
}


// Refine segmentation
std::size_t PlanarSegmentation::refine_segmentation(Mesh* mesh, std::size_t seg_number, std::map<int, Plane_3>* plane_map, std::map<int, std::set<Face>>* segment_map, double dist) {
	// Define plane angle threshold
	double theta = 10.0 * CGAL_PI / 180.0;

	// Collect segment indices
	std::vector<int> segments;
	for (auto id = 0; id < seg_number; id++) {
		segments.push_back(id);
	}

	// Iterate segments
	int id_1, id_2;
	std::set<Face> seg_1, seg_2;
	Plane_3 plane_1, plane_2;
	Vector_3 n1, n2;

	bool merged = false;
	do {
		merged = false;

		// Sort segments in ascending order of faces
		std::sort(segments.begin(), segments.end(),
				  [&](const int& id_1, const int& id_2)
			      {return (*segment_map)[id_1] < (*segment_map)[id_2]; });

		// Iterate segments
		for (auto i = 0; i < segments.size(); i++) {
			// Retrieve segment
			id_1 = segments[i];
			seg_1 = (*segment_map)[id_1];
			plane_1 = (*plane_map)[id_1];
			n1 = plane_1.orthogonal_vector();
			n1 = n1 / std::sqrt(n1.squared_length()); // Normalize

			// Check all other segments
			for (auto j = i + 1; j < segments.size(); j++) {
				// Retrieve segment
				id_2 = segments[j];
				seg_2 = (*segment_map)[id_2];
				plane_2 = (*plane_map)[id_2];
				n2 = plane_2.orthogonal_vector();
				n2 = n2 / std::sqrt(n2.squared_length()); // Normalize

				// Check plane angle
				if (std::abs(n1*n2) > std::cos(theta)) {
					// Check fitting
					if (check_fitting(mesh, &seg_1, &plane_2, dist) ||
						check_fitting(mesh, &seg_2, &plane_1, dist)) {
						// Merge
						int new_id = merge_segments(mesh, seg_number, &seg_1, &seg_2);

						// Update
						segments.push_back(new_id);
						(*segment_map)[new_id].insert(seg_1.begin(), seg_1.end());
						(*segment_map)[new_id].insert(seg_2.begin(), seg_2.end());
						(*plane_map)[new_id] = fit_plane_to_faces(mesh, &(*segment_map)[new_id]);

						// Delete
						auto pos_1 = std::find(segments.begin(), segments.end(), id_1);
						segments.erase(pos_1);
						segment_map->erase(id_1);
						plane_map->erase(id_1);

						auto pos_2 = std::find(segments.begin(), segments.end(), id_2);
						segments.erase(pos_2);
						segment_map->erase(id_2);
						plane_map->erase(id_2);

						// Increase segment number
						seg_number += 1;

						// Stop search
						merged = true; break;
					}
				}
			}
			if (merged) break;
		}
	} while (merged);

	return seg_number;
}