#pragma once

#include "Utils.h"


// Draw triangulation
inline void draw_triangulation(Triangulation_2* DT, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "alphas/tri/tri_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Store triangulation vertices
	std::map<Point_2, int> vertex_map;
	int num = 1;
	for (auto it = DT->all_vertices_begin(); it != DT->all_vertices_end(); ++it) {
		vertex_map[it->point()] = num;
		num++;
		os << "v " << it->point().x() << " " << it->point().y() << " " << 0.0 << std::endl;
	}

	// Store triangulation edges
	Point_2 source, target;
	for (auto it = DT->all_edges_begin(); it != DT->all_edges_end(); ++it) {
		source = DT->segment(it).source();
		target = DT->segment(it).target();
		os << "l " << vertex_map[source] << " " << vertex_map[target] << std::endl;
	}

	// Close file
	os.close();
}


// Draw segment
inline void draw_segment(std::vector<Triangle_2>* triangles, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "alphas/seg/seg_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Collect segment points
	std::set<Point_2> points;
	Point_2 p, q, r;
	for (auto triangle : *triangles) {
		// Recover triangle vertices
		p = Point_2(triangle.vertex(0).x(), triangle.vertex(0).y());
		q = Point_2(triangle.vertex(1).x(), triangle.vertex(1).y());
		r = Point_2(triangle.vertex(2).x(), triangle.vertex(2).y());

		// Update
		points.insert(p);
		points.insert(q);
		points.insert(r);
	}

	// Store segment vertices
	std::map<Point_2, int> vertex_map;
	int num = 1;
	for (auto point : points) {
		vertex_map[point] = num;
		num++;
		os << "v " << point.x() << " " << point.y() << " " << 0.0 << std::endl;
	}

	// Store segment edges
	for (auto triangle : *triangles) {
		// Recover triangle vertices
		p = Point_2(triangle.vertex(0).x(), triangle.vertex(0).y());
		q = Point_2(triangle.vertex(1).x(), triangle.vertex(1).y());
		r = Point_2(triangle.vertex(2).x(), triangle.vertex(2).y());

		os << "l " << vertex_map[p] << " " << vertex_map[q] << std::endl;
		os << "l " << vertex_map[q] << " " << vertex_map[r] << std::endl;
		os << "l " << vertex_map[r] << " " << vertex_map[p] << std::endl;
	}

	// Close file
	os.close();
}


// Draw alpha shape
inline void draw_alpha_shape(std::vector<Segment_2>* segments, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "alphas/alphas/alpha_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Store alpha vertices
	std::set<Point_2> vertices;
	for (auto segment : *segments) {
		vertices.insert(segment.source());
		vertices.insert(segment.target());
	}

	std::map<Point_2, int> vertex_map;
	int num = 1;
	for (auto vertex : vertices) {
		vertex_map[vertex] = num;
		num++;
		os << "v " << vertex.x() << " " << vertex.y() << " " << 0.0 << std::endl;
	}

	// Store alpha edges
	for (auto segment : *segments) {
		os << "l " << vertex_map[segment.source()] << " " << vertex_map[segment.target()] << std::endl;
	}

	// Close file
	os.close();
}