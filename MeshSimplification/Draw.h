#pragma once

#include "Utils.h"

// Draw line segments
inline void draw_line_segments(std::vector<Segment_2>* segments, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "draw/lines/l_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Store segment vertices
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


// Draw polygons
inline void draw_polygons(std::vector<Polygon_2>* polygons, unsigned int id) {
	int n = 1;
	for (auto polygon : *polygons) {
		// Open file
		std::ostringstream oss;
		oss << "draw/polygons/pol_" << id << "_" << n << ".obj";
		std::ofstream os(oss.str().data());

		// Store alpha vertices
		std::set<Point_2> vertices;
		for (Polygon_2::Edge_const_iterator e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
			vertices.insert(e->source());
			vertices.insert(e->target());
		}

		std::map<Point_2, int> vertex_map;
		int num = 1;
		for (auto vertex : vertices) {
			vertex_map[vertex] = num;
			num++;
			os << "v " << vertex.x() << " " << vertex.y() << " " << 0.0 << std::endl;
		}

		// Store alpha edges
		for (Polygon_2::Edge_const_iterator e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
			os << "l " << vertex_map[e->source()] << " " << vertex_map[e->target()] << std::endl;
		}

		// Close file
		os.close();
		n++;
	}
}