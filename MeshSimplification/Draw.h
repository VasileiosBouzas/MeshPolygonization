#pragma once

#include "Utils.h"

// Draw clipped lines
inline void draw_segments(std::vector<Segment_2>* segments, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "segments/seg_" << id << ".obj";
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


// Draw polygon
inline void draw_polygon(std::vector<Segment_2>* segments, unsigned int id, int n) {
	// Open file
	std::ostringstream oss;
	oss << "polygons/pol_" << id << "_" << n << ".obj";
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