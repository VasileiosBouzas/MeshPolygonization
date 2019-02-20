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

		// Recover polygon vertices
		std::set<Point_2> vertices;
		for (Polygon_2::Edge_const_iterator e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
			vertices.insert(e->source());
			vertices.insert(e->target());
		}

		// Store polygon vertices
		std::map<Point_2, int> vertex_map;
		int num = 1;
		for (auto vertex : vertices) {
			vertex_map[vertex] = num;
			num++;
			os << "v " << vertex.x() << " " << vertex.y() << " " << 0.0 << std::endl;
		}

		// Store polygon edges
		for (Polygon_2::Edge_const_iterator e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
			os << "l " << vertex_map[e->source()] << " " << vertex_map[e->target()] << std::endl;
		}

		// Close file
		os.close();
		n++;
	}
}


// Draw mesh segment
inline void draw_mesh_segment(std::vector<Polygon_2>* faces, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "draw/segments/seg_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Collect segment points
	std::set<Point_2> points;
	for (auto face : *faces) {
		// Recover faces vertices
		for (Polygon_2::Vertex_const_iterator v = face.vertices_begin(); v != face.vertices_end(); ++v) {
			// Insert vertex
			points.insert(*v);
		}
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
	for (auto face : *faces) {
		// Recover face edges
		for (Polygon_2::Edge_const_iterator e = face.edges_begin(); e != face.edges_end(); ++e) {
			os << "l " << vertex_map[e->source()] << " " << vertex_map[e->target()] << std::endl;
		}
	}

	// Close file
	os.close();
}


// Draw simplified face
inline void draw_face(Polygon_2* polygon, unsigned int id) {
	// Open file
	std::ostringstream oss;
	oss << "draw/faces/f_" << id << ".obj";
	std::ofstream os(oss.str().data());

	// Recover polygon vertices
	std::set<Point_2> vertices;
	for (Polygon_2::Edge_const_iterator e = polygon->edges_begin(); e != polygon->edges_end(); ++e) {
		vertices.insert(e->source());
		vertices.insert(e->target());
	}

	// Store polygon vertices
	std::map<Point_2, int> vertex_map;
	int num = 1;
	for (auto vertex : vertices) {
		vertex_map[vertex] = num;
		num++;
		os << "v " << vertex.x() << " " << vertex.y() << " " << 0.0 << std::endl;
	}

	// Store polygon edges
	for (Polygon_2::Edge_const_iterator e = polygon->edges_begin(); e != polygon->edges_end(); ++e) {
		os << "l " << vertex_map[e->source()] << " " << vertex_map[e->target()] << std::endl;
	}

	// Close file
	os.close();
}