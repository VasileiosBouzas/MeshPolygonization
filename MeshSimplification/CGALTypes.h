#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/centroid.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/bounding_box.h>


// GEOMETRY //
typedef CGAL::Simple_cartesian<double>                      Kernel;

// 2D //
typedef Kernel::Point_2                                     Point_2;
typedef Kernel::Segment_2                                   Segment_2;
typedef Kernel::Triangle_2                                  Triangle_2;
typedef Kernel::Vector_2                                    Vector_2;
typedef CGAL::Polygon_2<Kernel>                             Polygon_2;
// 2D //


// 3D //
typedef Kernel::Point_3                                     Point_3;
typedef Kernel::Segment_3                                   Segment_3;
typedef Kernel::Plane_3                                     Plane_3;
typedef Kernel::Line_3                                      Line_3;
typedef CGAL::Bbox_3                                        Bbox_3;
typedef Kernel::Vector_3                                    Vector_3;
// 3D //
// GEOMETRY //


// SURFACE_MESH //
typedef CGAL::Surface_mesh<Point_3>                         Mesh;
typedef Mesh::Vertex_index                                  Vertex;
typedef Mesh::Face_index                                    Face;
typedef Mesh::Edge_index                                    Edge;
typedef Mesh::Halfedge_index                                Halfedge;
typedef CGAL::Vertex_around_target_circulator<Mesh>         Vertex_around_target_circulator;
typedef CGAL::Face_around_target_circulator<Mesh>           Face_around_target_circulator;
typedef CGAL::Halfedge_around_face_circulator<Mesh>         Halfedge_around_face_circulator;
typedef Mesh::Property_map<Vertex, Point_3>                 VProp_geom;
typedef Mesh::Property_map<Vertex, int>                     VProp_int;
typedef Mesh::Property_map<Vertex, double>                  VProp_double;
typedef Mesh::Property_map<Face, int>                       FProp_int;
typedef Mesh::Property_map<Face, double>                    FProp_double;
typedef Mesh::Property_map<Face, Point_3>                   FProp_color;
typedef Mesh::Property_map<Face, Vector_3>                  FProp_normal;
typedef CGAL::Face_filtered_graph<Mesh>                     Filtered_graph;
// SURFACE_MESH //


// GRAPH //
struct GraphVertex {
	unsigned int segment;
};

typedef boost::adjacency_list<boost::setS,
	                          boost::vecS,
	                          boost::undirectedS,
                              GraphVertex> Graph;

typedef Graph::vertex_descriptor Graph_vertex;
typedef Graph::vertex_iterator   Graph_vertex_iterator;
typedef Graph::edge_descriptor   Graph_edge;
typedef Graph::edge_iterator     Graph_edge_iterator;
// GRAPH //


// 2D ARRANGEMENT //
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>

typedef CGAL::Arr_segment_traits_2<Kernel>     Traits_2;
typedef CGAL::Arrangement_2<Traits_2>          Arrangement_2;
typedef Traits_2::X_monotone_curve_2		   X_monotone_curve_2;
typedef Arrangement_2::Face_iterator           Face_iterator;
typedef Arrangement_2::Ccb_halfedge_circulator Ccb_halfedge_circulator;
// 2D ARRANGEMENT //


// TRIPLE INTERSECTION //
// Intersection of plane triplet
struct Triple_intersection {
	// Geometry
	Point_3 point;

	// Supporting planes
	std::set<int> planes;
};
// TRIPLE INTERSECTION //


// PLANE INTERSECTION //
// Intersecting line of two planes
struct Plane_intersection {
	// Geometry
	Segment_3 segment;

	// Vertices
	std::vector<int> vertices;

	// Faces
	std::vector<int> faces;

	// Adjacency fan
	std::vector<Face> fan;

	// Supporting planes
	std::set<int> planes;
};
// PLANE INTERSECTION //


// CANDIDATE FACE //
struct Candidate_face {
	// Geometry
	std::vector<Point_3> points;

	// 2D Geometry - To compute confidence
	Polygon_2 polygon;

	// Vertices
	std::vector<int> vertices;

	// Edges
	std::vector<int> edges;

	// Number of supporting faces
	std::size_t supporting_face_num;

	// Covered area
	double covered_area;

	// Total area
	double area;

	// Supporting plane
	int plane;
};
// CANDIDATE FACE //