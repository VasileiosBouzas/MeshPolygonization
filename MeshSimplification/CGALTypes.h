#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/centroid.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>


// GEOMETRY //
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef Kernel::Plane_3                                     Plane;
typedef Kernel::Line_3                                      Line;
// GEOMETRY //


// SURFACE_MESH //
typedef CGAL::Surface_mesh<Point>                           Mesh;
typedef Mesh::Vertex_index                                  Vertex;
typedef Mesh::Face_index                                    Face;
typedef Mesh::Edge_index                                    Edge;
typedef Mesh::Halfedge_index                                Halfedge;
typedef CGAL::Vertex_around_target_circulator<Mesh>         Vertex_around_target_circulator;
typedef CGAL::Face_around_target_circulator<Mesh>           Face_around_target_circulator;
typedef CGAL::Halfedge_around_face_circulator<Mesh>         Halfedge_around_face_circulator;
typedef Mesh::Property_map<Vertex, Point>                   VProp_geom;
typedef Mesh::Property_map<Vertex, int>                     VProp_int;
typedef Mesh::Property_map<Vertex, double>                  VProp_double;
typedef Mesh::Property_map<Face, int>                       FProp_int;
typedef Mesh::Property_map<Face, double>                    FProp_double;
typedef Mesh::Property_map<Face, Point>                     FProp_color;
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