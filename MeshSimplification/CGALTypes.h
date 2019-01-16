#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
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
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

// GEOMETRY //
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

// 2D // 
typedef Kernel::Point_2                                     Point_2;
typedef Kernel::Segment_2                                   Segment_2;
typedef CGAL::Alpha_shape_vertex_base_2<Kernel>             Vb;
typedef CGAL::Alpha_shape_face_base_2<Kernel>               Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>        Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>         Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                Alpha_shape_2;
typedef Alpha_shape_2::Alpha_iterator                       Alpha_iterator;
typedef Alpha_shape_2::Alpha_shape_edges_iterator           Alpha_shape_edges_iterator;
// 2D //

// 3D //
typedef Kernel::Point_3                                     Point_3;
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