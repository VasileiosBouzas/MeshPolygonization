#pragma once

#include "Utils.h"
#include "../solver/Mixed_integer_program_traits.h"
#include "../solver/GLPK_mixed_integer_program_traits.h"

typedef CGAL::GLPK_mixed_integer_program_traits<double>	 MIP_Solver;
typedef MIP_Solver::Variable			                 Variable;
typedef MIP_Solver::Linear_objective	                 Linear_objective;
typedef MIP_Solver::Linear_constraint	                 Linear_constraint;

inline std::vector<double> optimize(Mesh* mesh, std::vector<Plane_intersection>* edges) {
	// Face attributes //
	// Face index
	Mesh::Property_map<Face, std::size_t> face_indices = mesh->property_map<Face, std::size_t>("f:index").first;

	// Number of supporting faces
	Mesh::Property_map<Face, std::size_t> supporting_face_num = mesh->property_map<Face, std::size_t>("f:supporting_face_num").first;

	// Covered area
	Mesh::Property_map<Face, double> covered_area = mesh->property_map<Face, double>("f:covered_area").first;

	// Total area
	Mesh::Property_map<Face, double> area = mesh->property_map<Face, double>("f:area").first;
	// Face attributes //

	MIP_Solver solver;

	// Linear program coefficients
	double wt_fitting = 0.43;
	double wt_coverage = 0.27;
	double wt_complexity = 0.30;

	// Compute total number of supporting faces of the model
	double total_faces = 0.0;
	std::size_t idx = 0;
	for (auto f : mesh->faces()) {
		total_faces += supporting_face_num[f];
		face_indices[f] = idx;
		++idx;
	}

	// Binary variables:
	// x[0] ... x[num_faces - 1] : binary labels of all the input faces
	// x[num_faces] ... x[num_faces + num_edges - 1] : binary labels of all the intersecting edges (remain or not)

	// Determine variable number
	std::size_t num_faces = mesh->number_of_faces();
	std::size_t num_edges = edges->size();
	std::size_t total_variables = num_faces + num_edges;

	// Add variable
	const std::vector<Variable*>& variables = solver.create_n_variables(total_variables);
	for (std::size_t i = 0; i < total_variables; ++i) {
		Variable* v = variables[i];
		v->set_variable_type(Variable::BINARY);
	}

	// Add objective
	VProp_geom geom = mesh->points();
	std::vector<Point_3> points;
	idx = 0;
	for (auto vertex : mesh->vertices()) {
		points[idx] = geom[vertex];
		++idx;
	}

	typedef CGAL::Iso_cuboid_3<Kernel> Box;

	const Box& box = CGAL::bounding_box(points.begin(), points.end());
	double dx = box.xmax() - box.xmin();
	double dy = box.ymax() - box.ymin();
	double dz = box.zmax() - box.zmin();
	double box_area = double(2.0) * (dx * dy + dy * dz + dz * dx);

	// Chooses a better scale: all actual values multiplied by total number of points
	double coeff_data_fitting = wt_fitting;
	double coeff_coverage = total_faces * wt_coverage / box_area;

	Linear_objective * objective = solver.create_objective(Linear_objective::MINIMIZE);

	for (auto f : mesh->faces()) {
		std::size_t var_idx = face_indices[f];

		// Accumulates data fitting term
		double num = supporting_face_num[f];
		objective->add_coefficient(variables[var_idx], -coeff_data_fitting * num);

		// Accumulates model coverage term
		double uncovered_area = (area[f] - covered_area[f]);
		objective->add_coefficient(variables[var_idx], coeff_coverage * uncovered_area);
	}

	// Adds constraints: the number of faces associated with an edge must be either 2 or 0
	std::size_t var_edge_used_idx = 0;
	for (std::size_t i = 0; i < edges->size(); i++) {
		Linear_constraint* c = solver.create_constraint(0.0, 0.0);
		auto fan = (*edges)[i].fan;
		for (std::size_t j = 0; j < fan.size(); ++j) {
			auto f = fan[j];
			std::size_t var_idx = face_indices[f];
			c->add_coefficient(variables[var_idx], 1.0);
		}

		// If edge is adjacent to less than 2 faces, delete them
		// If edge is adjacent to more than 2 faces, choose two of them
		if (fan.size() >= 2) {
			std::size_t var_idx = num_faces + var_edge_used_idx;
			double num = double(fan.size() - 2);
			c->add_coefficient(variables[var_idx], -2.0 * num);
			++var_edge_used_idx;
		}
	}

	// Optimization
	std::vector<double> X;
	if (solver.solve()) {
		// Marks results
		X = solver.solution();
	}
	return X;
}