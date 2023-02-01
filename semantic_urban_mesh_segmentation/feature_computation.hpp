/*
*   Name        : feature_computation.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : handcrafted features extraction and texture processing
*   Availability:
*   Copyright   : Copyright (C) 2021 by Weixiao GAO (gaoweixiaocuhk@gmail.com)
*                 All rights reserved.
*
*				  This file is part of semantic_urban_mesh_segmentation: software
*				  for semantic segmentation of textured urban meshes.
*
*				  semantic_urban_mesh_segmentation is free software; you can
*				  redistribute it and/or modify it under the terms of the GNU
*				  General Public License Version 3 as published by the Free
*				  Software Foundation.
*
*				  semantic_urban_mesh_segmentation is distributed in the hope that
*				  it will be useful, but WITHOUT ANY WARRANTY; without even the
*				  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
*				  PURPOSE. See the GNU General Public License for more details.
*
*				  You should have received a copy of the GNU General Public License
*				  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#ifndef semantic_mesh_segmentation__FEATURE_COMPUTATION_HPP
#define semantic_mesh_segmentation__FEATURE_COMPUTATION_HPP


#include <opencv2/opencv.hpp>
#include <easy3d/kdtree.h>
#include <easy3d/point_cloud.h>

#include <omp.h>
#include <easy3d/kdtree.h>
#include <easy3d/point_cloud.h>
#include <easy3d/point_cloud_io.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "super_segment.hpp"
#include "PTCloud.hpp"
#include "sampling_function.hpp"
#include "parameters.hpp"
#include "property_parsing.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	//--- bool comparision ---
	//x coordinates compare
	inline bool smaller_coord
	(
		float x1,
		float x2
	) restrict(cpu)
	{
		if (x1 >= x2)
			return false;
		else
			return true;
	}

	//superfcaet area compare
	inline bool larger_segments_area
	(
		superfacets spf_1,
		superfacets spf_2
	)
	{
		if (spf_1.sum_area > spf_2.sum_area)
			return true;
		else
			return false;
	}

	//superfacet size compare
	inline bool larger_segment_size
	(
		superfacets spf_1,
		superfacets spf_2
	)
	{
		if (spf_1.face_vec.size() > spf_2.face_vec.size())
			return true;
		else
			return false;
	}

	inline bool lower_local_elevation
	(
		std::pair<int, float> pid_ele_1,
		std::pair<int, float> pid_ele_2
	)
	{
		if (pid_ele_1.second < pid_ele_2.second)
			return true;
		else
			return false;
	}
	//-------------------------------------------------------------------------------------------------------------------------------
	//--- value validation and normalization ---
	inline void value_validation_check(float &value)
	{
		if (!std::isnormal(value))
			value = default_feature_value_minmax.first;
	}

	inline void vec3_validation_check(vec3 &vec_in)
	{
		if (!std::isnormal(vec_in.x))
			vec_in.x = 0.0f;
		if (!std::isnormal(vec_in.y))
			vec_in.y = 0.0f;
		if (!std::isnormal(vec_in.z))
			vec_in.z = 1.0f;
	}

	//-------------------------------------------------------------------------------------------------------------------------------
	//--- geometric computation ---
	inline float FaceArea
	(
		SFMesh* smesh_out,
		SFMesh::Face& f
	) restrict(cpu)
	{
		vec3 v0, v1, v2;
		int ind = 0;
		for (auto v : smesh_out->vertices(f))
		{
			if (ind == 0)
				v0 = smesh_out->get_points_coord[v];
			else if (ind == 1)
				v1 = smesh_out->get_points_coord[v];
			else if (ind == 2)
				v2 = smesh_out->get_points_coord[v];
			++ind;
		}

		float a = (v1 - v0).norm();
		float b = (v2 - v1).norm();
		float c = (v2 - v0).norm();
		float s = (a + b + c)*0.5;

		float tmp_area = sqrt(s*(s - a)*(s - b)*(s - c));
		value_validation_check(tmp_area);

		return tmp_area;
	}

	inline float compute_radius(const vec3 &p, const vec3 &n, const vec3 &q)
	{
		// Compute radius of the ball that touches points p and q and whose center falls on the normal n from p
		float d = (p - q).norm();
		float cos_theta = dot(n, p - q) / d;
		return float(d / (2 * cos_theta));
	}

	inline float cos_angle(const vec3 p, const vec3 q)
	{
		// Calculate the cosine of angle between vector p and q, see http://en.wikipedia.org/wiki/Law_of_cosines#Vector_formulation
		float result = dot(p, q) / (p.norm() * q.norm());
		if (result > 1) return 1;
		else if (result < -1) return -1;
		return result;
	}

	//calcualte distance in 3D space
	inline double Distance_3D
	(
		easy3d::vec3 &p1,
		easy3d::vec3 &p2
	)
	{
		return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
	}

	// Returns the angle between vector a and vector b in degree.
	inline float Angle2DDegree(const vec3 &a, const vec3 &b)
	{
		vec2 a2d(a.x, a.y), b2d(b.x, b.y);
		float angle_product = dot(a2d, b2d) / (a2d.length() * b2d.length());
		if (angle_product < -1.0)
			angle_product = -1.0;
		else if (angle_product > 1.0)
			angle_product = 1.0;

		return 180.0 / M_PI * acos(angle_product);
	}

	//angle between normal vector
	inline double vector3D_angle
	(
		const easy3d::vec3 v1,
		const easy3d::vec3 v2
	)
	{
		double angle; //in degree
		double a = sqrt(pow(v1.x, 2) + pow(v1.y, 2) + pow(v1.z, 2));
		double b = sqrt(pow(v2.x, 2) + pow(v2.y, 2) + pow(v2.z, 2));
		double numer = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
		double final_v = numer / (a*b);
		//avoid rounding error
		if (final_v > 1)
			final_v = 1.0;
		else if (final_v < -1)
			final_v = -1.0;
		angle = acos(final_v) * 180.0f / M_PI;
		return angle;
	}

	//compute veriticaltiy
	inline float compute_verticality_from_normal
	(
		const vec3 &normal_temp
	)
	{
		vec3 n_z(0, 0, 1);
		float a_v3 = sqrt(pow(n_z.x, 2) + pow(n_z.y, 2) + pow(n_z.z, 2));
		float b_v3 = sqrt(pow(normal_temp.x, 2) + pow(normal_temp.y, 2) + pow(normal_temp.z, 2));
		float numer = n_z.x*normal_temp.x + n_z.y*normal_temp.y + n_z.z*normal_temp.z;
		float final_v = numer / (a_v3*b_v3);
		float a_v = std::abs(final_v);// std::acos(final_v) * 180 / M_PI;
		return a_v;
	}

	//calculate distance from the point to the least square plane
	inline double dist2plane
	(
		easy3d::vec4 &plane_param,
		easy3d::vec3 &v_test
	)
	{
		double dis = FLT_MAX;
		if (sqrt(pow(plane_param.x, 2) + pow(plane_param.y, 2) + pow(plane_param.z, 2) != 0))
			dis = abs(plane_param.x * v_test.x + plane_param.y * v_test.y + plane_param.z * v_test.z - plane_param.w) / sqrt(pow(plane_param.x, 2) + pow(plane_param.y, 2) + pow(plane_param.z, 2));
		return dis;
	}

	inline double dist2plane
	(
		easy3d::vec4 &plane_param,
		Point_3 &v_test
	)
	{
		double dis = FLT_MAX;
		if (sqrt(pow(plane_param.x, 2) + pow(plane_param.y, 2) + pow(plane_param.z, 2) != 0))
			dis = abs(plane_param.x * v_test.x() + plane_param.y * v_test.y() + plane_param.z * v_test.z() - plane_param.w) / sqrt(pow(plane_param.x, 2) + pow(plane_param.y, 2) + pow(plane_param.z, 2));
		return dis;
	}
	//-------------------------------------------------------------------------------------------------------------------------------
	//--- searching and recursive algorithm ---
	inline void Build_kdtree
	(
		easy3d::PointCloud* vertex_cloud,
		easy3d::KdTree *tree
	)
	{
		// create an instance of KdTree
		tree->begin();
		// build kd-tree using the point cloud data
		tree->add_point_cloud(vertex_cloud);
		tree->end();
	}

	//-------------------------------------------------------------------------------------------------------------------------------
	//--- texture coordinates related ---
	//check point in triangle
	inline bool PointinTriangle
	(
		std::vector<float>& U_vec,
		std::vector<float>& V_vec,
		std::vector<float>& P
	) //U_vec={X1,X2,X3}, V_vec={Y1,Y2,Y3}, P={X,Y}
	{
		float x0 = U_vec[0]; float y0 = V_vec[0];//A
		float x1 = U_vec[1]; float y1 = V_vec[1];//B
		float x2 = U_vec[2]; float y2 = V_vec[2];//C
		float x = P[0]; float y = P[1];

		float v0_x = x2 - x0; float v0_y = y2 - y0;
		float v1_x = x1 - x0; float v1_y = y1 - y0;
		float v2_x = x - x0; float v2_y = y - y0;

		float dot00 = v0_x * v0_x + v0_y * v0_y;
		float dot01 = v0_x * v1_x + v0_y * v1_y;
		float dot02 = v0_x * v2_x + v0_y * v2_y;
		float dot11 = v1_x * v1_x + v1_y * v1_y;
		float dot12 = v1_x * v2_x + v1_y * v2_y;

		float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

		float u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
		if (u < 0 || u > 1) // if u out of range, return directly
		{
			return false;
		}

		float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
		if (v < 0 || v > 1) // if v out of range, return directly
		{
			return false;
		}
		return u + v <= 1;
	}

	//From UV to 3D coordinates
	inline void uv_to_3D_coordinates
	(
		std::vector<vec2> &uv_triangle,
		std::vector<vec3> &coord3d_triangle,
		vec2 &current_uv,
		vec3 &current_3d
	)
	{
		float T00 = uv_triangle[0].x - uv_triangle[2].x;
		float T01 = uv_triangle[1].x - uv_triangle[2].x;
		float T10 = uv_triangle[0].y - uv_triangle[2].y;
		float T11 = uv_triangle[1].y - uv_triangle[2].y;

		float d = T00 * T11 - T01 * T10;

		float iT00 = T11 / d;
		float iT01 = -T01 / d;
		float iT10 = -T10 / d;
		float iT11 = T00 / d;

		float lambda0 = iT00 * (current_uv.x - uv_triangle[2].x) + iT01 * (current_uv.y - uv_triangle[2].y);
		float lambda1 = iT10 * (current_uv.x - uv_triangle[2].x) + iT11 * (current_uv.y - uv_triangle[2].y);
		float lambda2 = 1.0 - lambda0 - lambda1;

		current_3d.x = coord3d_triangle[0].x * lambda0 + coord3d_triangle[1].x * lambda1 + coord3d_triangle[2].x * lambda2;
		current_3d.y = coord3d_triangle[0].y * lambda0 + coord3d_triangle[1].y * lambda1 + coord3d_triangle[2].y * lambda2;
		current_3d.z = coord3d_triangle[0].z * lambda0 + coord3d_triangle[1].z * lambda1 + coord3d_triangle[2].z * lambda2;
	}

	//-------------------------------------------------------------------------------------------------------------------------------
	//--- mathematic computation ---
	inline void PCA_EigenSolver
	(
		std::vector<int> &current_point_range,
		PTCloud* point_cloud,
		DiagonalizeTraits::Vector &eigen_values,
		DiagonalizeTraits::Matrix &eigen_vectors,
		Plane &plane_c
	) restrict(cpu)
	{
		auto points_coord = point_cloud->get_vertex_property<easy3d::vec3>("v:point");
		std::vector<Point_3> sampling_point;
		for (auto p : current_point_range)
		{
			easy3d::PointCloud::Vertex vtx(p);
			Point_3 temp_point(points_coord[vtx].x, points_coord[vtx].y, points_coord[vtx].z);
			sampling_point.push_back(temp_point);
		}

		Point_3 c = centroid(sampling_point.begin(), sampling_point.end(), Kernel(), CGAL::Dimension_tag<0>());

		DiagonalizeTraits::Covariance_matrix covariance = { { 0., 0., 0., 0., 0., 0. } };
		CGAL::internal::assemble_covariance_matrix_3(sampling_point.begin(), sampling_point.end(), covariance, c, Kernel(), (Point_3*)NULL, CGAL::Dimension_tag<0>(), DiagonalizeTraits());
		eigen_values = { { 0. , 0., 0. } };
		eigen_vectors = { { 0., 0., 0.,
							0., 0., 0.,
							0., 0., 0. } };
		//eig1 < eig2 < eig3
		DiagonalizeTraits::diagonalize_selfadjoint_covariance_matrix(covariance, eigen_values, eigen_vectors);

		// degenerate case 
		if (eigen_values[0] == eigen_values[1] &&
			eigen_values[1] == eigen_values[2])
		{
			// assemble a default horizontal plane that goes
			// through the centroid.
			plane_c = Plane(c, Vector_3(FT(0), FT(0), FT(1)));
		}
		else // regular and line case
		{
			Vector_3 normal(eigen_vectors[0],
				eigen_vectors[1],
				eigen_vectors[2]);
			plane_c = Plane(c, normal);
			//return FT(1) - eigen_values[0] / eigen_values[1];
		} // end regular case
	}

	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	inline void multi_scales_elevations
	(
		std::vector<std::pair<float, float>>  &multi_scales_seg_minmax_ele,
		std::vector<float> &seg_mulsc_ele_fea,
		float &seg_center_z
	)
	{
		for (int i = 0; i < multi_scale_ele_radius.size(); ++i)
		{
			float z_diff = 1.0f;
			if (multi_scales_seg_minmax_ele[i].second - multi_scales_seg_minmax_ele[i].first > 0)
				z_diff = multi_scales_seg_minmax_ele[i].second - multi_scales_seg_minmax_ele[i].first;
			seg_mulsc_ele_fea[i] = std::sqrtf((seg_center_z - multi_scales_seg_minmax_ele[i].first) / z_diff);
			value_validation_check(seg_mulsc_ele_fea[i]);
		}
	}


	//------------------------------------------------ ------------------------------- ----------------------------------------------//
		//--- configuration merge mesh and point cloud ---
	inline void mesh_configuration
	(
		SFMesh *smesh_all
	)
	{
		smesh_all->get_points_coord = smesh_all->get_vertex_property<vec3>("v:point");
		smesh_all->add_face_property<int>("f:" + label_definition, -1);
		smesh_all->get_face_truth_label = smesh_all->get_face_property<int>("f:" + label_definition);
		smesh_all->add_face_property<int>("f:face_predict", -1);
		smesh_all->get_face_predict_label = smesh_all->get_face_property<int>("f:face_predict");
		smesh_all->add_face_property<vec3>("f:color");
		smesh_all->get_face_color = smesh_all->get_face_property<vec3>("f:color");
		smesh_all->add_face_property<std::vector<float>>("f:texcoord");
		smesh_all->get_face_texcoord = smesh_all->get_face_property<std::vector<float>>("f:texcoord");
		smesh_all->add_face_property<int>("f:texnumber");
		smesh_all->get_face_texnumber = smesh_all->get_face_property<int>("f:texnumber");
		smesh_all->add_face_property<vec3>("f:normal");
		smesh_all->get_face_normals = smesh_all->get_face_property<vec3>("f:normal");
		smesh_all->add_vertex_property<vec3>("v:normal");
		smesh_all->get_points_normals = smesh_all->get_vertex_property<vec3>("v:normal");

		if (processing_mode == 2)
		{
			smesh_all->add_face_property<float>("f:label_probabilities");
			smesh_all->get_face_predict_prob = smesh_all->get_face_property<float>("f:label_probabilities");
		}
	}

	inline void pointcloud_configuration
	(
		PTCloud *cloud_all
	)
	{
		cloud_all->get_points_coord = cloud_all->get_vertex_property<vec3>("v:point");
		cloud_all->add_vertex_property<vec3>("v:normal");
		cloud_all->get_points_normals = cloud_all->get_vertex_property<vec3>("v:normal");
		cloud_all->add_vertex_property<vec3>("v:color");
		cloud_all->get_points_color = cloud_all->get_vertex_property<vec3>("v:color");
	}


	//find n rings point on line
	inline void find_n_rings_point_on_line
	(
		SFMesh* smesh_out,
		std::vector<easy3d::vec3> &alpha_border_points,
		std::vector<Point_3> &vertex_on_line,
		const int n,
		const int current_id
	)
	{
		int num_points = alpha_border_points.size();
		Point_3 p_tmp(alpha_border_points[current_id].x, alpha_border_points[current_id].y, alpha_border_points[current_id].z);
		vertex_on_line.push_back(p_tmp);
		for (int i = 1; i < n + 1; ++i) if (vertex_on_line.size() <= alpha_border_points.size())
		{
			if (current_id + i < num_points)
			{
				Point_3 p_forward(alpha_border_points[current_id + i].x, alpha_border_points[current_id + i].y, alpha_border_points[current_id + i].z);
				vertex_on_line.push_back(p_forward);
			}
			else
			{
				Point_3 p_forward(alpha_border_points[current_id + i - num_points].x, alpha_border_points[current_id + i - num_points].y, alpha_border_points[current_id + i - num_points].z);
				vertex_on_line.push_back(p_forward);
			}

			if (vertex_on_line.size() >= alpha_border_points.size())
				break;

			if (current_id - i >= 0)
			{
				Point_3 p_backward(alpha_border_points[current_id - i].x, alpha_border_points[current_id - i].y, alpha_border_points[current_id - i].z);
				vertex_on_line.push_back(p_backward);
			}
			else
			{
				Point_3 p_backward(alpha_border_points[num_points + current_id - i].x, alpha_border_points[num_points + current_id - i].y, alpha_border_points[num_points + current_id - i].z);
				vertex_on_line.push_back(p_backward);
			}
		}
	}

	//find n ring edges
	inline void find_n_rings_neighbor_of_vertex
	(
		SFMesh* smesh_out,
		SFMesh::Edge &edx,
		const int n,
		std::vector<int> &edge_n_rings_neg,
		std::map<int, bool> &checked_neighbor
	)
	{
		SFMesh::Vertex vs = smesh_out->vertex(edx, 0);
		SFMesh::Vertex vt = smesh_out->vertex(edx, 1);
		for (auto hc : smesh_out->halfedges(vs))
		{
			SFMesh::Edge eo = smesh_out->edge(hc);
			if (n > 1)
			{
				find_n_rings_neighbor_of_vertex(smesh_out, eo, n - 1, edge_n_rings_neg, checked_neighbor);
			}

			auto it_e = checked_neighbor.find(eo.idx());
			if (it_e == checked_neighbor.end())
			{
				checked_neighbor[eo.idx()] = true;
				edge_n_rings_neg.push_back(eo.idx());
			}
		}

		for (auto hc : smesh_out->halfedges(vt))
		{
			SFMesh::Edge eo = smesh_out->edge(hc);
			if (n > 1)
			{
				find_n_rings_neighbor_of_vertex(smesh_out, eo, n - 1, edge_n_rings_neg, checked_neighbor);
			}

			auto it_e = checked_neighbor.find(eo.idx());
			if (it_e == checked_neighbor.end())
			{
				checked_neighbor[eo.idx()] = true;
				edge_n_rings_neg.push_back(eo.idx());
			}
		}
	}

	inline void get_segment_border_points
	(
		PTCloud *cloud_out,
		std::vector<int> &seg_truth,
		std::vector<std::vector<int>> &seg_pcl_vec
	)
	{
		cloud_out->add_vertex_property<std::vector<int>>("v:points_points_segment_ids", std::vector<int>());
		auto get_points_segment_ids = cloud_out->get_vertex_property<std::vector<int>>("v:points_points_segment_ids");

		for (int seg_i = 0; seg_i < seg_pcl_vec.size(); ++seg_i)
		{
			for (int vi = 0; vi < seg_pcl_vec[seg_i].size(); ++vi)
			{
				PTCloud::Vertex vtx(seg_pcl_vec[seg_i][vi]);
				get_points_segment_ids[vtx].push_back(seg_i);
			}
		}

		for (int pi = 0; pi < cloud_out->vertices_size(); ++pi)
		{
			std::set<int> ptx_seg_la;
			PTCloud::Vertex ptx(pi);
			for (int seg_i = 0; seg_i < get_points_segment_ids[ptx].size(); ++seg_i)
			{
				ptx_seg_la.insert(seg_truth[get_points_segment_ids[ptx][seg_i]]);
				if (ptx_seg_la.size() > 1)
					break;
			}
			if (ptx_seg_la.size() == 1)
				cloud_out->get_points_use_seg_truth[ptx] = true;
		}

		cloud_out->remove_vertex_property(get_points_segment_ids);
	}


	//--- contour related computation ---
	inline void compute_shape_features
	(
		superfacets &segment,
		SFMesh *smesh_out,
		float &shape_descriptor,
		float &compactness,
		float &shape_index
	)
	{
		int ring_point_size = segment.alpha_border_points.size();
		float total_length = segment.border_length;

		for (int i = 0; i < ring_point_size; ++i)
		{
			std::vector<Point_3> vertex_on_line;
			find_n_rings_point_on_line(smesh_out, segment.alpha_border_points, vertex_on_line, border_growing_neighbor, i);
			Line_3 licgal;
			shape_descriptor += CGAL::linear_least_squares_fitting_3(vertex_on_line.begin(), vertex_on_line.end(), licgal, CGAL::Dimension_tag<0>());
		}

		if (ring_point_size > 0)
		{
			shape_descriptor = shape_descriptor / float(ring_point_size);
		}
		else
		{
			shape_descriptor = default_feature_value_minmax.first;
		}

		if (total_length > default_feature_value_minmax.first)
		{
			compactness = 4.0f * M_PI * segment.sum_area / float(pow(total_length, 2.0f));
			if (compactness > cutoff_spfcompact_max)
				compactness = cutoff_spfcompact_max;
			shape_index = total_length / float(pow(segment.sum_area, 1.0f / 4.0f));
		}
		else
		{
			compactness = default_feature_value_minmax.first;
			shape_index = default_feature_value_minmax.first;
		}

		value_validation_check(shape_descriptor);
		value_validation_check(compactness);
		value_validation_check(shape_index);
	}

	template<class Triangulation_2_Exact, class Alpha_shape_2_Exact, class Edge_circulator_Exact, class Vertex_handle_Exact, class Face_handle_Exact>
	inline void extract_alpha_shape_boundary
	(
		superfacets &segment_out,
		std::vector<Point_3_Exact> &alpha_candidate_points_cgal
	)
	{
		Triangulation_2_Exact tr_tmp;
		tr_tmp.insert(alpha_candidate_points_cgal.begin(), alpha_candidate_points_cgal.end());

		//use alpha to extract the 2D boundary
		Alpha_shape_2_Exact border_shape(tr_tmp,
			FT_Exact(alpha_shape_val),
			Alpha_shape_2_Exact::GENERAL);

		// flood filling 
		auto grower = AlphaShapeRegionGrower
			<Alpha_shape_2_Exact, Face_handle_Exact, Vertex_handle_Exact>(border_shape);
		grower.grow();

		std::vector<std::vector<vec3>> alpha_rings;
		for (auto& kv : grower.region_map)
		{
			auto region_label = kv.first;
			auto v_start = kv.second;

			// find edges of outer boundary in order
			std::vector<vec3> current_ring;
			current_ring.push_back(vec3(v_start->point().x(), v_start->point().y(), v_start->point().z()));

			// secondly, walk along the entire boundary starting from v_start
			Vertex_handle_Exact v_next, v_prev = v_start, v_cur = v_start;
			size_t v_cntr = 0;
			do
			{
				Edge_circulator_Exact ec(border_shape.incident_edges(v_cur)), done(ec);
				do {
					// find the vertex on the other side of the incident edge ec
					auto v = ec->first->vertex(border_shape.cw(ec->second));
					if (v_cur == v)
						v = ec->first->vertex(border_shape.ccw(ec->second));
					// find labels of two adjacent faces
					auto label1 = grower.face_map[ec->first];
					auto label2 = grower.face_map[ec->first->neighbor(ec->second)];
					// check if the edge is on the boundary of the region and if we are not going backwards
					bool exterior = label1 == -1 || label2 == -1;
					bool region = label1 == region_label || label2 == region_label;
					if ((exterior && region) && (v != v_prev))
					{
						v_next = v;
						current_ring.push_back(vec3(v_next->point().x(), v_next->point().y(), v_next->point().z()));
						break;
					}
				} while (++ec != done);
				v_prev = v_cur;
				v_cur = v_next;

			} while (v_next != v_start);
			// finally, store the ring 
			alpha_rings.push_back(current_ring);
		}

		//extract alppha ring with longest total length
		std::tuple<int, float, std::vector<vec3>> ringind_maxlength(-1, -FLT_MAX, std::vector<vec3>());
		int ring_i = 0;
		for (auto ring : alpha_rings)
		{
			float length = 0.0f;
			for (int vi = 1; vi < ring.size(); ++vi)
			{
				length += Distance_3D(ring[vi - 1], ring[vi]);
			}

			if (length > get<1>(ringind_maxlength))
			{
				get<0>(ringind_maxlength) = ring_i;
				get<1>(ringind_maxlength) = length;
				get<2>(ringind_maxlength) = ring;
			}
			++ring_i;
		}

		for (int vi = 1; vi < get<2>(ringind_maxlength).size(); ++vi)
		{
			//int vs = alpha_vertices.size();
			//alpha_vertices.push_back(get<2>(ringind_maxlength)[vi - 1]);
			segment_out.segment_alpha_vertices.push_back(std::make_tuple(vi - 1, get<2>(ringind_maxlength)[vi - 1], false));
			segment_out.alpha_border_points.push_back(get<2>(ringind_maxlength)[vi - 1]);

			//int vt = alpha_vertices.size();
			//alpha_vertices.push_back(get<2>(ringind_maxlength)[vi]);
			segment_out.segment_alpha_vertices.push_back(std::make_tuple(vi, get<2>(ringind_maxlength)[vi], false));
			segment_out.alpha_border_points.push_back(get<2>(ringind_maxlength)[vi]);

			//alpha_edges.push_back(std::make_pair(vs, vt));
			segment_out.segment_alpha_edges.push_back(std::make_tuple(vi - 1, std::make_pair(vi - 1, vi), false));
		}

		if (get<2>(ringind_maxlength).size() > 1)
			segment_out.border_length = get<1>(ringind_maxlength);
		else
		{
			segment_out.border_length = default_feature_value_minmax.first;
			for (auto pts : segment_out.ExactVec_CGAL)
			{
				segment_out.alpha_border_points.push_back(vec3(pts.x(), pts.y(), pts.z()));
			}
		}
	}


	inline void MarkMeshBoundaryAlphaVertices
	(
		SFMesh* smesh_out,
		PTCloud *cloud_pt_3d,
		superfacets &segment,
		std::vector<superfacets> &segment_all,
		std::map<vec3, PTCloud::Vertex> &vec_pcl_map
	)
	{
#pragma omp parallel for schedule(dynamic)
		for (int alpha_pi = 0; alpha_pi < segment.segment_alpha_vertices.size(); ++alpha_pi)
		{
			vec3 p_tmp = get<1>(segment.segment_alpha_vertices[alpha_pi]);
			if (cloud_pt_3d->get_points_on_mesh_border[vec_pcl_map[p_tmp]])
			{
				get<2>(segment.segment_alpha_vertices[alpha_pi]) = true;
			}
		}
	}

	//For mesh contain duplicate vertices and interior mesh boundary
	inline void extract_segment_longest_border
	(
		std::vector<superfacets> &segment_all,
		SFMesh *smesh_out,
		PTCloud *cloud_pt_3d,
		superfacets &segment
	)
	{
		//Extract segment points and convert to CGAL point type
		std::vector<Point_3_Exact> alpha_candidate_points_cgal;
		std::vector<Point_3> fitting_points;
		std::map<vec3, PTCloud::Vertex> vec_pcl_map;
		alpha_candidate_points_cgal.resize(segment.sampled_points.size());
		fitting_points.resize(segment.sampled_points.size());
		//#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < segment.sampled_points.size(); ++i)
		{
			PTCloud::Vertex ptx(segment.sampled_points[i]);
			vec3 p_tmp = cloud_pt_3d->get_points_coord[ptx];
			alpha_candidate_points_cgal[i] = Point_3_Exact(p_tmp.x, p_tmp.y, p_tmp.z);
			fitting_points[i] = Point_3(p_tmp.x, p_tmp.y, p_tmp.z);
			vec_pcl_map[p_tmp] = ptx;
		}

		//check the which plane to project and compute alpha shape
		vec3 xy_normal(0.0f, 0.0f, 1.0f), xz_normal(0.0f, 1.0f, 0.0f), yz_normal(1.0f, 0.0f, 0.0f);
		Plane plcgal;
		CGAL::linear_least_squares_fitting_3(fitting_points.begin(), fitting_points.end(), plcgal, CGAL::Dimension_tag<0>());
		vec3 plane_normal(plcgal.a(), plcgal.b(), plcgal.c());
		float xy_angle = vector3D_angle(plane_normal, xy_normal);
		float xz_angle = vector3D_angle(plane_normal, xz_normal);
		float yz_angle = vector3D_angle(plane_normal, yz_normal);

		xy_angle = xy_angle > 90.0f ? 180.0f - xy_angle : xy_angle;
		xz_angle = xz_angle > 90.0f ? 180.0f - xz_angle : xz_angle;
		yz_angle = yz_angle > 90.0f ? 180.0f - yz_angle : yz_angle;

		if (xz_angle < xy_angle && xz_angle < yz_angle)
		{
			extract_alpha_shape_boundary
				<Triangulation_2_Exact_xz,
				Alpha_shape_2_Exact_xz,
				Edge_circulator_Exact_xz,
				Vertex_handle_Exact_xz,
				Face_handle_Exact_xz>(segment, alpha_candidate_points_cgal);
		}
		else if (yz_angle < xy_angle && yz_angle < xz_angle)
		{
			extract_alpha_shape_boundary
				<Triangulation_2_Exact_yz,
				Alpha_shape_2_Exact_yz,
				Edge_circulator_Exact_yz,
				Vertex_handle_Exact_yz,
				Face_handle_Exact_yz>(segment, alpha_candidate_points_cgal);
		}
		else
		{
			extract_alpha_shape_boundary
				<Triangulation_2_Exact_xy,
				Alpha_shape_2_Exact_xy,
				Edge_circulator_Exact_xy,
				Vertex_handle_Exact_xy,
				Face_handle_Exact_xy>(segment, alpha_candidate_points_cgal);
		}

		//process mesh border points, ignore them in the later process, in case of processing large segment cost too much time 
		if (ignore_mesh_boundary.first && segment.contain_mesh_border_facets)
			MarkMeshBoundaryAlphaVertices(smesh_out, cloud_pt_3d, segment, segment_all, vec_pcl_map);

		//use alpha vertices to compute shape features
		for (int alpha_pi = 0; alpha_pi < segment.segment_alpha_vertices.size(); ++alpha_pi)
		{
			if (ignore_mesh_boundary.first)
			{
				if (!get<2>(segment.segment_alpha_vertices[alpha_pi]))
					segment.border_points.push_back(get<1>(segment.segment_alpha_vertices[alpha_pi]));
			}
			else
			{
				segment.border_points.push_back(get<1>(segment.segment_alpha_vertices[alpha_pi]));
			}
		}
	}

	//extract segment border based features
	inline void segment_shape_based_features
	(
		SFMesh *smesh_out,
		PTCloud *cloud_pt_3d, //use dense
		std::vector<superfacets> &segments,
		const int seg_i,
		float &shape_descriptor,
		float &compactness,
		float &shape_index
	) restrict(cpu)
	{
		//Extract contour from segment sampled point cloud based on alpha-shape
		extract_segment_longest_border(segments, smesh_out, cloud_pt_3d, segments[seg_i]);

		//compute shape based features
		compute_shape_features(segments[seg_i], smesh_out, shape_descriptor, compactness, shape_index);
	}
	//functions declare
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	//------------------------------------------------ Surface mesh sampling functions ----------------------------------------------//
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	void texture_pointcloud_generation(SFMesh*, SFMesh::Face &, std::vector<cv::Mat> &, std::vector<cv::Mat> &, PTCloud* tex_cloud = nullptr);

	void face_texture_processor(SFMesh *, std::vector<cv::Mat> &, const int, PTCloud* tex_cloud = nullptr);

	void get_sampling_cloud_normals_from_mesh_faces(SFMesh *, PTCloud *, PTCloud *, PTCloud *, easy3d::KdTree *, easy3d::KdTree *);

	void finalization_sampling_point_cloud(PTCloud*, PTCloud *, easy3d::PointCloud*, easy3d::PointCloud*);

	void finalization_sampling_point_cloud(PTCloud *, easy3d::PointCloud*);

	void parsing_texture_color_to_sampled_pointcloud(PTCloud *, PTCloud *, const int, PTCloud::Vertex &);

	void match_ele_sampling_pointcloud_with_mesh_faces(SFMesh *, PTCloud *, PTCloud *, easy3d::KdTree *);
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	//------------------------------------------------  Features computation functions ----------------------------------------------//
	//------------------------------------------------ ------------------------------- ----------------------------------------------//	
	void merge_pointcloud(PTCloud *, PTCloud *, SFMesh *, int &, const int, std::map<int, int> &ptidx_faceid_map_all = std::map<int, int>());

	void merge_mesh(SFMesh *, SFMesh *, int &, int &, int &, SFMesh *smesh_overseg = nullptr);

	void compute_geometric_features_only_per_scale(SFMesh*, PTCloud*, superfacets &, const int, std::vector<float> &, std::vector< std::vector<float>> &);

	void compute_features_on_vertices_face_centers_pcl(SFMesh*, PTCloud*, superfacets &, const int, std::vector<float> &, std::vector<float> &, std::vector< std::vector<float>> &);

	void compute_radiometric_features_on_face_textures(SFMesh*, superfacets &, const int, std::vector<float> &);

	void medial_ball_features(SFMesh *, PTCloud *);

	void local_elevation_for_pointcloud(SFMesh *, PTCloud*, std::vector<superfacets>&);

	void feature_batch_normalization(std::vector< std::vector< float > > &);

	void compute_segment_basic_features(SFMesh *, std::vector<superfacets> &, std::vector<int> &, std::vector<std::pair<int, float>> &, std::vector< std::vector< float > > &, std::vector< std::vector<float> >&, std::vector< std::vector<float> >&, PTCloud*, PTCloud*);

	void compute_segment_features
	(
		SFMesh *,
		std::vector<superfacets> &,
		PTCloud *,
		std::vector<std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &
	);

	void tiles_merge_to_batch
	(
		SFMesh *,
		PTCloud *,
		PTCloud *,
		PTCloud *,
		SFMesh *,
		PTCloud *,
		PTCloud *,
		PTCloud *,
		std::map<int, int> &,
		int &,
		int &,
		int &,
		int &,
		SFMesh *smesh_overseg = nullptr
	);

	void process_batch_tiles(std::vector<std::pair<int, std::string>> &, const int);

	void process_single_tile(const int);

	void get_segment_features(SFMesh *, PTCloud *, PTCloud*, PTCloud *, PTCloud *, std::map<int, int> &, const int);

	void visualization_process_batch_tiles(std::vector<std::pair<int, std::string>> &, const int);

	void visualization_process_single_tiles(const int);

	void normalization_all
	(
		std::vector< std::vector<float>> &,
		std::vector< std::vector< std::vector<float> > > &,
		std::vector< std::vector< std::vector<float> > > &,
		std::vector< std::vector<float> > &
	);

	void PNP_MRF_single_tiles(const int);

	void feature_selection_for_GCN
	(
		SFMesh *,
		PTCloud *,
		std::vector<std::vector<int>> &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &
	);

	void feature_selection_for_GCN_single_tiles(const int);
}

#endif//MESH_SEGMENTATION__SEGMENT_FILE_HPP