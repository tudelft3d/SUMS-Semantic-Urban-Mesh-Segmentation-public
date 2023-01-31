/*
*   Name        : SFMesh.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh definition
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
#ifndef semantic_mesh_segmentation__SFMesh_HPP
#define semantic_mesh_segmentation__SFMesh_HPP


#include <omp.h>
#include <easy3d/surface_mesh.h>
#include "parameters.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	class SFMesh :public easy3d::SurfaceMesh
	{
	public:

		SFMesh()
		{
			//-------------------Add, Get vert properties--------------------
			add_vert_planarity = add_vertex_property<float>("v:vert_planarity", 0.0f);
			get_vert_planarity = get_vertex_property<float>("v:vert_planarity");

			add_vert_component_id = add_vertex_property<int>("v:vert_component_id", -1);
			get_vert_component_id = get_vertex_property<int>("v:vert_component_id");

			add_vert_check = add_vertex_property<bool>("v:vert_check", false);
			get_vert_check = get_vertex_property<bool>("v:vert_check");

			//-------------------Add, Get edge properties--------------------

			add_edge_boundary_truth = add_edge_property<int>("e:edge_boundary_truth", 0);
			get_edge_boundary_truth = get_edge_property<int>("e:edge_boundary_truth");

			add_edge_boundary_predict = add_edge_property<int>("e:edge_boundary_predict", 0);
			get_edge_boundary_predict = get_edge_property<int>("e:edge_boundary_predict");

			//-------------------Add, Get face properties--------------------
			add_face_segment_id = add_face_property<int>("f:face_segment_id", -1);
			get_face_segment_id = get_face_property<int>("f:face_segment_id");

			add_face_component_id = add_face_property<int>("f:face_component_id", -1);
			get_face_component_id = get_face_property<int>("f:face_component_id");

			add_face_smoothed_seg_id = add_face_property<int>("f:face_smoothed_seg_id", -1);
			get_face_smoothed_seg_id = get_face_property<int>("f:face_smoothed_seg_id");

			add_face_spid_update = add_face_property<int>("f:face_spid_update", -1);
			get_face_spid_update = get_face_property<int>("f:face_spid_update");

			add_face_area = add_face_property<float>("f:face_area", 0.0f);
			get_face_area = get_face_property<float>("f:face_area");

			add_face_predict_prob = add_face_property<float>("f:label_probabilities", 0.0f);
			get_face_predict_prob = get_face_property<float>("f:label_probabilities");

			add_face_1ring_avg_predict_prob = add_face_property<float>("f:one_ring_avg_label_probabilities", 0.0f);
			get_face_1ring_avg_predict_prob = get_face_property<float>("f:one_ring_avg_label_probabilities");

			add_face_center = add_face_property<vec3>("f:face_center", vec3(0, 0, 0));
			get_face_center = get_face_property<vec3>("f:face_center");

			add_face_planar_segment_plane_normal = add_face_property<vec3>("f:face_planar_segment_plane_normal", vec3(0, 0, 0));
			get_face_planar_segment_plane_normal = get_face_property<vec3>("f:face_planar_segment_plane_normal");

			add_face_1_ring_neighbor = add_face_property<std::set<int>>("f:face_1_ring_neighbor", std::set<int>());
			get_face_1_ring_neighbor = get_face_property<std::set<int>>("f:face_1_ring_neighbor");

			add_face_sampled_points = add_face_property<std::vector<int>>("f:face_sampled_points", std::vector<int>());
			get_face_sampled_points = get_face_property<std::vector<int>>("f:face_sampled_points");

			add_face_ele_sampled_points = add_face_property<std::vector<int>>("f:face_ele_sampled_points", std::vector<int>());
			get_face_ele_sampled_points = get_face_property<std::vector<int>>("f:face_ele_sampled_points");

			add_face_texture_points = add_face_property<std::vector<int>>("f:face_texture_points", std::vector<int>());
			get_face_texture_points = get_face_property<std::vector<int>>("f:face_texture_points");

			add_face_tile_index = add_face_property<int>("f:face_tile_index", -1);
			get_face_tile_index = get_face_property<int>("f:face_tile_index");

			add_face_component_visit = add_face_property<bool>("f:face_component_visit", false);
			get_face_component_visit = get_face_property<bool>("f:face_component_visit");

			add_face_interior_medialball_radius = add_face_property<std::vector<float>>("f:face_interior_medialball_radius",std::vector<float>());
			get_face_interior_medialball_radius = get_face_property<std::vector<float>>("f:face_interior_medialball_radius");

			add_face_rgb_x = add_face_property<std::vector<float>>("f:face_rgb_x", std::vector<float>());
			get_face_rgb_x = get_face_property<std::vector<float>>("f:face_rgb_x");

			add_face_rgb_y = add_face_property<std::vector<float>>("f:face_rgb_y", std::vector<float>());
			get_face_rgb_y = get_face_property<std::vector<float>>("f:face_rgb_y");

			add_face_rgb_z = add_face_property<std::vector<float>>("f:face_rgb_z", std::vector<float>());
			get_face_rgb_z = get_face_property<std::vector<float>>("f:face_rgb_z");

			add_face_hsv_x = add_face_property<std::vector<float>>("f:face_hsv_x", std::vector<float>());
			get_face_hsv_x = get_face_property<std::vector<float>>("f:face_hsv_x");

			add_face_hsv_y = add_face_property<std::vector<float>>("f:face_hsv_y", std::vector<float>());
			get_face_hsv_y = get_face_property<std::vector<float>>("f:face_hsv_y");

			add_face_hsv_z = add_face_property<std::vector<float>>("f:face_hsv_z", std::vector<float>());
			get_face_hsv_z = get_face_property<std::vector<float>>("f:face_hsv_z");

			add_face_all_predict_prob = add_face_property<std::vector<float>>("f:all_label_probabilities", std::vector<float>());
			get_face_all_predict_prob = get_face_property<std::vector<float>>("f:all_label_probabilities");

			add_face_sampled_points_id_index_map = add_face_property<std::map<int, int>>("f:face_sampled_points_id_index_map");
			get_face_sampled_points_id_index_map = get_face_property<std::map<int, int>>("f:face_sampled_points_id_index_map");

			add_face_ele_sampled_points_id_index_map = add_face_property<std::map<int, int>>("f:face_ele_sampled_points_id_index_map");
			get_face_ele_sampled_points_id_index_map = get_face_property<std::map<int, int>>("f:face_ele_sampled_points_id_index_map");

			add_face_texture_points_id_index_map = add_face_property<std::map<int, int>>("f:face_texture_points_id_index_map");
			get_face_texture_points_id_index_map = get_face_property<std::map<int, int>>("f:face_texture_points_id_index_map");

			add_face_spherical_neg_points = add_face_property<std::vector<vec3>>("f:face_spherical_neg_points", std::vector<vec3>());
			get_face_spherical_neg_points = get_face_property<std::vector<vec3>>("f:face_spherical_neg_points");

			add_face_segid_local_longrange_elevation_vec = add_face_property<std::vector<std::pair<int, float>>>("f:face_segid_local_longrange_elevation_vec", std::vector<std::pair<int, float>>(local_ground_segs, std::pair<int, float>(-1, FLT_MAX)));
			get_face_segid_local_longrange_elevation_vec = get_face_property<std::vector<std::pair<int, float>>>("f:face_segid_local_longrange_elevation_vec");

			add_face_mulsc_2_minmax_elevation = add_face_property<std::vector<std::pair<float, float>>>("f:face_multiscales_minmax_elevation", std::vector<std::pair<float, float>>(multi_scale_ele_radius.size(), std::pair<float, float>(FLT_MAX, -FLT_MAX)));
			get_face_mulsc_2_minmax_elevation = get_face_property<std::vector<std::pair<float, float>>>("f:face_multiscales_minmax_elevation");

			add_face_check = add_face_property<bool>("f:face_check", false);
			get_face_check = get_face_property<bool>("f:face_check");

			add_face_error_color = add_face_property<vec3>("f:face_error_color", vec3());
			get_face_error_color = get_face_property<vec3>("f:face_error_color");

			add_face_1ring_neighbor = add_face_property<std::vector<std::tuple<int, int, bool>>>("f:face_1ring_neighbor");
			get_face_1ring_neighbor = get_face_property<std::vector<std::tuple<int, int, bool>>>("f:face_1ring_neighbor");

			add_face_segid_local_elevation_vec = add_face_property<std::vector<std::pair<int, float>>>("f:face_segid_local_elevation_vec", std::vector<std::pair<int, float>>(local_ground_segs, std::pair<int, float>(-1, FLT_MAX)));
			get_face_segid_local_elevation_vec = get_face_property<std::vector<std::pair<int, float>>>("f:face_segid_local_elevation_vec");
		}


		//------------------Vertex Attributes-----------------------//
		SurfaceMesh::VertexProperty<vec3>
			get_points_normals,
			get_points_coord;

		SurfaceMesh::VertexProperty<int>
			add_vert_component_id, get_vert_component_id;

		SurfaceMesh::VertexProperty<float>
			add_vert_planarity, get_vert_planarity;

		SurfaceMesh::VertexProperty<bool>
			add_vert_check, get_vert_check;
		//------------------Edge Attributes-----------------------//
		SurfaceMesh::EdgeProperty<int>
			add_edge_boundary_truth, get_edge_boundary_truth,
			add_edge_boundary_predict, get_edge_boundary_predict;

		// ------------------Face Attributes-----------------------//
		SurfaceMesh::FaceProperty<bool>
			add_face_check, get_face_check,
			add_face_component_visit, get_face_component_visit;

		SurfaceMesh::FaceProperty<int>
			get_face_texnumber,
			get_face_truth_label,
			get_face_predict_label,
			add_face_segment_id, get_face_segment_id,
			add_face_smoothed_seg_id, get_face_smoothed_seg_id,
			add_face_tile_index, get_face_tile_index,
			add_face_spid_update, get_face_spid_update,
			add_face_component_id, get_face_component_id;

		SurfaceMesh::FaceProperty<float>
			add_face_predict_prob, get_face_predict_prob,
			add_face_1ring_avg_predict_prob, get_face_1ring_avg_predict_prob,
			add_face_area, get_face_area;

		SurfaceMesh::FaceProperty<vec3>
			get_face_normals,
			get_face_color,
			add_face_error_color, get_face_error_color,
			add_face_planar_segment_plane_normal, get_face_planar_segment_plane_normal,
			add_face_center, get_face_center;

		SurfaceMesh::FaceProperty<std::vector<vec3>>
			add_face_spherical_neg_points, get_face_spherical_neg_points;

		SurfaceMesh::FaceProperty<std::vector<float>>
			get_face_texcoord,
			add_face_rgb_x, get_face_rgb_x,
			add_face_rgb_y, get_face_rgb_y,
			add_face_rgb_z, get_face_rgb_z,
			add_face_hsv_x, get_face_hsv_x,
			add_face_hsv_y, get_face_hsv_y,
			add_face_hsv_z, get_face_hsv_z,
			add_face_all_predict_prob, get_face_all_predict_prob,
			add_face_interior_medialball_radius, get_face_interior_medialball_radius;

		SurfaceMesh::FaceProperty<std::vector<int>>
			add_face_sampled_points, get_face_sampled_points,
			add_face_ele_sampled_points, get_face_ele_sampled_points,
			add_face_texture_points, get_face_texture_points;

		SurfaceMesh::FaceProperty<std::map<int, int>>
			add_face_sampled_points_id_index_map, get_face_sampled_points_id_index_map,
			add_face_ele_sampled_points_id_index_map, get_face_ele_sampled_points_id_index_map,
			add_face_texture_points_id_index_map, get_face_texture_points_id_index_map;

		SurfaceMesh::FaceProperty<std::vector<std::pair<int, float>>>
			add_face_segid_local_longrange_elevation_vec, get_face_segid_local_longrange_elevation_vec;

		SurfaceMesh::FaceProperty<std::vector<std::pair<float, float>>>
			add_face_mulsc_2_minmax_elevation, get_face_mulsc_2_minmax_elevation;

		SurfaceMesh::FaceProperty<std::set<int>>
			add_face_1_ring_neighbor, get_face_1_ring_neighbor;

		SurfaceMesh::FaceProperty<std::vector<std::tuple<int, int, bool>>> //neighbor face id, index in the neighbor face' neighbor, check visited
			add_face_1ring_neighbor, get_face_1ring_neighbor;

		SurfaceMesh::FaceProperty<std::vector<std::pair<int, float>>>
			add_face_segid_local_elevation_vec, get_face_segid_local_elevation_vec;

		std::vector<std::string> texture_names;

		void remove_common_non_used_properties();
		void remove_non_used_properties_for_feature_mesh();
		void remove_non_used_properties_for_segment_mesh();
		void remove_non_used_properties_for_semantic_mesh();
		void SFMesh::remove_non_used_properties_for_pnp_mesh();
		void remove_non_used_properties_for_error_mesh();
		void remove_non_used_properties_for_visualization_mesh();

		void add_segment_properties(std::vector<SFMesh::FaceProperty < float >> &, std::vector<std::pair<std::string, int>> &, const int);

		void add_selected_feature_properties
		(
			std::vector< std::vector <int>> &,
			std::vector<int> &,
			std::vector< std::vector<float>> &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &
		);

		float mesh_area = 0.0f;
		std::vector<float> class_area = std::vector<float>(0, 0.0f);
		float SFMesh::FaceArea(SFMesh::Face&);

		~SFMesh()
		{
			this->vertex_properties().clear();
			this->face_properties().clear();
			this->edge_properties().clear();
			this->free_memory();
			this->clear();
		};
	};

	inline float SFMesh::FaceArea(SFMesh::Face& f)
	{
		vec3 v0, v1, v2;
		int ind = 0;
		for (auto v : this->vertices(f))
		{
			if (ind == 0)
				v0 = this->get_points_coord[v];
			else if (ind == 1)
				v1 = this->get_points_coord[v];
			else if (ind == 2)
				v2 = this->get_points_coord[v];
			++ind;
		}

		float a = (v1 - v0).norm();
		float b = (v2 - v1).norm();
		float c = (v2 - v0).norm();
		float s = (a + b + c)*0.5;

		return  sqrt(s*(s - a)*(s - b)*(s - c));
	}

	inline void SFMesh::remove_common_non_used_properties()
	{
		//--------------Remove vertices properties------------------
		this->remove_vertex_property(this->get_points_normals);
		this->remove_vertex_property(this->get_vert_check);
		this->remove_vertex_property(this->get_vert_planarity);
		this->remove_vertex_property(this->get_vert_component_id);

		//--------------Remove edges properties------------------
		this->remove_edge_property(this->get_edge_boundary_truth);
		this->remove_edge_property(this->get_edge_boundary_predict);

		//--------------Remove faces properties------------------
		this->remove_face_property(this->get_face_check);
		this->remove_face_property(this->get_face_center);
		this->remove_face_property(this->get_face_area);
		this->remove_face_property(this->get_face_sampled_points);
		this->remove_face_property(this->get_face_ele_sampled_points);
		this->remove_face_property(this->get_face_texture_points);
		this->remove_face_property(this->get_face_interior_medialball_radius);
		this->remove_face_property(this->get_face_sampled_points_id_index_map);
		this->remove_face_property(this->get_face_ele_sampled_points_id_index_map);
		this->remove_face_property(this->get_face_texture_points_id_index_map);
		this->remove_face_property(this->get_face_segid_local_longrange_elevation_vec);
		this->remove_face_property(this->get_face_mulsc_2_minmax_elevation);
		this->remove_face_property(this->get_face_spherical_neg_points);
		this->remove_face_property(this->get_face_1_ring_neighbor);
		this->remove_face_property(this->get_face_component_id);
		this->remove_face_property(this->get_face_error_color);
		this->remove_face_property(this->get_face_smoothed_seg_id);
		this->remove_face_property(this->get_face_planar_segment_plane_normal);
		this->remove_face_property(this->get_face_1ring_neighbor);
		this->remove_face_property(this->get_face_spid_update);
		this->remove_face_property(this->get_face_segid_local_elevation_vec);

		this->remove_face_property(get_face_rgb_x);
		this->remove_face_property(get_face_rgb_y);
		this->remove_face_property(get_face_rgb_z);
		this->remove_face_property(get_face_hsv_x);
		this->remove_face_property(get_face_hsv_y);
		this->remove_face_property(get_face_hsv_z);
	}

	inline void SFMesh::remove_non_used_properties_for_segment_mesh()
	{
		//--------------Remove faces properties------------------
		this->remove_face_property(this->get_face_predict_prob);
		this->remove_face_property(this->get_face_truth_label);
		this->remove_face_property(this->get_face_predict_label);

		this->remove_face_property(this->get_face_1ring_avg_predict_prob);
		this->remove_face_property(this->get_face_all_predict_prob);
	}

	inline void SFMesh::remove_non_used_properties_for_semantic_mesh()
	{
		this->remove_face_property(this->get_face_1ring_avg_predict_prob);
		this->remove_face_property(this->get_face_all_predict_prob);
	}

	inline void SFMesh::remove_non_used_properties_for_pnp_mesh()
	{
		this->remove_face_property(this->get_face_1ring_avg_predict_prob);
		this->remove_face_property(this->get_face_all_predict_prob);

		this->remove_face_property(this->get_face_predict_label);
		this->remove_face_property(this->get_face_segment_id);
		this->remove_face_property(this->get_face_predict_prob);
	}

	inline void SFMesh::remove_non_used_properties_for_error_mesh()
	{
		this->remove_face_property(this->get_face_truth_label);
		this->remove_face_property(this->get_face_predict_label);
		this->remove_face_property(this->get_face_1ring_avg_predict_prob);
		this->remove_face_property(this->get_face_all_predict_prob);
		this->remove_face_property(this->get_face_segment_id);
		this->remove_face_property(this->get_face_predict_prob);
	}

	inline void SFMesh::remove_non_used_properties_for_visualization_mesh()
	{
		this->remove_face_property(this->get_face_segment_id);
		this->remove_face_property(this->get_face_predict_prob);
		this->remove_face_property(this->get_face_truth_label);
		this->remove_face_property(this->get_face_predict_label);
		this->remove_face_property(this->get_face_color);
	}

	inline void SFMesh::remove_non_used_properties_for_feature_mesh()
	{
		this->remove_face_property(this->get_face_predict_prob);
		this->remove_face_property(this->get_face_truth_label);
		this->remove_face_property(this->get_face_predict_label);
		this->remove_face_property(this->get_face_segment_id);
		this->remove_face_property(this->get_face_color);
		this->remove_face_property(this->get_face_property<std::vector<int>>("f:mesh_faces_id"));
	}

	inline void SFMesh::add_segment_properties
	(
		std::vector<SFMesh::FaceProperty < float >> &feas,
		std::vector<std::pair<std::string, int>> &segment_feature_base_names,
		const int selected_fea
	)
	{
		for (int fea_i = 0; fea_i < segment_feature_base_names.size(); ++fea_i)
		{
			std::string fea_temp = selected_mesh_face_features[selected_fea].first + segment_feature_base_names[fea_i].first;
			this->add_face_property<float>(fea_temp);
			feas.emplace_back(this->get_face_property<float>(fea_temp));
		}
	}

	inline void SFMesh::add_selected_feature_properties
	(
		std::vector< std::vector <int>> &seg_face_vec,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas
	)
	{
		std::vector<SFMesh::FaceProperty < float >> f_basic_feas, f_eigen_feas, f_color_feas, f_mulsc_ele_feas;
		std::map<int, bool> use_feas = { {0, false}, {1, false}, {2, false}, {3, false}};
		//controlled by muti scale2 elevations 
		std::vector<std::pair<std::string, int>> temp_ele_feature_base_names;
		if (!multi_scale_ele_radius.empty())
			for (int sci = 0; sci < multi_scale_ele_radius.size(); ++sci)
				temp_ele_feature_base_names.push_back(std::pair<std::string, int>(std::to_string(sci), sci));

		for (auto sf : selected_mesh_face_features)
		{
			switch (sf.second)
			{
			case 0: this->add_segment_properties(f_basic_feas, basic_feature_base_names, 0); use_feas[0] = true; break;
			case 1: this->add_segment_properties(f_eigen_feas, eigen_feature_base_names, 1); use_feas[1] = true; break;
			case 2: this->add_segment_properties(f_color_feas, color_feature_base_names, 2); use_feas[2] = true; break;
			case 3: this->add_segment_properties(f_mulsc_ele_feas, temp_ele_feature_base_names, 3); use_feas[3] = true; break;
			}
		}
		this->add_face_property<int>("f:" + label_definition, -1);

		for (int seg_i = 0; seg_i < seg_face_vec.size(); ++seg_i)
		{
			for (int fi = 0 ; fi < seg_face_vec[seg_i].size(); ++fi)
			{
				SFMesh::Face fdx(seg_face_vec[seg_i][fi]);
				this->get_face_property<int>("f:" + label_definition)[fdx] = seg_truth[seg_i];
				if (use_feas[0])
					for (int shp_i = 0; shp_i < basic_feas[seg_i].size(); ++shp_i)
						f_basic_feas[shp_i][fdx] = basic_feas[seg_i][shp_i];

				if (use_feas[3])
					for (int ele_f_i = 0; ele_f_i < mulsc_ele_feas[seg_i].size(); ++ele_f_i)
						f_mulsc_ele_feas[ele_f_i][fdx] = mulsc_ele_feas[seg_i][ele_f_i];

				if (use_feas[1])
				{
					for (int geo_i = 0; geo_i < eigen_feas[seg_i].size(); ++geo_i)
						f_eigen_feas[geo_i][fdx] = eigen_feas[seg_i][geo_i];
				}

				if (use_feas[2])
				{
					for (int tex_i = 0; tex_i < color_feas[seg_i].size(); ++tex_i)
						f_color_feas[tex_i][fdx] = color_feas[seg_i][tex_i];
				}
			}
		}
	}
}

#endif