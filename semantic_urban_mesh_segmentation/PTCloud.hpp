/*
*   Name        : PTCloud.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for point cloud definition, including features and sampled point clouds
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
#ifndef semantic_mesh_segmentation__PTCloud_HPP
#define semantic_mesh_segmentation__PTCloud_HPP

#pragma once

#include <omp.h>
#include <easy3d/point_cloud.h>
#include <easy3d/surface_mesh.h>
#include "parameters.hpp"

using namespace easy3d;

namespace semantic_mesh_segmentation
{
	typedef std::pair
		<
		int,		 //1.point index
		float        //2.local elevation 
		> ptx_z;

	typedef std::pair
		<
		float,		 //1.min local elevation
		float        //2.max local elevation 
		> ptx_minmax;

	inline bool smaller_segment_size
	(
		std::pair<int, std::vector<int>> seg_1,
		std::pair<int, std::vector<int>> seg_2
	)
	{
		if (seg_1.second.size() < seg_2.second.size())
			return true;
		else
			return false;
	}

	class PTCloud :public easy3d::PointCloud
	{
	public:
		PTCloud()
		{
			add_points_rgb_x = add_vertex_property<float>("v:points_rgb_x", 0.0f);
			get_points_rgb_x = get_vertex_property<float>("v:points_rgb_x");

			add_points_rgb_y = add_vertex_property<float>("v:points_rgb_y", 0.0f);
			get_points_rgb_y = get_vertex_property<float>("v:points_rgb_y");

			add_points_rgb_z = add_vertex_property<float>("v:points_rgb_z", 0.0f);
			get_points_rgb_z = get_vertex_property<float>("v:points_rgb_z");

			add_points_hsv_x = add_vertex_property<float>("v:points_hsv_x", 0.0f);
			get_points_hsv_x = get_vertex_property<float>("v:points_hsv_x");

			add_points_hsv_y = add_vertex_property<float>("v:points_hsv_y", 0.0f);
			get_points_hsv_y = get_vertex_property<float>("v:points_hsv_y");

			add_points_hsv_z = add_vertex_property<float>("v:points_hsv_z", 0.0f);
			get_points_hsv_z = get_vertex_property<float>("v:points_hsv_z");

			add_points_face_belong_id = add_vertex_property<int>("v:points_face_belong_id", -1);
			get_points_face_belong_id = get_vertex_property<int>("v:points_face_belong_id");

			add_points_ground_truth = add_vertex_property<int>("v:label", -1);
			get_points_ground_truth = get_vertex_property<int>("v:label");

			add_points_tile_index = add_vertex_property<int>("v:points_tile_index", -1);
			get_points_tile_index = get_vertex_property<int>("v:points_tile_index");

			add_points_on_mesh_border = add_vertex_property<bool>("v:points_on_mesh_border", false);
			get_points_on_mesh_border = get_vertex_property<bool>("v:points_on_mesh_border");
	
			add_points_face_belong_ids = add_vertex_property<std::vector<int>>("v:points_face_belong_ids", std::vector<int>());
			get_points_face_belong_ids = get_vertex_property<std::vector<int>>("v:points_face_belong_ids");

			add_points_face_ele_belong_ids = add_vertex_property<std::vector<int>>("v:points_face_ele_belong_ids", std::vector<int>());
			get_points_face_ele_belong_ids = get_vertex_property<std::vector<int>>("v:points_face_ele_belong_ids");

			add_point_mulscale_2_elevation = add_vertex_property<std::vector<ptx_minmax>>("v:point_mulscale_2_elevation", std::vector<ptx_minmax>(multi_scale_ele_radius.size(), ptx_minmax(FLT_MAX, -FLT_MAX)));
			get_point_mulscale_2_elevation = get_vertex_property<std::vector<ptx_minmax>>("v:point_mulscale_2_elevation");

			add_points_use_seg_truth = add_vertex_property<bool>("v:points_use_seg_truth", false);
			get_points_use_seg_truth = get_vertex_property<bool>("v:points_use_seg_truth");
		}

		//------------------Vertex Attributes-----------------------//
		PointCloud::VertexProperty<int>
			add_points_face_belong_id, get_points_face_belong_id,
			add_points_ground_truth, get_points_ground_truth,
			add_points_tile_index, get_points_tile_index;

		PointCloud::VertexProperty<std::vector<int>>
			add_points_face_belong_ids, get_points_face_belong_ids,
			add_points_face_ele_belong_ids, get_points_face_ele_belong_ids;

		PointCloud::VertexProperty<bool>
			add_points_use_seg_truth, get_points_use_seg_truth,
			add_points_on_mesh_border, get_points_on_mesh_border;

		PointCloud::VertexProperty<float>
			add_points_rgb_x, get_points_rgb_x,
			add_points_rgb_y, get_points_rgb_y,
			add_points_rgb_z, get_points_rgb_z,
			add_points_hsv_x, get_points_hsv_x,
			add_points_hsv_y, get_points_hsv_y,
			add_points_hsv_z, get_points_hsv_z;

		PointCloud::VertexProperty<vec3>
			get_points_normals,
			get_points_coord,
			get_points_color;

		PointCloud::VertexProperty<std::vector<ptx_minmax>>
			add_point_mulscale_2_elevation, get_point_mulscale_2_elevation;

		void remove_all_properties();

		void add_segment_properties(PTCloud::VertexProperty < std::vector<int> > &, std::string &);
		void add_segment_properties(PTCloud::VertexProperty < std::vector<float> > &, std::string &);

		void add_all_feature_properties
		(
			std::vector<std::vector<int>> &,
			std::vector<int> &,
			std::vector<int> &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &
		);

		void add_selected_feature_properties_for_GCN
		(
			std::vector<std::vector<int>> &,
			std::vector<int> &,
			std::vector<int> &,
			std::vector< std::vector<float>> &,
			std::vector< std::vector<float> > &,
			std::vector< std::vector<float> > &
		);

		void add_segment_properties(std::vector<PTCloud::VertexProperty < float >> &, std::vector<std::pair<std::string, int>> &, std::map<int, bool> &, const int);

		~PTCloud()
		{
			this->vertex_properties().clear();
			this->clear();
			this->free_memory();
		};
	};

	inline void PTCloud::remove_all_properties()
	{
		//remove face properties
		//this->remove_vertex_property(this->get_points_normals);
		//this->remove_vertex_property(this->get_points_color);
		///this->remove_vertex_property(this->get_points_face_belong_id);
		///this->remove_vertex_property(this->get_points_face_ele_belong_ids);
		this->remove_vertex_property(get_points_rgb_x);
		this->remove_vertex_property(get_points_rgb_y);
		this->remove_vertex_property(get_points_rgb_z);
		this->remove_vertex_property(get_points_hsv_x);
		this->remove_vertex_property(get_points_hsv_y);
		this->remove_vertex_property(get_points_hsv_z);
		this->remove_vertex_property(get_points_on_mesh_border);
	}

	//--- add feature properties onto point cloud and expand to minimum outuput type std::vector<float>---
	inline void PTCloud::add_segment_properties
	(
		PTCloud::VertexProperty < std::vector<int> > &feas,
		std::string &v_fea
	)
	{
		//j = features
		this->add_vertex_property<std::vector<int>>(v_fea);
		feas = this->get_vertex_property<std::vector<int>>(v_fea);
	}

	inline void PTCloud::add_segment_properties
	(
		PTCloud::VertexProperty < std::vector<float> > &feas,
		std::string &v_fea
	)
	{
		//j = features
		this->add_vertex_property<std::vector<float>>(v_fea);
		feas = this->get_vertex_property<std::vector<float>>(v_fea);
	}

	inline void PTCloud::add_all_feature_properties
	(
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas
	)
	{
		PTCloud::VertexProperty < std::vector<int> > p_faces;
		PTCloud::VertexProperty < std::vector<float> > p_basic_feas, p_mulsc_ele_feas;
		PTCloud::VertexProperty < std::vector<float> >	p_eigen_feas, p_color_feas;

		std::map<int, bool> use_feas = { {0, false}, {1, false}, {2, false}, {3, false} };

		this->add_vertex_property<int>("v:segment_id", -1);
		auto seg_id = this->get_vertex_property<int>("v:segment_id");
		add_segment_properties(p_faces, std::string("v:mesh_faces_id"));

		//properties for watershed segmentation
		this->add_vertex_property<float>("v:segment_relative_elevation", -1);
		auto get_seg_relative_ele = this->get_vertex_property<float>("v:segment_relative_elevation");

		this->add_vertex_property<std::vector<float>>("v:segment_plane_params", std::vector<float>());
		auto seg_plane_param = this->get_vertex_property<std::vector<float>>("v:segment_plane_params");

		for (auto sf : selected_pcl_vertex_features)
		{
			switch (sf.second)
			{
			case 0: this->add_segment_properties(p_basic_feas, sf.first); use_feas[0] = true; break;
			case 1: this->add_segment_properties(p_eigen_feas, sf.first); use_feas[1] = true; break;
			case 2: this->add_segment_properties(p_color_feas, sf.first); use_feas[2] = true; break;
			case 3: this->add_segment_properties(p_mulsc_ele_feas, sf.first); use_feas[3] = true; break;
			}
		}

		int pi = 0;
		for (int sfi = 0; sfi < basic_feas.size(); ++sfi)
		{
			PTCloud::Vertex ptx(sfi);
			seg_id[ptx] = seg_ids[sfi];

			//for features
			this->get_points_ground_truth[ptx] = seg_truth[sfi];
			p_faces[ptx].insert(p_faces[ptx].end(), seg_face_vec[sfi].begin(), seg_face_vec[sfi].end());
			if (use_feas[0])
				p_basic_feas[ptx].insert(p_basic_feas[ptx].end(), basic_feas[sfi].begin(), basic_feas[sfi].end());
			if (use_feas[3])
				p_mulsc_ele_feas[ptx].insert(p_mulsc_ele_feas[ptx].end(), mulsc_ele_feas[sfi].begin(), mulsc_ele_feas[sfi].end());

			if (use_feas[1])
				p_eigen_feas[ptx].insert(p_eigen_feas[ptx].end(), eigen_feas[sfi].begin(), eigen_feas[sfi].end());
			if (use_feas[2])
				p_color_feas[ptx].insert(p_color_feas[ptx].end(), color_feas[sfi].begin(), color_feas[sfi].end());
		}
	}

	//---add feature properties and expand to minimum output type <float> for using ---
	inline void PTCloud::add_segment_properties
	(
		std::vector<PTCloud::VertexProperty < float >> &feas,
		std::vector<std::pair<std::string, int>> &segment_feature_base_names,
		std::map<int, bool> &selecte_feas,
		const int selected_fea
	)
	{
		for (int fea_i = 0; fea_i < segment_feature_base_names.size(); ++fea_i)
		{
			if (selecte_feas[fea_i])
			{
				std::string fea_temp = selected_pcl_vertex_features[selected_fea].first + '_' + segment_feature_base_names[fea_i].first;
				this->add_vertex_property<float>(fea_temp);
				feas.emplace_back(this->get_vertex_property<float>(fea_temp));
			}
		}
	}
	
	//--- get feature properties from point cloud ---
	inline void get_segment_properties
	(
		PointCloud *pcl,
		PointCloud::VertexProperty < std::vector<int> > &feas,
		std::string &v_fea
	)
	{
		//j = features
		feas = pcl->get_vertex_property<std::vector<int>>(v_fea);
	}

	inline void get_segment_properties
	(
		PointCloud *pcl,
		PointCloud::VertexProperty < std::vector<float> > &feas,
		std::string &v_fea
	)
	{
		//j = features
		feas = pcl->get_vertex_property<std::vector<float>>(v_fea);
	}

	inline void get_all_feature_properties_from_feature_point_cloud
	(
		PointCloud *pcl,
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> & basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas
	)
	{
		std::cout << "  Get all feature properties from feature point cloud, use ";
		const double t_total = omp_get_wtime();
		PointCloud::VertexProperty < std::vector<int> > p_faces;
		PointCloud::VertexProperty < std::vector<float> > p_basic_feas, p_mulsc_ele_feas;
		PointCloud::VertexProperty < std::vector<float> > p_eigen_feas, p_color_feas;
		std::map<int, bool> use_feas = { {0, false}, {1, false}, {2, false}, {3, false}};

		get_segment_properties(pcl, p_faces, std::string("v:mesh_faces_id"));
		for (auto sf : selected_pcl_vertex_features)
		{
			switch (sf.second)
			{
			case 0: get_segment_properties(pcl, p_basic_feas, sf.first); use_feas[0] = true; break;
			case 1: get_segment_properties(pcl, p_eigen_feas, sf.first); use_feas[1] = true; break;
			case 2: get_segment_properties(pcl, p_color_feas, sf.first); use_feas[2] = true; break;
			case 3: get_segment_properties(pcl, p_mulsc_ele_feas, sf.first); use_feas[3] = true; break;
			}
		}

		PointCloud::Vertex p_ini(0);//feature size
		seg_face_vec = std::vector< std::vector<int>>(pcl->n_vertices(), std::vector<int>());
		basic_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());
		mulsc_ele_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());

		//i,j = segments, features
		eigen_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());
		color_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());

		for (int sfi = 0; sfi < pcl->vertices_size(); ++sfi)
		{
			PointCloud::Vertex ptx(sfi);
			seg_ids.push_back(pcl->get_vertex_property<int>("v:segment_id")[ptx]);
			seg_truth.push_back(pcl->get_vertex_property<int>("v:label")[ptx]);
			seg_face_vec[sfi].insert(seg_face_vec[sfi].end(), p_faces[ptx].begin(), p_faces[ptx].end());
			if (use_feas[0])
				basic_feas[sfi].insert(basic_feas[sfi].end(), p_basic_feas[ptx].begin(), p_basic_feas[ptx].end());
			if (use_feas[3])
				mulsc_ele_feas[sfi].insert(mulsc_ele_feas[sfi].end(), p_mulsc_ele_feas[ptx].begin(), p_mulsc_ele_feas[ptx].end());

			if (use_feas[1])
				eigen_feas[sfi].insert(eigen_feas[sfi].end(), p_eigen_feas[ptx].begin(), p_eigen_feas[ptx].end());
			if (use_feas[2])
				color_feas[sfi].insert(color_feas[sfi].end(), p_color_feas[ptx].begin(), p_color_feas[ptx].end());
		}
		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}


	//--- add selected features output for GCN ---
	inline void PTCloud::add_selected_feature_properties_for_GCN
	(
		std::vector<std::vector<int>> &seg_pcl_vec,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> & basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas
	)
	{
		std::vector<PTCloud::VertexProperty< float >> v_basic_feas, v_eigen_feas, v_color_feas;
		this->add_vertex_property<int>("v:point_segment_id", -1);
		for (int selc_i = 0; selc_i < use_feas.size(); ++selc_i)
		{
			if (use_feas[selc_i])
			{
				switch (selc_i)
				{
					case 0: this->add_segment_properties(v_basic_feas, basic_feature_base_names, use_basic_features, selc_i); break;
					case 1: this->add_segment_properties(v_eigen_feas, eigen_feature_base_names, use_eigen_features, selc_i); break;
					case 2: this->add_segment_properties(v_color_feas, color_feature_base_names, use_color_features, selc_i); break;
				}
			}
		}

		for (int seg_i = 0; seg_i < seg_pcl_vec.size(); ++seg_i)
		{
			for (int vi = 0; vi < seg_pcl_vec[seg_i].size(); ++vi)
			{
				PTCloud::Vertex vtx(seg_pcl_vec[seg_i][vi]);
				this->get_vertex_property<int>("v:point_segment_id")[vtx] = seg_ids[seg_i];
				if (sampling_strategy == 1 || sampling_strategy == 2)
				{
					if (this->get_points_use_seg_truth[vtx])
					{
						if (seg_truth[seg_i] != -1)
							this->get_points_ground_truth[vtx] = seg_truth[seg_i];
						else
							this->get_points_ground_truth[vtx] = -1;
					}
					else
						this->get_points_ground_truth[vtx] = -1;
				}

				if (use_feas[0])
				{
					int fea_ind = 0;
					for (int bas_i = 0; bas_i < basic_feas[seg_i].size(); ++bas_i)
					{
						if (use_basic_features[bas_i])
						{
							v_basic_feas[fea_ind][vtx] = basic_feas[seg_i][bas_i];
							++fea_ind;
						}
					}
				}

				if (use_feas[1])
				{
					int fea_ind = 0;
					for (int eng_i = 0; eng_i < eigen_feas[seg_i].size(); ++eng_i)
					{
						if (use_eigen_features[eng_i])
						{
							v_eigen_feas[fea_ind][vtx] = eigen_feas[seg_i][eng_i];
							++fea_ind;
						}
					}
				}

				if (use_feas[2])
				{
					int fea_ind = 0;
					for (int col_i = 0; col_i < color_feas[seg_i].size(); ++col_i)
					{
						if (use_color_features[col_i])
						{
							v_color_feas[fea_ind][vtx] = color_feas[seg_i][col_i];
							++fea_ind;
						}
					}
				}
			}
		}
	}
}

#endif