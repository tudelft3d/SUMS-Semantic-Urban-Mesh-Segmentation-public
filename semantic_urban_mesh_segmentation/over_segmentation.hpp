/*
*   Name        : over_segmentation.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh over-segmentation
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
#ifndef semantic_mesh_segmentation__OVER_SEGMENTATION_HPP
#define semantic_mesh_segmentation__OVER_SEGMENTATION_HPP

#include <omp.h>
#include <queue>
#include <map>
#include <algorithm>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <opencv2/opencv.hpp>

#include "mesh_io.hpp"
#include "super_segment.hpp"
#include "feature_computation.hpp"
#include "mesh_classifier.hpp"

namespace semantic_mesh_segmentation
{
	//--- bool comparision ---
	template<typename T_pair>
	inline bool larger_planarity
	(
		T_pair vpl_1,
		T_pair vpl_2
	)
	{
		if (vpl_1.second > vpl_2.second)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	inline bool smaller_segments_area
	(
		superfacets spf_1,
		superfacets spf_2
	)
	{
		if (spf_1.sum_area < spf_2.sum_area)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	inline std::vector<std::string> get_input_names_with_delim
	(
		const std::string delim_in,
		std::string &str_in
	)
	{
		char *delim = new char[delim_in.length() + 1];
		strcpy(delim, delim_in.c_str());
		char *char_in = new char[str_in.length() + 1];
		strcpy(char_in, str_in.c_str());
		char *tmp = strtok(char_in, delim);
		std::vector<std::string> str_out;

		str_out.push_back(tmp);
		while (tmp != NULL)
		{
			tmp = strtok(NULL, delim);
			if (tmp != NULL)
				str_out.push_back(tmp);
		}
		return str_out;
	}

	//checking segment pair
	inline void segment_pair_checking
	(
		std::pair<int, int> &temp_pair1,
		std::pair<int, int> &temp_pair2,
		std::vector<std::pair<int, int>> &spf_edges,
		std::map<std::pair<int, int>, bool> &check_segneg_visited
	)
	{
		auto it_1 = check_segneg_visited.find(temp_pair1);
		auto it_2 = check_segneg_visited.find(temp_pair2);
		if (it_1 == check_segneg_visited.end() && it_2 == check_segneg_visited.end())
		{
			check_segneg_visited[temp_pair1] = true;
			check_segneg_visited[temp_pair2] = true;
			spf_edges.emplace_back(temp_pair1);
		}
	}

	//-------------------------------------------------------------------------------------------------------------------------------
	//functions declare
	void finding_segments_neighbor
	(
		SFMesh *,
		std::vector<superfacets> & //for all segment only
	);

	int separate_connected_components
	(
		SFMesh *,
		std::vector<superfacets>&,
		int &
	);

	void initial_vertex_planarity
	(
		SFMesh *,
		PTCloud* ,
		std::vector<vertex_planarity> &
	);

	bool region_growing_conditions
	(
		SFMesh*,
		vec3 &,
		SFMesh::Face &,
		SFMesh::Face &,
		vec4 &,
		vec3 &,
		float &
	);

	void mohaverdi_region_growing_on_mesh
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<vertex_planarity> &,
		const int &,
		int &
	);

	void perform_mohaverdi_oversegmentation
	(
		SFMesh *,
		std::vector<superfacets>&,
		std::vector<vertex_planarity> &,
		std::map<int, int> &,
		std::map<int, std::vector<int>> &
	);

	void cgal_regiongrowing
	(
		PTCloud *,
		SFMesh *,
		std::vector<superfacets> &,
		std::map<int, int> &,
		std::map<int, std::vector<int>> &
	);

	void cgal_regiongrowing
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::map<int, int> &,
		std::map<int, std::vector<int>> &
	);


	void finding_adjacent_segments
	(
		SFMesh *,
		PTCloud *,
		std::vector<superfacets> &,
		std::map<int, int> &,
		std::map<int, int> &,
		std::map<int, std::vector<int>> &
	);

	void segment_region_growing
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void merge_segments
	(
		SFMesh *,
		PTCloud *,
		std::vector<superfacets> &,
		std::vector<superfacets> &,
		std::map<int, int> &,
		std::map<int, int> &,
		std::map<int, std::vector<int>> &
	);

	void get_segments_color
	(
		SFMesh*,
		std::vector<superfacets> &
	);

	void plane_fitting_unary_term
	(
		SFMesh* ,
		SFMesh::Vertex &,
		SFMesh::Face &,
		vec4 &,
		std::pair<int, float> &,
		float &
	);

	void rearrange_segments
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void assign_initial_segments_on_mesh
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void parsing_with_planar_region
	(
		SFMesh *,
		std::vector<superfacets> &,
		const int 
	);

	void planar_segment_merge_preprocessing
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::map<int, std::vector<int>> &
	);

	void finding_adjacent_segments_based_on_connectivity
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::map<int, std::vector<int>> &
	);

	void segment_region_growing_pssnet
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void get_one_ring_neighbor
	(
		SFMesh *,
		SFMesh::Vertex &,
		SFMesh::Face &
	);

	void face_global_smooth_on_mesh(SFMesh *);

	void merge_single_segments
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void merge_single_triangles
	(
		SFMesh *,
		std::vector<superfacets> &
	);

	void rearrange_connected_segments
	(
		SFMesh *,
		std::vector<superfacets> &
	);

	void merge_planar_segments
	(
		SFMesh *,
		std::vector<superfacets> &,
		std::vector<superfacets> &
	);

	void perform_pnpmrf_oversegmentation_on_mesh
	(
		SFMesh *,
		std::vector<superfacets>& ,
		std::vector<vertex_planarity> &
	);

	void get_superfacets
	(
		SFMesh *,
		PTCloud *,
		PTCloud *,
		std::vector<superfacets>& 
	);

	void construct_exterior_mat_relations
	(
		SFMesh *,
		PTCloud *,
		std::vector<superfacets>& 
	);

	void local_elevation_for_pssnet_pointcloud
	(
		SFMesh *,
		PTCloud*,
		std::vector<superfacets>& 
	);

	void segment_geometric_relationships
	(
		SFMesh *,
		PTCloud *,
		std::vector<superfacets> &,
		std::vector<std::pair<int, int>> &,
		std::vector<std::pair<int, int>> &,
		std::map<std::pair<int, int>, bool> &,
		std::map<std::pair<int, int>, bool> &
	);

	void remove_close_vertices_for_delaunay
	(
		std::vector<superfacets> &,
		std::vector<Point_3> &,
		std::map<Point_3, int> &,
		std::map<Point_3, int> &
	);

	void construct_segment_sampled_point_edges
	(
		std::vector<superfacets> &,
		std::vector<std::pair<int, int>> &,
		std::vector<std::pair<int, int>> &,
		std::map<std::pair<int, int>, bool> &
	);
}

#endif//MESH_SEGMENTATION__SEGMENT_FILE_HPP
