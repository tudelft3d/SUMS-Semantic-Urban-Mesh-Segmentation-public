/*
*   Name        : super_segment.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh segment definition
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
#ifndef semantic_mesh_segmentation__SUPER_SEGMENT_HPP
#define semantic_mesh_segmentation__SUPER_SEGMENT_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>

#include "SFMesh.hpp"
#include "parameters.hpp"
#include "CGAL_Define.hpp"

namespace semantic_mesh_segmentation
{
	typedef std::pair
		<
		SFMesh::Vertex, //1.triangle vertex
		float        //2.planarity feature value 
		> vertex_planarity;

	class superfacets
	{
	public:
		int id, index, ground_truth, predict, local_ground_longrange_segid, label, local_ground_id;
		float sum_area, avg_ele, relative_ele, local_longrange_ground_ele, border_length;
		bool contain_mesh_border_facets, seg_visited, is_non_planar, is_ignore;
		std::vector<int> neighbor_ids, ExactVec, face_id_vec, exterior_mat_ids, sampled_points_ids, local_ground_ids, parallelism_neg_ids;
		std::pair<float, float> local_elevation_minmax;
		std::vector<std::pair<int, float>> segid_local_longrange_zmin_vec;

		std::vector<SFMesh::Face> face_vec;
		std::vector<Point_3> ExactVec_CGAL;

		easy3d::vec3 avg_center;
		easy3d::vec4 plane_parameter;
		std::vector<int> sampled_points;
		Plane plane_cgal;
		std::vector<vec3> sampled_pts_coords, border_points, alpha_border_points;
		std::vector<std::pair<int, float>> segid_local_zmin_vec;
		std::vector<std::tuple<int, easy3d::vec3, bool>> segment_alpha_vertices;//global index, coords, is border
		std::vector<std::tuple<int, std::pair<int, int>, bool>> segment_alpha_edges;//global index, global vs and vt, is border
		superfacets
		()
		{
			ground_truth = -1;
			predict = -1;
			label = -1;
			local_ground_longrange_segid = -1;
			sampled_points = std::vector<int>();
			local_elevation_minmax = std::pair<float, float>(FLT_MAX, -FLT_MAX);
			sum_area = 0.0f;
			avg_ele = 0.0f;
			relative_ele = 0.0f;
			local_longrange_ground_ele = 0.0f;
			contain_mesh_border_facets = false;
			seg_visited = false;
			is_non_planar = false;
			is_ignore = false;
			local_ground_id = -1;
		}

		superfacets
		(
			int id_,
			std::vector<SFMesh::Face> face_vec_,
			easy3d::vec4 plane_parameter_
		) :
			id(id_),
			face_vec(face_vec_),
			plane_parameter(plane_parameter_)
		{
			ground_truth = -1;
			predict = -1;
			label = -1;
			local_ground_longrange_segid = -1;
			sampled_points = std::vector<int>();
			local_elevation_minmax = std::pair<float, float>(FLT_MAX, -FLT_MAX);
			sum_area = 0.0f;
			avg_ele = 0.0f;
			relative_ele = 0.0f;
			local_longrange_ground_ele = 0.0f;
			contain_mesh_border_facets = false;
			seg_visited = false;
			is_non_planar = false;
			is_ignore = false;
			local_ground_id = -1;
		}

		superfacets
		(
			int id_,
			std::vector<SFMesh::Face> face_vec_,
			easy3d::vec4 plane_parameter_,
			std::vector<Point_3> ExactVec_CGAL_
		) :
			id(id_),
			face_vec(face_vec_),
			plane_parameter(plane_parameter_),
			ExactVec_CGAL(ExactVec_CGAL_)
		{
			ground_truth = 0;
			local_ground_longrange_segid = -1;
			label = -1;
			sampled_points = std::vector<int>();
			local_elevation_minmax = std::pair<float, float>(FLT_MAX, -FLT_MAX);
			sum_area = 0.0f;
			avg_ele = 0.0f;
			relative_ele = 0.0f;
			local_longrange_ground_ele = 0.0f;
			contain_mesh_border_facets = false;
			seg_visited = false;
			is_non_planar = false;
			is_ignore = false;
			local_ground_id = -1;
		}

		superfacets
		(
			int id_,
			std::vector<SFMesh::Face> face_vec_
		) :
			id(id_),
			face_vec(face_vec_)
		{
			ground_truth = 0;
			local_ground_longrange_segid = -1;
			label = -1;
			sampled_points = std::vector<int>();
			local_elevation_minmax = std::pair<float, float>(FLT_MAX, -FLT_MAX);
			sum_area = 0.0f;
			avg_ele = 0.0f;
			relative_ele = 0.0f;
			local_longrange_ground_ele = 0.0f;
			contain_mesh_border_facets = false;
			seg_visited = false;
			is_non_planar = false;
			is_ignore = false;
			local_ground_id = -1;
		}
	};

}

#endif