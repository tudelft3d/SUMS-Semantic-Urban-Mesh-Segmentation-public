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
}

#endif//MESH_SEGMENTATION__SEGMENT_FILE_HPP
