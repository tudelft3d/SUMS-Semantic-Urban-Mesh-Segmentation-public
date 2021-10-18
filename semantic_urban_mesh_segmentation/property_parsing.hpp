/*
*   Name        : property_parsing.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh property parsing and point cloud sampling
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
#ifndef semantic_mesh_segmentation__PROPERTY_PARSING_HPP
#define semantic_mesh_segmentation__PROPERTY_PARSING_HPP

#include <map>
#include <cmath>
#include <cfloat>
#include <algorithm>

#include <easy3d/surface_mesh_io.h>

#include "feature_computation.hpp"
#include "sampling_function.hpp"
#include "mesh_io.hpp"
#include "super_segment.hpp"

namespace semantic_mesh_segmentation
{
	//template funciton
	template<typename A, typename B>
	std::pair<B, A> flip_pair(const std::pair<A, B> &p)
	{
		return std::pair<B, A>(p.second, p.first);
	}

	template<typename A, typename B>
	std::multimap<B, A> flip_map(const std::map<A, B> &src)
	{
		std::multimap<B, A> dst;
		std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()),
			flip_pair<A, B>);
		return dst;
	}

	// --- declared functions ---
	void sampling_point_cloud_on_mesh(SFMesh*, std::vector<cv::Mat> &, PTCloud *, PTCloud *, PTCloud *, std::map<int, int> &, const int);

	void sampling_point_cloud_on_mesh(SFMesh*, std::vector<cv::Mat> &, PTCloud *, const int);

	void construct_superfacets
	(
		SFMesh *,
		PTCloud *,
		std::vector<superfacets>&,
		std::vector<std::vector<int> > &,
		std::vector<int> &
	);

	void input_mesh_configuration(SFMesh *);

	void add_mesh_properties_from_input
	(
		SFMesh *,
		const int,
		SFMesh *smesh_seg = nullptr,
		std::vector<cv::Mat> &texture_maps = std::vector<cv::Mat>(),
		const int batch_index = -1
	);

	void construct_feature_pointclouds
	(
		std::vector<std::vector<int>> &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		PTCloud*
	);

	void visualization_feature_on_mesh
	(
		SFMesh *smesh_in,
		std::vector<std::vector<int>> &,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		SFMesh *
	);

	void parsing_to_pcl(PTCloud *, Point_range&);

	void parsing_to_mesh(SFMesh *, Mesh&);

	std::vector<Cluster_point> convert_to_CGAL_Cluster(std::vector< std::vector<float>> &);

	void check_ignored_truth_labels();

	void ignore_truth_labels(std::vector<int> &);

}

#endif//MESH_SEGMENTATION__SEGMENT_FILE_HPP