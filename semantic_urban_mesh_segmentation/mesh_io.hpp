/*
*   Name        : mesh_io.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : input/output of meshes (*.ply), point clouds (*.ply), textures (*.jpg) and texts (*.txt).
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
#ifndef semantic_mesh_segmentation__MESH_IO_HPP
#define semantic_mesh_segmentation__MESH_IO_HPP

#include <filesystem>
#include <direct.h>
#include <io.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <omp.h>
#include <numeric>

#include <easy3d/surface_mesh.h>
#include <easy3d/kdtree.h>
#include <easy3d/point_cloud.h>
#include <easy3d/point_cloud_io.h>
#include <easy3d/surface_mesh_io.h>
#include <opencv2/opencv.hpp>

#include "SFMesh.hpp"
#include "property_parsing.hpp"
#include "super_segment.hpp"
#include "parameters.hpp"
#include "PTCloud.hpp"
#include "CGAL_Define.hpp"

namespace semantic_mesh_segmentation
{
	inline bool smallest_segment_id
	(
		superfacets spf_1,
		superfacets spf_2
	)
	{
		if (spf_1.id < spf_2.id)
			return true;
		else
			return false;
	}

	//---------------------- function for splitting string ----------------------//
	inline std::vector<std::string> Split(const std::string &s, const std::string &delim, const bool keep_empty = true)
	{
		std::vector<std::string> result;
		if (delim.empty()) {
			result.push_back(s);
			return result;
		}
		std::string::const_iterator substart = s.begin(), subend;
		while (true) {
			subend = search(substart, s.end(), delim.begin(), delim.end());
			std::string temp(substart, subend);
			if (keep_empty || !temp.empty()) {
				result.push_back(temp);
			}
			if (subend == s.end()) {
				break;
			}
			substart = subend + delim.size();
		}
		return result;
	}

	inline std::string get_file_based_name(const std::string &str)
	{
		char * file = (char *)str.data();
		char szDrive[_MAX_DRIVE], szDir[_MAX_DIR], szFname[_MAX_FNAME], szExt[_MAX_EXT];
		_splitpath(file, szDrive, szDir, szFname, szExt);

		//separate char
		char *delim = "_";
		std::string base_name = "";
		char *tmp = strtok(szFname, delim);
		while (tmp != NULL)
		{
			bool is_ignore = false;
			for (auto ig_str : ignored_str)
			{
				if (ig_str == tmp)
				{
					is_ignore = true;
					break;
				}
			}

			if (!is_ignore)
			{
				if (base_name != "")
					base_name += delim;
				base_name += tmp;
			}

			tmp = strtok(NULL, delim);
		}

		return base_name;
	}

	//-----------------read data-----------------//
	void getAllFiles(const std::string&, const std::string&, std::vector<std::string> &, std::vector<std::string> &);

	void rply_input(SFMesh*, char*);

	void read_pointcloud_data(SFMesh *, PTCloud*, const int, const int);

	void read_pointcloud_data(PTCloud*, const int, const int);

	easy3d::PointCloud* read_semantic_pointcloud_data(const int);

	void read_mesh_data(SFMesh *, const int, std::vector<cv::Mat> &texture_maps = std::vector<cv::Mat>(), const int batch_index = -1);

	void read_test_mesh_data(SFMesh *, const int);

	void read_graph_nodes_ply(std::vector<superfacets> &, const int);

	void read_seg_mesh_with_pnp_info(SFMesh*, const int);

	void read_labeled_mesh_data(SFMesh *, const int);

	void read_and_write_oversegmentation_ground_truth();

	void read_oversegmentation_testdata(SFMesh*, const int);

	void read_oversegmentation_truthdata(SFMesh*, const int);

	void read_txt_batches(std::vector<std::vector<std::pair<int, std::string>>> &);

	easy3d::PointCloud* read_feature_pointcloud_data(const std::string);

	void read_txt_config(const std::string &path);

	//-----------------write data-----------------//
	void rply_output(SFMesh*, const char*, const std::vector<std::string> &);

	void rply_output(SFMesh*, char*);

	void rply_output(PTCloud*, char*);

	void write_mesh_segments(SFMesh*, const int, bool write_pnp = false);

	void write_pointcloud_data(PTCloud*, const int, const int);

	void write_tex_pointcloud_data(PTCloud*, const int);

	void write_feature_mesh_data(SFMesh*, const int);

	void write_feature_pointcloud_data(PTCloud*, const std::string);

	void write_semantic_mesh_data(SFMesh*, const int);

	void write_error_mesh_data(SFMesh*, const int);

	void write_pnp_mesh_data(SFMesh*, const int);

	void save_txt_batches(std::vector<std::vector<std::pair<int, std::string>>> &);

	void save_txt_statistics(std::vector<float> &, std::vector<float> &);

	void save_txt_feature_importance(std::vector<std::string> &, std::vector<std::pair<int, int>> &, const int);

	void save_txt_mesh_areas(std::vector<std::pair<std::string, float>> &, std::vector<std::pair<std::string, std::vector<float>>> &);

	void save_txt_feature_divergence(std::vector<std::string> &, std::vector<float> &);

	void save_txt_evaluation(CGAL::Classification::Label_set &, CGAL::Classification::Evaluation &, std::ostringstream &, const int);

	void save_graph_nodes_ply(std::vector<superfacets> &, PTCloud *, const int);

	void save_graph_edges_ply
	(
		PTCloud*,
		std::vector<std::pair<int, int>> &,
		const int,
		std::vector<superfacets> spf_current = std::vector<superfacets>()
	);

	void save_txt_evaluation
	(
		std::vector<float> &,
		std::vector<float> &,
		std::vector<float> &,
		std::ostringstream &,
		const int 
	);

	void write_feature_pointcloud_data_for_GCN
	(
		PTCloud*,
		const int
	);

	void get_mesh_labels(SFMesh *, std::vector<int> &, std::vector< std::vector<int>> &);
}

#endif//MESH_SEGMENTATION__SEGMENT_FILE_HPP