/*
*   Name        : mesh_processor.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh processing pipeline
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
#ifndef semantic_mesh_segmentation__MESH_PROCESSOR_HPP
#define semantic_mesh_segmentation__MESH_PROCESSOR_HPP

#include <direct.h>
#include <omp.h>
#include <easy3d/point_cloud_io.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <Python.h>
#include "mesh_io.hpp"
#include "over_segmentation.hpp"
#include "feature_computation.hpp"
#include "property_parsing.hpp"
#include "parameters.hpp"
#include "mesh_classifier.hpp"

namespace semantic_mesh_segmentation
{
	void get_training_data();

	void get_testing_data();

	void get_predicting_data();

	void get_validation_data();

	bool check_sub_names
	(
		int &,
		std::vector<std::set<std::string>> &,
		std::string &
	);

	void update_base_names
	(
		int &,
		std::vector<std::set<std::string>> &,
		std::string &
	);

	void update_sub_names
	(
		int &,
		std::vector<std::set<std::string>> &,
		std::string &
	);

	void fill_pre_batch_or_activate_new
	(
		int &,
		std::string &ba_i,
		std::vector<std::set<std::string>> &,
		std::vector<std::set<std::string>> &,
		std::map<int, bool> &,
		const std::string,
		const std::string,
		const int,
		std::map<int, int> &
	);

	void square_batch_separation(std::vector<std::vector<std::pair<int, std::string>>> &);

	void conversion_from_ground_truth
	(
		SFMesh *,
		std::vector<float> &
	);

	void augmented_node_feauture_random
	(
		std::vector<float> &,
		std::vector<float> &,
		std::vector<float> &,
		std::vector<float> &
	);

	void joint_labels_feature_concatenation
	(
		std::vector<int> &,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::map<std::pair<int, int>, std::pair<float, bool>> &
	);

	void training_feature_process_batch_tiles
	(
		std::vector<int> &,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector<std::pair<int, std::string>> &,
		const int 
	);

	int separate_connected_components
	(
		SFMesh *,
		int &
	);

	void construct_superfacet_neighbors
	(
		SFMesh*,
		std::map<std::pair<int, int>, std::pair<float, bool>> & 
	);

	void training_feature_process_single_tiles
	(
		std::vector<int> &,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		const int 
	);

	void testing_feature_process_batch_tiles_with_model
	(
		Label_set &,
		Feature_set &,
		CGAL::Classification::ETHZ::Random_forest_classifier &,
		std::vector<std::pair<int, std::string>> &,
		const int 
	);

	void testing_feature_process_single_tiles_with_model
	(
		Label_set &,
		Feature_set &,
		CGAL::Classification::ETHZ::Random_forest_classifier &,
		const int
	);

	void testing_feature_process_batch_tiles_without_model
	(
		CGAL::Classification::ETHZ::Random_forest_classifier &,
		Feature_set &,
		std::vector<std::pair<int, std::string>> &,
		const int
	);

	void testing_feature_process_single_tiles_without_model
	(
		CGAL::Classification::ETHZ::Random_forest_classifier &,
		Feature_set &,
		const int
	);

	void parsing_semantics_from_color
	(
		SFMesh*,
		PointCloud*
	);

	void feature_diversity_process_batch_tiles
	(
		std::vector<float> &batches_feas_var,
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	);

	void feature_diversity_process_single_tiles
	(
		std::vector<float> &,
		const int
	);

	void nearst_neighbor_labels_assign
	(
		SFMesh*,
		PointCloud*,
		PTCloud *
	);

	void parsing_semantics_from_labelstring
	(
		SFMesh*,
		PointCloud*
	);

	vec3 hsv_to_rgb(const vec3& c);

	void error_map
	(
		SFMesh *,
		const int 
	);

	void merge_semantic_pointcloud
	(
		PointCloud*,
		PTCloud *,
		SFMesh *,
		int &
	);

	void semantic_pcl_process_batch_tiles
	(
		std::vector<std::pair<int, std::string>> &,
		const int
	);

	void semantic_pcl_process_single_tiles
	(
		const int
	);

	void processing_semantic_pcl_input
	(
		SFMesh*,
		PointCloud*,
		PTCloud* = nullptr
	);

	void compute_feature_diversity
	(
		std::vector<float> &
	);

	void compute_mesh_area
	(
		std::vector<std::pair<std::string, float>> &,
		std::vector<std::pair<std::string, std::vector<float>>> &
	);

	void smote_data_augmentation();
}
#endif