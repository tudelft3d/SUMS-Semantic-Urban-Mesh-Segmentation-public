/*
*   Name        : parameters.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : parameter definition and default parameter setting
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
#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <iostream>
#include <vector>
#include <map>
#include <easy3d/surface_mesh.h>

namespace semantic_mesh_segmentation
{
	//--- operating modes --- 
	enum struct operating_mode
	{
		Pipeline,
		Mesh_feature_extraction,
		Save_mesh_features_for_visualization,
		Feature_extraction_backbone,
		Visualization_features_backbone,
		Class_statistics,
		Generate_semantic_sampled_points,
		Train_config,
		Test_config,
		Train_Backbone,
		Test_Backbone,
		Train_and_Test_Backbone,
		Train_and_Test_config,
		Data_evaluation_for_all_tiles_config,
		Data_evaluation_for_all_tiles,
		Moha_Verdi_SOTA_pipeline,
		Evaluation_SOTA,
		Process_semantic_pcl,
		Feature_diversity_meaure,
		Compute_mesh_area,
		PSSNet_pipeline_for_GCN,
		Get_labels_for_planar_non_planar_from_semantic,
		PSSNet_oversegmentation,
		PSSNet_oversegmentation_backbone,
		PSSNet_oversegmentation_evaluation,
		PSSNet_graph_construction,
		PSSNet_graph_construction_backbone,
		PSSNet_pcl_generation_for_GCN,
		PSSNet_pcl_generation_for_GCN_backbone
	};

	//--- global variables for point cloud sampling ---
	extern easy3d::Box3 mesh_bounding_box;
	extern float mesh_all_area;
	extern float texture_pointcloud_density;
	extern float sampling_ratio;
	extern int sampling_points_number;
	extern int bestSamplePoolSize;
	extern float sampling_point_density;
	extern float ele_sampling_point_density;
	extern int processing_mode;
	extern std::string label_definition;
	extern bool use_binary;

	//--- Medial axis shrinking ball parameters ---
	extern float mat_delta_convergance;
	extern float mat_initialized_radius;
	extern float mat_denoising_seperation_angle;
	extern int mat_iteration_limit_number;
	extern int input_type_for_statistics;

	//--- global variable for features ---
	extern std::pair<float, float> default_feature_value_minmax;
	extern std::vector<float> multi_scale_ele_radius;
	extern float long_range_radius_default, short_range_radius_default;
	extern int local_ground_segs;
	extern std::vector<int> hsv_bins;

	//cgal region growing parameters setting
	extern float adjacent_radius;
	extern float adjacent_pt2plane_distance;
	extern float adjacent_seg_angle;
	extern float mesh_distance_to_plane;
	extern float mesh_accepted_angle;
	extern float mesh_minimum_region_size;;
	extern float pcl_distance_to_plane;
	extern float pcl_accepted_angle;
	extern float pcl_minimum_region_size;
	extern int pcl_k_nn;

	//--- cutoff value ---
	extern int cutoff_spf_vertex_count;
	extern float cutoff_spfarea_max;
	extern float cutoff_spffacetdensity_max;
	extern float relative_elevation_cut_off_max;
	extern float eigen_feature_cut_off;
	extern float mat_radius_cut_off;
	extern float training_mesh_area;
	extern float test_mesh_area;
	//--- global variable for superfacet id-index map ---
	extern std::map<int, int> superfacet_id_index_map;

	//--- default ---
	extern operating_mode current_mode, previous_mode;

	//--- sampling parameters ---
	extern int sampling_strategy;
	extern int sampling_strategy_training;
	extern int sampling_strategy_testing;
	extern int sampling_strategy_predicting;
	extern int sampling_strategy_validation;

	//--- bool parameters ---
	extern std::map<std::string, bool> process_data_selection;
	extern bool save_sampled_pointclouds;
	extern bool save_oversegmentation_mesh;
	extern bool is_pointclouds_exist;
	extern bool use_existing_splited_batch;
	extern bool use_merged_segments;
	extern bool use_merged_segments_on_training; //control to use region growing or (other mesh segments or facets)
	extern bool use_merged_segments_on_testing;
	extern bool use_merged_segments_on_predicting;
	extern bool use_merged_segments_on_validation;

	extern bool use_batch_processing;
	extern bool use_batch_processing_on_training;
	extern bool use_batch_processing_on_testing;
	extern bool use_batch_processing_on_predicting;
	extern bool use_batch_processing_on_validation;

	extern bool use_pointcloud_region_growing;
	extern bool use_pointcloud_region_growing_on_training;
	extern bool use_pointcloud_region_growing_on_testing;
	extern bool use_pointcloud_region_growing_on_predicting;
	extern bool use_pointcloud_region_growing_on_validation;

	extern bool use_existing_mesh_segments;
	extern bool use_existing_mesh_segments_on_training;
	extern bool use_existing_mesh_segments_on_testing;
	extern bool use_existing_mesh_segments_on_predicting;
	extern bool use_existing_mesh_segments_on_validation;
	extern bool save_feature_importance;

	extern bool use_face_pixels_color_aggregation;
	extern bool use_face_pixels_color_aggregation_on_training;
	extern bool use_face_pixels_color_aggregation_on_testing;
	extern bool use_face_pixels_color_aggregation_on_predicting;
	extern bool use_face_pixels_color_aggregation_on_validation;

	extern bool with_texture;
	extern bool save_tex_cloud, save_textures_in_predict;
	extern bool add_point_color;
	extern bool add_point_color_for_dp_input;
	extern bool save_error_map;

	//--- Data augmentation ---
	extern bool augmented_data_exist;
	extern bool enable_augment;
	extern int used_k_neighbors;

	//--- Classification parameters ---
	extern int rf_tree_numbers;
	extern int rf_tree_depth;

	//--- Selected features ---//
	extern std::map<int, bool> use_feas;
	extern std::map<int, bool> use_mulsc_eles;
	extern std::map<int, bool> use_basic_features;
	extern std::map<int, bool> use_eigen_features;
	extern std::map<int, bool> use_color_features;

	//--- folder path ---
	extern std::string root_path, data_path, training_data_path, testing_data_path, predicting_data_path, validation_data_path, partition_folder_path;
	extern std::vector<std::string> ignored_str;
	extern std::vector<std::string> batch_ignored_str;
	extern std::string batch_delim_separate_char;
	extern std::string sota_folder_path, partition_folder_path, sota_prefixs, partition_prefixs;
	extern std::string seg_aug_py_path;

	//--- processing semantic point cloud ---
	extern std::string label_string;
	extern int label_minus;
	extern bool equal_cloud;

	//--- input file names ---
	extern std::vector<std::string> folder_names_level_0, folder_names_level_1, folder_names_pssnet;
	extern std::vector<std::string>
		base_names, ply_files, file_folders, txt_files, file_formats, prefixs, ply_comment_element,
		training_base_names, testing_base_names, predicting_base_names, validation_base_names,
		training_ply_files, testing_ply_files, predicting_ply_files, validation_ply_files,
		training_file_folders, testing_file_folders, predicting_file_folders, validation_file_folders;
	extern std::vector<std::string> data_types;
	extern std::map<std::string, int> file_ind_map, training_file_ind_map, testing_file_ind_map, predicting_file_ind_map, validation_file_ind_map;

	//--- level of details --			
	extern int train_test_predict_val;
	extern int batch_size;
	extern int sub_batch_size;

	//--- label names ---
	extern std::vector<std::string> labels_name, ignored_labels_name, labels_name_pnp;
	extern std::vector<easy3d::vec3> labels_color, labels_color_pnp;
	typedef typename int label_type;//other: int; PointNet, PointNet2, ConvPoints: float
	extern std::map<int, int> label_shiftdis, new_label_shiftback;
	extern std::map<int, bool> label_ignore;
	extern std::map<std::string, int> L1_to_L0_label_map;

	//--- feature names in files ---//
	extern std::vector<std::pair<std::string, int>> selected_pcl_vertex_features;
	extern std::vector<std::pair<std::string, int>> selected_mesh_face_features;
	extern std::vector<std::pair<std::string, int>> basic_feature_base_names;
	extern std::vector<std::pair<std::string, int>> eigen_feature_base_names;
	extern std::vector<std::pair<std::string, int>> color_feature_base_names;

	//--- Selected features ---//
	extern std::map<int, bool> use_feas;
	extern std::map<int, bool> use_mulsc_eles;
	extern std::map<int, bool> use_basic_features;
	extern std::map<int, bool> use_eigen_feas;
	extern std::map<int, bool> use_color_feas;

	//--- Mohammad Rouhani and Yannick Verdie region growing setting ---
	extern float mr_facet_neg_sphericial_radius;
	extern float mr_limit_distance;
	extern float mr_angle_thres;
	extern float mr_L1_color_dis;
	extern float mr_max_sp_area;

	extern float mrf_lambda_mh;
	extern float mrf_energy_amplify;

	extern bool enable_MohaVer_region_growing;
	extern bool enable_joint_labeling;

	//****************** PSSNet parameters ****************
	extern bool generate_groundtruth_segments;
	extern bool only_evaluation;
	extern bool use_GCN_features;
	extern bool only_write_GCN_features;

	//over-segmentation
	extern float radius_default, mrf_lambda_d, mrf_lambda_m, mrf_lambda_g, mrf_lambda_p;

	//graph
	extern bool with_node_graphs, use_edges_between_boder_points, parallelism_relations;

	extern float tolerance_angle;

	extern bool local_ground_relations;

	extern bool exterior_mat_relations;

	extern bool delaunay_relations_on_sampled_points;
	extern double remove_close_vertices_for_delaunay_dis;

	extern int cutoff_spf_vertex_count, alpha_shape_val, border_growing_neighbor;
	extern float cutoff_spfcompact_max;
	extern std::pair<bool, int> ignore_mesh_boundary;
}

#endif // 


