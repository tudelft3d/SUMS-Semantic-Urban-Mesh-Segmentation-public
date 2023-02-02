/*
*   Name        : parameters.cpp
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

#include "parameters.hpp"
#include <easy3d/constant.h>
#include <tuple>

namespace semantic_mesh_segmentation
{
	//****************** Common parameters ******************
	std::string root_path = "C:/data/git_sum/data/";
	std::string seg_aug_py_path = "../../Semantic-segmentation-of-large-scale-urban-meshes/";

	int processing_mode = 0;//0: RF(SUM Paper); 1: SOTA (Competition methods); 2: PSSNet
	std::string label_definition = "label";
	bool use_binary = true;

	std::map<std::string, bool> process_data_selection
	{
		{"train", true},
		{"test", true},
		{"predict", false},
		{"validate", true}
	};

	//in ply data: -1: unlabelled; 0: unclassified; 1: ground, 2: .....
	//in program: -2: unlabelled; -1: unclassifiedl; 0: ground, 1: ..... 
	std::vector<std::string> labels_name
	{
		"ground",             //0
		"vegetation",         //1
		"building",            //2
		"water",               //3
		"car",                 //4
		"boat"                 //5
	};

	std::vector<easy3d::vec3> labels_color
	{
		easy3d::vec3(float(170.0f / 255.0f), float(85.0f / 255.0f), 0.0f),//0
		easy3d::vec3(0.0f, 1.0f, 0.0f), //1
		easy3d::vec3(1.0f, 1.0f, 0.0f), //2
		easy3d::vec3(0.0f, 1.0f, 1.0f), //3
		easy3d::vec3(1.0f, 0.0f, 1.0f), //4
		easy3d::vec3(0.0f, 0.0f, float(153.0f / 255.0f)) //5
	};

	std::vector<std::string> ignored_labels_name{};

	//label names
	std::vector<std::string> labels_name_pnp
	{
		//Level 0
		"non_planar",
		"planar"

		//H3D
		//"non_planar",//label 7 Tree
		//"planar_1",  //label 0 Low Vegetation 
		//"planar_2",  //label 8 Soil/Gravel 
		//"planar_3"   //label 1 Impervious Surface, label 2 Vehicle, label 3 Urban Furniture, label 4 Roof, label 5 Facade, label 6 Shrub, label 9 Vertical Surface, label 10 Chimney 
	};

	//0 for non-planar, 1 for planar type 1, 2 for planar type 2, ...
	std::map<std::string, int> L1_to_L0_label_map
	{
		//L1, SUM
		{"ground", 1},       //0
		{"vegetation", 0},   //1
		{"building", 1},     //2
		{"water", 1},        //3
		{"car", 1},          //4
		{"boat", 1}          //5

		//Level 1, merged H3D
		//{"Low_Vegetation", 0},         //0
		//{"Impervious_Surface", 0},     //1
		//{"Vehicle", 3},                //2
		//{"Urban_Furniture", -1},       //3
		//{"Roof", 2},                   //4
		//{"Facade", 2},                 //5
		//{"Shrub", 1},                  //6
		//{"Tree", 1},                   //7
		//{"Soil_Gravel", 0},            //8
		//{"Vertical_Surface", -1},      //9
		//{"Chimney", 2}                 //10

		//Level 1, H3D, C11 planar and non planar
		//{"Low_Vegetation", 1},         //0
		//{"Impervious_Surface", 3},     //1
		//{"Vehicle", 3},                //2
		//{"Urban_Furniture", 3},        //3
		//{"Roof", 3},                   //4
		//{"Facade", 3},                 //5
		//{"Shrub", 3},                  //6
		//{"Tree", 0},                   //7
		//{"Soil_Gravel", 2},            //8
		//{"Vertical_Surface", 3},       //9
		//{"Chimney", 3}                 //10
	};

	std::vector<easy3d::vec3> labels_color_pnp
	{
		//L0, SUM
		easy3d::vec3(1.0, 0.4, 0.7),//0
		easy3d::vec3(0.5, 0.5, 0.5) //1

		//Merged SUM
		//easy3d::vec3(float(170.0f / 255.0f), float(85.0f / 255.0f), 0.0f),//0
		//easy3d::vec3(0.0f, 1.0f, 0.0f), //1
		//easy3d::vec3(1.0f, 1.0f, 0.0f), //2
		//easy3d::vec3(1.0f, 0.0f, 1.0f), //4

		//H3D
		//easy3d::vec3(1.0, 0.4, 0.7), //0
		//easy3d::vec3(0.2, 0.2, 0.2), //1
		//easy3d::vec3(0.5, 0.5, 0.5), //2
		//easy3d::vec3(0.8, 0.8, 0.8)  //3

		//S3DIS
		//easy3d::vec3(1.0, 0.4, 0.7), //0
		//easy3d::vec3(0.2, 0.2, 0.2), //1
		//easy3d::vec3(0.8, 0.8, 0.8)  //2
	};

	//****************** Mesh_feature_extraction ******************
	bool with_texture = true; // depends on the read data
	bool is_pointclouds_exist = false;

	//save intermediate data
	bool save_sampled_pointclouds = false;
	bool save_oversegmentation_mesh = true;
	bool save_tex_cloud = false;
	bool save_textures_in_predict = false;

	//settings for feature computation
	std::vector<float> multi_scale_ele_radius = { 10.0f, 20.0f, 40.0f };//From Mohammad Rouhani's paper
	std::vector<int> hsv_bins = { 15, 5, 5 };

	//settings for feature computation: Medial axis shrinking ball parameters
	float mat_delta_convergance = 1E-5f;
	float mat_initialized_radius = 200.0f;
	float mat_denoising_seperation_angle = 60 * (M_PI / 180);
	int mat_iteration_limit_number = 30;//30, 9999

	//settings for feature computation: relative elevation from local ground extraction 
	float long_range_radius_default = 30.0f;//30.0f;//100.0f
	int local_ground_segs = 5;// 5;

	//settings for sampled point clouds
	float sampling_point_density = 0.5f; //0.1f, 0.25f (faster, lower accuracy), 0.5f/3.0f (2 times slower, higher accuracy)
	float ele_sampling_point_density = 0.1f; //0.1f (faster, lower accuracy), 0.5f (2 times slower, higher accuracy)

	//settings for over-segmentation
	bool use_existing_mesh_segments_on_training = false; //control to use region growing or (other mesh segments or facets)
	bool use_existing_mesh_segments_on_testing = false;
	bool use_existing_mesh_segments_on_predicting = false;
	bool use_existing_mesh_segments_on_validation = false;

	//setting for merge segments
	bool use_merged_segments_on_training = false; //control to use region growing or (other mesh segments or facets)
	bool use_merged_segments_on_testing = false;
	bool use_merged_segments_on_predicting = false;
	bool use_merged_segments_on_validation = false;
	float adjacent_radius = 0.5f;
	float adjacent_pt2plane_distance = 0.5f;
	float adjacent_seg_angle = 20.0f;

	//settings for texture color processing
	bool use_face_pixels_color_aggregation_on_training = false; //control to use region growing or (other mesh segments or facets)
	bool use_face_pixels_color_aggregation_on_testing = false;
	bool use_face_pixels_color_aggregation_on_predicting = false;
	bool use_face_pixels_color_aggregation_on_validation = false;

	//settings for over-segmentation: region growing on mesh
	float mesh_distance_to_plane = 0.5f; //RFSPG: 0.5
	float mesh_accepted_angle = 90.0f; //RFSPG: 90
	float mesh_minimum_region_size = 0.0f;//1

	//****************** Train_and_Test parameters ******************
	int rf_tree_numbers = 200;// 200; MOHR: 100;  Note too many trees may causes boost serialization errors!!! 
	int rf_tree_depth = 50;// 50; MOHR: 25

	//smote data augmentation
	bool augmented_data_exist = false;
	bool enable_augment = false;
	int used_k_neighbors = 15;

	//save data
	bool save_error_map = true;
	bool save_feature_importance = false;

	//****************** Class_statistics ******************
	int input_type_for_statistics = 0;//0: mesh; 1: sampled point cloud

	//****************** SOTA: Moha_Verdi_SOTA_pipeline ******************
	//over-segmentation parameters
	float mr_facet_neg_sphericial_radius = 1.8288f; //2 yards
	float mr_limit_distance = FLT_MAX; //0.5 yards
	float mr_angle_thres = 20.0f; //degree
	float mr_L1_color_dis = 90;   //over 256*3, RGB
	float mr_max_sp_area = 100.0; //100.0f;
	float short_range_radius_default = 3.0f;//for initial vertex planarity

	//graph-cut parameters
	float mrf_lambda_mh = 0.5f;
	float mrf_energy_amplify = 10E1;

	//keep defaults, will be automatic activated in the operation mode
	bool enable_MohaVer_region_growing = false;
	bool enable_joint_labeling = false;

	//****************** SOTA : all deep learning methods ******************
	bool add_point_color_for_dp_input = true; //if not then computation fast
	//"spg/"             | "kpconv/"    | "randlanet/"      | "pointnet2/" | "pointnet/"
	//"_pcl_gcn_pred_up" | "_pcl_sampled" | "_pcl_sampled_pred" | "_pcl_sampled" | "_pcl_gcn"
	std::string sota_folder_path = "spg/", sota_prefixs = "_pcl_gcn_pred_up";
	//"moha_rg/"
	std::string partition_folder_path = "segments/", partition_prefixs = "_mesh_seg";

	//SPG:"pred"; KPConv: "preds"; RandLanet: "label"; PointNet, PointNet2: "pred"
	std::string label_string = "pred";
	//SPG: 1; KPConv: 0; RandLanet: 1;  PointNet2: 0 ; PointNet: 0
	int label_minus = 1; //0: no minus, if label start from 0; 1: minus 1, if label start from 1

	bool equal_cloud = true; //others: true; Randlanet: false;

	//labels type: change label_type in parameters.hpp

	//****************** Evaluation ******************
	bool use_area_weight = true;

	//****************** PSSNet parameters ****************
	bool generate_groundtruth_segments = false;
	bool only_evaluation = false; //for test and validation data only
	bool use_GCN_features = true;
	bool only_write_GCN_features = false;

	//over-segmentation
	float radius_default = 3.0f;//initial vertex planarity, 0.5f
	float mrf_lambda_d = 1.2f,//default:1.2f. >1.0: increase weight of distance to plane; <1.0: decrease weight of distance to plane
		mrf_lambda_m = 0.05,//default: 0.05f, planar region growing angle weight, 
		mrf_lambda_g = 0.9f,//default:0.9f 
		mrf_lambda_p = 0.6f;//default:0.6f, post global smoothness

	//graph
	bool with_node_graphs = false;// true
	bool use_edges_between_boder_points = true;

	bool parallelism_relations = true;//true
	float tolerance_angle = 5.0f;//default: 5.0

	bool local_ground_relations = true;//true

	bool exterior_mat_relations = true;//true

	bool delaunay_relations_on_sampled_points = true;
	double remove_close_vertices_for_delaunay_dis = 10E-3 * 2.0f;

	//feature parameters
	int alpha_shape_val = 1;//1
	int border_growing_neighbor = 10;//>0, 5 for non batch; 20 for batch
	float cutoff_spfcompact_max = 1.0f;
	std::pair<bool, int> ignore_mesh_boundary(true, 1.0f); //is ignore, boundary search radius 
	//****************** Default parameters ******************
	//initialized values
	std::pair<float, float> default_feature_value_minmax(0.000001f, 0.999999f);
	easy3d::Box3 mesh_bounding_box = easy3d::Box3();
	float mesh_all_area = 0.0f;
	float sampling_ratio = 20.0f; //Point cloud sampling ratio to total number of vertex
	int sampling_points_number = 1000;//smallest sampling number, -1 for use the number of vertices
	int bestSamplePoolSize = 10;
	float texture_pointcloud_density = 10.0E3;//10.0E3 is full resolution
	float training_mesh_area = 0.0f;
	float test_mesh_area = 0.0f;
	int train_test_predict_val = -1;
	int sampling_strategy = -1;

	//default bool switcher
	bool use_batch_processing = false;  // depends on the read data
	bool add_point_color = false; //if not then computation fast
	bool use_existing_mesh_segments = false;//if not use over-segmentation, then read the existing segments from the input
	bool use_pointcloud_region_growing = false;
	bool use_face_pixels_color_aggregation = false;
	bool use_merged_segments = false;

	//feature cutoff values
	int cutoff_spf_vertex_count = 10000;
	float cutoff_spfarea_max = 100.0f;
	float cutoff_spffacetdensity_max = 1.0f;
	float relative_elevation_cut_off_max = 35.0f;
	float eigen_feature_cut_off = 0.999f;
	float mat_radius_cut_off = 500.0f;

	//check ignored labels
	std::map<int, int> label_shiftdis = {}, new_label_shiftback = {};
	std::map<int, bool> label_ignore = {};

	//feature selection, same as SUM paper
	std::map<int, bool> use_feas
	{
		{0, true}, //segment_basic_features
		{1, true}, //segment_eigen_features
		{2, true}, //segment_color_features
		{3, true} //elevation_features
	};

	std::map<int, bool> use_mulsc_eles // for elevations
	{
		{0, true},//10.0f
		{1, true},//20.0f
		{2, true} //40.0f
	};

	//individual feature selection
	std::map<int, bool> use_basic_features
	{
		{0, true},  //avg_center_z
		{1, true},  //interior_mat_radius
		{2, true},  //sum_area
		{3, true},  //relative_elevation
		{4, true},  //triangle density (only useful when input is adaptive mesh)
		{5, true},  //vertex_count
		{6, true},  //points_to_plane_dist_mean
		{7, true},  //compactness
		{8, true},  //shape_index
		{9, true}   //shape_descriptor
	};

	std::map<int, bool> use_eigen_features
	{
		{0, false},  //eigen_1
		{1, false},  //eigen_2
		{2, false},  //eigen_3
		{3, true},   //verticality
		{4, true},   //linearity
		{5, false},  //planarity
		{6, true},   //sphericity
		{7, false},  //anisotropy
		{8, false},  //eigenentropy
		{9, false},  //omnivariance
		{10, false}, //sumeigenvals
		{11, true},  //curvature
		{12, false}, //verticality_eig1
		{13, false}, //verticality_eig3
		{14, false}, //surface
		{15, false}, //volume
		{16, false}, //absolute_eigvec_1_moment_1st
		{17, false}, //absolute_eigvec_2_moment_1st
		{18, false}, //absolute_eigvec_3_moment_1st
		{19, false}, //absolute_eigvec_1_moment_2nd
		{20, false}, //absolute_eigvec_2_moment_2nd
		{21, false}, //absolute_eigvec_3_moment_2nd
		{22, false}, //vertical_moment_1st
		{23, false}, //vertical_moment_2nd
		{24, false}  //uniformity
	};

	std::map<int, bool> use_color_features
	{
		{0, false},//red
		{1, false},//green
		{2, false},//blue
		{3, true},//hue
		{4, true},//sat
		{5, true},//val
		{6, true},//greenness
		{7, false},//red_var
		{8, false},//green_var
		{9, false},//blue_var
		{10, true},//hue_var
		{11, true},//sat_var
		{12, true},//val_var
		{13, false},//greenness_var
		{14, true},//hue_bin_0
		{15, true},//hue_bin_1
		{16, true},//hue_bin_2
		{17, true},//hue_bin_3
		{18, true},//hue_bin_4
		{19, true},//hue_bin_5
		{20, true},//hue_bin_6
		{21, true},//hue_bin_7
		{22, true},//hue_bin_8
		{23, true},//hue_bin_9
		{24, true},//hue_bin_10
		{25, true},//hue_bin_11
		{26, true},//hue_bin_12
		{27, true},//hue_bin_13
		{28, true},//hue_bin_14
		{29, true},//sat_bin_0
		{30, true},//sat_bin_1
		{31, true},//sat_bin_2
		{32, true},//sat_bin_3
		{33, true},//sat_bin_4
		{34, true},//val_bin_0
		{35, true},//val_bin_1
		{36, true},//val_bin_2
		{37, true},//val_bin_3
		{38, true} //val_bin_4
	};

	//****************** Batch processing parameters ******************
	bool use_existing_splited_batch = false;

	bool use_batch_processing_on_training = false;
	bool use_batch_processing_on_testing = false;
	bool use_batch_processing_on_predicting = false;
	bool use_batch_processing_on_validation = false;

	bool use_pointcloud_region_growing_on_training = false;
	bool use_pointcloud_region_growing_on_testing = false;
	bool use_pointcloud_region_growing_on_predicting = false;
	bool use_pointcloud_region_growing_on_validation = false;

	int batch_size = 5;//5, batch length: ****_
	int sub_batch_size = 5;//5, batch width _****

	//-1: no sampling; 0: sampled only; 1: face center only; 2: face center plus face vertices; 
	//3: sampled and face center; 4: sampled and face center plus face vertices.
	int sampling_strategy_training = false;
	int sampling_strategy_testing = false;
	int sampling_strategy_predicting = false;
	int sampling_strategy_validation = false;

	//settings for over-segmentation: region growing on point clouds
	float pcl_distance_to_plane = 2.0f; //2.0f, 0.75f
	float pcl_accepted_angle = 45; //45
	float pcl_minimum_region_size = 0;
	int pcl_k_nn = 100;//100

	//****************** Default definition ******************
	std::vector<std::string>
		base_names, ply_files, file_folders, txt_files,
		training_base_names, testing_base_names, predicting_base_names, validation_base_names,
		training_ply_files, testing_ply_files, predicting_ply_files, validation_ply_files,
		training_file_folders, testing_file_folders, predicting_file_folders, validation_file_folders;
	std::map<std::string, int> file_ind_map, training_file_ind_map, testing_file_ind_map, predicting_file_ind_map, validation_file_ind_map;

	operating_mode current_mode = operating_mode::Pipeline, previous_mode;
	std::string data_path, training_data_path, testing_data_path, predicting_data_path, validation_data_path;

	std::map<int, int> superfacet_id_index_map = std::map<int, int>();

	//file or format names
	std::vector<std::string> ignored_str = { "classification", "mesh", "groundtruth", "L0", "L1", "L2", "veg", "car" };
	std::vector<std::string> batch_ignored_str = { "Tile" };
	std::string batch_delim_separate_char = "_+";//"_" for Den Haag; "_+" for Helsinki

	//****************** files / folders / format / feature names ******************
	std::vector<std::string> file_formats
	{
		"ply",
		"jpg"
	};

	std::vector<std::string> folder_names_level_0
	{
		"evaluation/",//0
		"feature/",//1
		"input/",//2
		"model/",//3
		"output/",//4
		"pointcloud/",//5
		"segments/",//6
		"visualization/",//7
		"sota/",//8
		"sampled_pointcloud/", //9
		"semantic_pointcloud/", //10
		"PSSNet/" //11
	};

	std::vector<std::string> folder_names_level_1
	{
		"train/", //0
		"test/",  //1
		"predict/", //2
		"validate/", //3
		"train_augmented/"  //4
	};

	std::vector<std::string> folder_names_pssnet
	{
		"spg_input/",  //0
		"segments_pnp/",//1
		"graph_edges/", //2
		"graph_nodes/", //3
		"pcl/",         //4
		"segments_truth/", //5
		"feature_pnp/",    //6
		"visualization_pnp/", //7
		"spg_output/" //8
	};

	std::vector<std::string> prefixs
	{
		"_pcl_",//0
		"_error",//1
		"sampled",//2
		"feas",//3
		"_mesh_",//4
		"classification",//5
		"_evaluation",//6
		"seg",//7
		"truth",//8
		"batch_",//9
		"ele",//10
		"tex",//11
		"_class_statistics",//12
		"feature_bank_importance",//13
		"_aug",//14
		"_groundtruth_L0", //15
		"_graph",//16
		"gcn" //17
	};

	std::vector<std::string> data_types
	{
		"train",
		"test",
		"predict",
		"validate"
	};

	std::vector<std::string> ply_comment_element
	{
		"TextureFile",
		"label",
		"unclassified", //labeled as unclassified
		"unlabeled"
	};

	std::vector<std::pair<std::string, int>> selected_pcl_vertex_features
	{
		{"v:segment_basic_features", 0},
		{"v:segment_eigen_features", 1},
		{"v:segment_color_features", 2},
		{"v:multiscale_elevation_features", 3} //relative to local minimum elevation of points
	};

	std::vector<std::pair<std::string, int>> selected_mesh_face_features
	{
		{"f:segment_basic_features_", 0},
		{"f:segment_eigen_features_", 1},
		{"f:segment_color_features_", 2},
		{"f:multiscale_elevation_features_", 3} //relative to local minimum elevation of points
	};

	std::vector<std::pair<std::string, int>> basic_feature_base_names
	{
		{"avg_center_z", 0},
		{"interior_mat_radius", 1},
		{"sum_area", 2},
		{"relative_elevation", 3}, //relative to local ground which defined as local largest lower segment
		{"triangle_density", 4},
		{"vertex_count", 5},
		{"points_to_plane_dist_mean", 6},
		{"compactness", 7},
		{"shape_index", 8},
		{"shape_descriptor", 9}
	};

	std::vector<std::pair<std::string, int>> eigen_feature_base_names
	{
		{"eigen_1", 0},
		{"eigen_2", 1},
		{"eigen_3", 2},
		{"verticality", 3},
		{"linearity", 4},
		{"planarity", 5},
		{"sphericity", 6},
		{"anisotropy", 7},
		{"eigenentropy", 8},
		{"omnivariance", 9},
		{"sumeigenvals", 10},
		{"curvature", 11},
		{"verticality_eig1", 12},
		{"verticality_eig3", 13},
		{"surface", 14},
		{"volume", 15},
		{"absolute_eigvec_1_moment_1st", 16},
		{"absolute_eigvec_2_moment_1st", 17},
		{"absolute_eigvec_3_moment_1st", 18},
		{"absolute_eigvec_1_moment_2nd", 19},
		{"absolute_eigvec_2_moment_2nd", 20},
		{"absolute_eigvec_3_moment_2nd", 21},
		{"vertical_moment_1st", 22},
		{"vertical_moment_2nd", 23},
		{"uniformity", 24}
	};

	std::vector<std::pair<std::string, int>> color_feature_base_names
	{
		{"red", 0},
		{"green", 1},
		{"blue", 2},
		{"hue", 3},
		{"sat", 4},
		{"val", 5},
		{"greenness", 6},
		{"red_var", 7},
		{"green_var", 8},
		{"blue_var", 9},
		{"hue_var", 10},
		{"sat_var", 11},
		{"val_var", 12},
		{"greenness_var", 13},
		{"hue_bin_0", 14},
		{"hue_bin_1", 15},
		{"hue_bin_2", 16},
		{"hue_bin_3", 17},
		{"hue_bin_4", 18},
		{"hue_bin_5", 19},
		{"hue_bin_6", 20},
		{"hue_bin_7", 21},
		{"hue_bin_8", 22},
		{"hue_bin_9", 23},
		{"hue_bin_10", 24},
		{"hue_bin_11", 25},
		{"hue_bin_12", 26},
		{"hue_bin_13", 27},
		{"hue_bin_14", 28},
		{"sat_bin_0", 29},
		{"sat_bin_1", 30},
		{"sat_bin_2", 31},
		{"sat_bin_3", 32},
		{"sat_bin_4", 33},
		{"val_bin_0", 34},
		{"val_bin_1", 35},
		{"val_bin_2", 36},
		{"val_bin_3", 37},
		{"val_bin_4", 38}
	};
}
