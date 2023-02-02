/*
*   Name        : operation_function.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for run different operation modes
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

#include "operation_function.hpp"
namespace semantic_mesh_segmentation
{
	void run(const operating_mode& main_mode)
    {
        switch (main_mode)
        {
			//--- Mesh feature extraction ---
		case operating_mode::Pipeline:
		{
			current_mode = operating_mode::Pipeline;
			run(operating_mode::Mesh_feature_extraction);
			run(operating_mode::Train_and_Test_config);
			run(operating_mode::Data_evaluation_for_all_tiles_config);
			break;
		}

		//--- Mesh feature extraction ---
		case operating_mode::Mesh_feature_extraction:
		{
			current_mode = operating_mode::Mesh_feature_extraction;

			if (process_data_selection["train"])
			{
				std::cout << "--------------------- Generating train mesh features ---------------------" << std::endl;
				train_test_predict_val = 0;
				get_training_data();

				data_path = training_data_path;
				base_names = training_base_names;
				ply_files = training_ply_files;
				file_folders = training_file_folders;
				file_ind_map = training_file_ind_map;
				use_batch_processing = use_batch_processing_on_training;
				use_existing_mesh_segments = use_existing_mesh_segments_on_training;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_training;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_training;
				use_merged_segments = use_merged_segments_on_training;
				sampling_strategy = sampling_strategy_training;
				run(operating_mode::Feature_extraction_backbone);
			}

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Generating test mesh features ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();

				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;
				use_existing_mesh_segments = use_existing_mesh_segments_on_testing;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_testing;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_testing;
				use_merged_segments = use_merged_segments_on_testing;
				sampling_strategy = sampling_strategy_testing;
				run(operating_mode::Feature_extraction_backbone);
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Generating predict mesh features ---------------------" << std::endl;
				train_test_predict_val = 2;
				get_predicting_data();

				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				file_ind_map = predicting_file_ind_map;
				use_batch_processing = use_batch_processing_on_predicting;
				use_existing_mesh_segments = use_existing_mesh_segments_on_predicting;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_predicting;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_predicting;
				use_merged_segments = use_merged_segments_on_predicting;
				sampling_strategy = sampling_strategy_predicting;
				run(operating_mode::Feature_extraction_backbone);
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- Generating validate mesh features ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();

				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				use_existing_mesh_segments = use_existing_mesh_segments_on_validation;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_validation;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_validation;
				use_merged_segments = use_merged_segments_on_validation;
				sampling_strategy = sampling_strategy_validation;
				run(operating_mode::Feature_extraction_backbone);
			}

			break;
		}

		//--- Triangle is the smallest unit of superfacets ---
		case operating_mode::Feature_extraction_backbone:
		{
			current_mode = operating_mode::Feature_extraction_backbone;
			if (processing_mode == 2)
			{
				labels_name = labels_name_pnp;
				labels_color = labels_color_pnp;
			}

			if (use_batch_processing)
			{
				//Batch separation
				std::vector<std::vector<std::pair<int, std::string>>> all_batches;
				if (use_existing_splited_batch)
					read_txt_batches(all_batches);
				else
					square_batch_separation(all_batches);

				//Batch processing
				for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
				{
					std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
					process_batch_tiles(all_batches[bi], bi);
				}
			}
			else
			{
				//process single tile
				for (std::size_t mi = 0; mi < base_names.size(); ++mi)
					process_single_tile(mi);
			}
			break;
		}

//----------------------------------------------------------------------------------------------------------------------------------------------------

		case operating_mode::Train_config:
		{
			current_mode = operating_mode::Train_config;
			std::cout << "--------------------- Training Mode ---------------------" << std::endl;
			train_test_predict_val = 0;
			get_training_data();
			data_path = training_data_path;
			base_names = training_base_names;
			ply_files = training_ply_files;
			file_folders = training_file_folders;
			file_ind_map = training_file_ind_map;
			use_batch_processing = use_batch_processing_on_training;
			run(operating_mode::Train_Backbone);
			break;
		}

		case operating_mode::Test_config:
		{
			current_mode = operating_mode::Test_config;

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Testing Mode ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();
				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;
				run(operating_mode::Test_Backbone);
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Predicting Mode ---------------------" << std::endl;
				train_test_predict_val = 2;
				get_predicting_data();
				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				file_ind_map = predicting_file_ind_map;
				use_batch_processing = use_batch_processing_on_predicting;
				run(operating_mode::Test_Backbone);
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- Validation Mode ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();
				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				run(operating_mode::Test_Backbone);
			}

			break;
		}

		case operating_mode::Train_and_Test_config:
		{
			current_mode = operating_mode::Train_and_Test_config;

			train_test_predict_val = 0;
			get_training_data();
			data_path = training_data_path;
			base_names = training_base_names;
			ply_files = training_ply_files;
			file_folders = training_file_folders;
			file_ind_map = training_file_ind_map;
			use_batch_processing = use_batch_processing_on_training;
			run(operating_mode::Train_and_Test_Backbone);

			break;
		}

		case operating_mode::Train_Backbone:
		{
			current_mode = operating_mode::Train_Backbone;
			if (processing_mode == 2)
			{
				labels_name = labels_name_pnp;
				labels_color = labels_color_pnp;
			}
			//process single tile
			std::vector<int> seg_truth_train;
			std::vector< std::vector<float>> basic_feas_train, mulsc_ele_feas_train;
			std::vector< std::vector<float>> segment_eigen_feas_train, segment_color_feas_train;
			if (enable_augment && !augmented_data_exist)
				smote_data_augmentation();

			if (use_batch_processing)
			{
				//Batch separation
				std::vector<std::vector<std::pair<int, std::string>>> all_batches;
				read_txt_batches(all_batches);

				//Batch processing
				for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
				{
					std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
					training_feature_process_batch_tiles
					(
						seg_truth_train,
						basic_feas_train,
						segment_eigen_feas_train,
						segment_color_feas_train,
						mulsc_ele_feas_train,
						all_batches[bi],
						bi
					);
				}
			}
			else
			{
				for (std::size_t pi = 0; pi < base_names.size(); ++pi)
				{
					training_feature_process_single_tiles
					(
						seg_truth_train,
						basic_feas_train,
						segment_eigen_feas_train,
						segment_color_feas_train,
						mulsc_ele_feas_train,
						pi
					);
				}
			}

			std::cout << "Training mesh total area: " << training_mesh_area << std::endl;

			std::vector<Cluster_point> cluster_train = convert_to_CGAL_Cluster(basic_feas_train);
			Label_set labels_train;
			Feature_set node_features;
			CGAL::Classification::ETHZ::Random_forest_classifier classifier(labels_train, node_features);

			ETH_RF_Train_Base
			(
				cluster_train,
				seg_truth_train,
				basic_feas_train,
				segment_eigen_feas_train, segment_color_feas_train,
				mulsc_ele_feas_train,
				labels_train,
				node_features,
				classifier
			);

			break;
		}

		case operating_mode::Test_Backbone:
		{
			current_mode = operating_mode::Test_Backbone;
			if (processing_mode == 2)
			{
				labels_name = labels_name_pnp;
				labels_color = labels_color_pnp;
			}
			//read training models
			Label_set labels_test;
			Feature_set node_features;
			CGAL::Classification::ETHZ::Random_forest_classifier eth_rf_classifier(labels_test, node_features);
			std::cout << "Loading trained model -> ";
			std::ifstream fconfig;
			if (processing_mode == 0)
			{
				fconfig = std::ifstream(root_path
					+ folder_names_level_0[3]
					+ "trained_model.gz"
					, std::ios_base::in | std::ios_base::binary);
			}
			else
			{
				fconfig = std::ifstream(root_path
					+ folder_names_level_0[8]
					+ sota_folder_path
					+ folder_names_level_0[3]
					+ "trained_model.gz"
					, std::ios_base::in | std::ios_base::binary);
			}

			eth_rf_classifier.load_configuration(fconfig);
			std::cout << "Done !" << std::endl;
			fconfig.close();

			if (use_batch_processing)
			{
				//Batch separation
				std::vector<std::vector<std::pair<int, std::string>>> all_batches;
				read_txt_batches(all_batches);

				//Batch processing
				for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
				{
					std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
					testing_feature_process_batch_tiles_with_model(labels_test, node_features, eth_rf_classifier, all_batches[bi], bi);
					labels_test.clear();
					node_features.clear();
				}
			}
			else
			{
				for (std::size_t pi = 0; pi < base_names.size(); ++pi)
				{
					testing_feature_process_single_tiles_with_model(labels_test, node_features, eth_rf_classifier, pi);
					labels_test.clear();
					node_features.clear();
				}
			}
			break;
		}

		case operating_mode::Train_and_Test_Backbone:
		{
			current_mode = operating_mode::Train_and_Test_Backbone;
			if (processing_mode == 2)
			{
				labels_name = labels_name_pnp;
				labels_color = labels_color_pnp;
			}
			std::cout << "--------------------- Training Mode ---------------------" << std::endl;
			//--- For training data ---//
			std::vector<int> seg_truth_train;
			std::vector< std::vector<float> > basic_feas_train, mulsc_ele_feas_train;
			std::vector< std::vector<float> > segment_eigen_feas_train, segment_color_feas_train;
			if (enable_augment && !augmented_data_exist)
				smote_data_augmentation();

			if (use_batch_processing)
			{
				//Batch separation
				std::vector<std::vector<std::pair<int, std::string>>> all_batches;
				read_txt_batches(all_batches);

				//Batch processing
				for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
				{
					std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
					training_feature_process_batch_tiles
					(
						seg_truth_train,
						basic_feas_train,
						segment_eigen_feas_train,
						segment_color_feas_train,
						mulsc_ele_feas_train,
						all_batches[bi],
						bi
					);
				}
			}
			else
			{
				for (std::size_t pi = 0; pi < base_names.size(); ++pi)
				{
					training_feature_process_single_tiles
					(
						seg_truth_train,
						basic_feas_train,
						segment_eigen_feas_train,
						segment_color_feas_train,
						mulsc_ele_feas_train,
						pi
					);
				}
			}

			std::vector<Cluster_point> cluster_train = convert_to_CGAL_Cluster(basic_feas_train);
			Label_set labels_train;
			Feature_set node_features;
			CGAL::Classification::ETHZ::Random_forest_classifier classifier(labels_train, node_features);
			ETH_RF_Train_Base
			(
				cluster_train,
				seg_truth_train,
				basic_feas_train,
				segment_eigen_feas_train, segment_color_feas_train,
				mulsc_ele_feas_train,
				labels_train,
				node_features,
				classifier
			);

			//--- For testing data ---//
			process_data_selection["train"] = false;
			std::vector<bool> train_predict
			{
				process_data_selection["train"],
				process_data_selection["test"],
				process_data_selection["predict"],
				process_data_selection["validate"]
			};
			for (int tr_pr_i = 0; tr_pr_i < train_predict.size(); ++tr_pr_i)
			{
				if (train_predict[tr_pr_i])
				{
					changing_to_test_or_predict(tr_pr_i);
					if (use_batch_processing)
					{
						//Batch separation
						std::vector<std::vector<std::pair<int, std::string>>> all_batches;
						read_txt_batches(all_batches);

						//Batch processing
						for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
						{
							std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
							testing_feature_process_batch_tiles_without_model(classifier, node_features, all_batches[bi], bi);
						}
					}
					else
					{
						for (std::size_t pi = 0; pi < base_names.size(); ++pi)
						{
							testing_feature_process_single_tiles_without_model(classifier, node_features, pi);
						}
					}
				}
			}

			std::cout << "Training mesh total surface area: " << std::fixed << std::showpoint << std::setprecision(6)
				<< training_mesh_area << std::endl;
			std::cout << "Testing mesh total surface area: " << std::fixed << std::showpoint << std::setprecision(6)
				<< test_mesh_area << std::endl;

			break;
		}

		//--- Mesh feature extraction ---
		case operating_mode::Data_evaluation_for_all_tiles_config:
		{
			current_mode = operating_mode::Data_evaluation_for_all_tiles_config;
			if (processing_mode == 2)
			{
				labels_name = labels_name_pnp;
				labels_color = labels_color_pnp;
			}
			if (process_data_selection["test"])
			{
				if (processing_mode == 0)
					std::cout << "--------------------- RF Test data evaluation ---------------------" << std::endl;
				else if (processing_mode == 1)
					std::cout << "--------------------- SOTA Test data evaluation ---------------------" << std::endl;

				train_test_predict_val = 1;
				get_testing_data();
				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;

				run(operating_mode::Data_evaluation_for_all_tiles);
			}

			if (process_data_selection["validate"])
			{
				if (processing_mode == 0)
					std::cout << "--------------------- RF Validation data evaluation ---------------------" << std::endl;
				else if (processing_mode == 1)
					std::cout << "--------------------- SOTA Validation data evaluation ---------------------" << std::endl;

				train_test_predict_val = 3;
				get_validation_data();
				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				run(operating_mode::Data_evaluation_for_all_tiles);
			}

			break;
		}


		//--- Compute statistics for all test data ---
		case operating_mode::Data_evaluation_for_all_tiles:
		{
			current_mode = operating_mode::Data_evaluation_for_all_tiles;
			//add labels
			Label_set labels;
			add_labels(labels);

			//add test data 
			std::vector<int> face_truth_label, face_test_label;
			std::vector<float> face_area_weighted;
			if (!ignored_labels_name.empty())
				check_ignored_truth_labels();

			for (std::size_t mi = 0; mi < base_names.size(); ++mi)
			{
				//read original mesh
				SFMesh *smesh_original = new SFMesh;
				read_test_mesh_data(smesh_original, mi);

				for (auto fd : smesh_original->faces())
				{
					if (smesh_original->get_face_truth_label[fd] != 0 && smesh_original->get_face_truth_label[fd] != -1)
					{
						if (!ignored_labels_name.empty())
						{
							if (!label_ignore[smesh_original->get_face_truth_label[fd] - 1])
							{
								int new_pred_label = smesh_original->get_face_predict_label[fd] - 1 - label_shiftdis[smesh_original->get_face_predict_label[fd] - 1];
								int new_truth_label = smesh_original->get_face_truth_label[fd] - 1 - label_shiftdis[smesh_original->get_face_truth_label[fd] - 1];

								face_truth_label.push_back(new_truth_label);
								face_test_label.push_back(new_pred_label);
								face_area_weighted.push_back(FaceArea(smesh_original, fd));
							}
						}
						else
						{
							face_truth_label.push_back(smesh_original->get_face_truth_label[fd] - 1);
							face_test_label.push_back(smesh_original->get_face_predict_label[fd] - 1);
							face_area_weighted.push_back(FaceArea(smesh_original, fd));
						}
					}
				}

				//output error map
				if (save_error_map)
					error_map(smesh_original, mi);

				delete smesh_original;
			}

			evaluation_all_test_data
			(
				labels,
				face_truth_label,
				face_test_label,
				face_area_weighted
			);

			break;
		}

//---------------------------------------------------------------------------------------------------------------------------------------------------
		case operating_mode::Process_semantic_pcl:
		{
			current_mode = operating_mode::Process_semantic_pcl;
			std::cout << "--------------------- Parsing semantics from point cloud to mesh ---------------------" << std::endl;
			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Testing Mode ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();
				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				use_batch_processing = use_batch_processing_on_testing;

				if (use_batch_processing)
				{
					//Batch separation
					std::vector<std::vector<std::pair<int, std::string>>> all_batches;
					read_txt_batches(all_batches);

					//Batch processing
					for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
					{
						std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
						semantic_pcl_process_batch_tiles(all_batches[bi], bi);
					}
				}
				else
				{
					for (std::size_t pi = 0; pi < base_names.size(); ++pi)
					{
						semantic_pcl_process_single_tiles(pi);
					}
				}
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Predicting Mode ---------------------" << std::endl;
				process_data_selection["test"] = false;
				train_test_predict_val = 2;
				get_predicting_data();
				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;

				use_batch_processing = use_batch_processing_on_predicting;
				if (use_batch_processing)
				{
					//Batch separation
					std::vector<std::vector<std::pair<int, std::string>>> all_batches;
					read_txt_batches(all_batches);

					//Batch processing
					for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
					{
						std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
						semantic_pcl_process_batch_tiles(all_batches[bi], bi);
					}
				}
				else
				{
					for (std::size_t pi = 0; pi < base_names.size(); ++pi)
					{
						semantic_pcl_process_single_tiles(pi);
					}
				}
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- validation Mode ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();
				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				use_batch_processing = use_batch_processing_on_validation;

				if (use_batch_processing)
				{
					//Batch separation
					std::vector<std::vector<std::pair<int, std::string>>> all_batches;
					read_txt_batches(all_batches);

					//Batch processing
					for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
					{
						std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
						semantic_pcl_process_batch_tiles(all_batches[bi], bi);
					}
				}
				else
				{
					for (std::size_t pi = 0; pi < base_names.size(); ++pi)
					{
						semantic_pcl_process_single_tiles(pi);
					}
				}
			}
			break;
		}

		//--- Mesh feature visualization ---
		case operating_mode::Save_mesh_features_for_visualization:
		{
			current_mode = operating_mode::Save_mesh_features_for_visualization;

			if (process_data_selection["train"])
			{
				std::cout << "--------------------- Saving training mesh features for visualization ---------------------" << std::endl;
				train_test_predict_val = 0;
				get_training_data();

				data_path = training_data_path;
				base_names = training_base_names;
				ply_files = training_ply_files;
				file_folders = training_file_folders;
				file_ind_map = training_file_ind_map;
				use_batch_processing = use_batch_processing_on_training;
				//visualization features by write into mesh
				run(operating_mode::Visualization_features_backbone);
			}

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Saving testing mesh features for visualization ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();

				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;
				//visualization features by write into mesh
				run(operating_mode::Visualization_features_backbone);
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Saving predict mesh features for visualization ---------------------" << std::endl;
				train_test_predict_val = 2;
				get_predicting_data();

				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				file_ind_map = predicting_file_ind_map;
				use_batch_processing = use_batch_processing_on_predicting;
				//visualization features by write into mesh
				run(operating_mode::Visualization_features_backbone);
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- Saving validate mesh features for visualization ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();

				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				//visualization features by write into mesh
				run(operating_mode::Visualization_features_backbone);
			}

			break;
		}

		//--- write features on mesh faces for visualization ---
		case operating_mode::Visualization_features_backbone:
		{
			current_mode = operating_mode::Visualization_features_backbone;

			if (use_batch_processing)
			{
				//Batch separation
				std::vector<std::vector<std::pair<int, std::string>>> all_batches;
				read_txt_batches(all_batches);

				//Batch processing
				for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
				{
					std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
					visualization_process_batch_tiles(all_batches[bi], bi);
				}
			}
			else
			{
				//process single tile
				for (std::size_t pi = 0; pi < base_names.size(); ++pi)
 					visualization_process_single_tiles(pi);
			}
			break;
		}
//----------------------------------------------------------------------------------------------------------------------------------------------------

		//--- Compute semantic class statistics ---
		case operating_mode::Class_statistics:
		{
			current_mode = operating_mode::Class_statistics;
			use_existing_mesh_segments = true;

			//--- For testing data ---//
			std::vector<bool> train_predict
			{
				process_data_selection["train"],
				process_data_selection["test"],
				process_data_selection["predict"],
				process_data_selection["validate"]
			};

			for (int tr_pr_i = 0; tr_pr_i < train_predict.size(); ++tr_pr_i)
			{
				if (train_predict[tr_pr_i])
				{
					changing_to_test_or_predict(tr_pr_i);

					std::vector<float> label_statistics(labels_name.size() + 1, 0);
					std::vector<float> label_seg_statistics(labels_name.size() + 1, 0);
					for (std::size_t mi = 0; mi < base_names.size(); ++mi)
					{
						if (input_type_for_statistics == 0)
						{
							//read original mesh
							SFMesh *smesh_original = new SFMesh;
							read_labeled_mesh_data(smesh_original, mi);

							SFMesh *smesh_seg = new SFMesh;
							read_mesh_data(smesh_seg, mi);

							conversion_from_ground_truth(smesh_seg, label_seg_statistics);

							for (auto fd : smesh_original->faces())
							{
								int current_label_ind = smesh_original->get_face_truth_label[fd];
								if (smesh_original->get_face_truth_label[fd] != 0 && smesh_original->get_face_truth_label[fd] != -1)
								{
									label_statistics[current_label_ind] += FaceArea(smesh_original, fd);
								}
								else
								{
									label_statistics[0] += FaceArea(smesh_original, fd);
								}
							}

							delete smesh_seg;
							delete smesh_original;
						}
						else if (input_type_for_statistics == 1)
						{
							PTCloud *cloud = new PTCloud;
							read_pointcloud_data(cloud, 0, mi);

							auto point_label = cloud->get_points_ground_truth;
							for (auto ptx : cloud->vertices())
							{
								int current_label_ind = point_label[ptx];
								if (current_label_ind != 0 && current_label_ind != -1)
								{
									label_statistics[current_label_ind] += 1;
								}
								else
								{
									label_statistics[0] += 1;
								}
							}
							delete cloud;
						}
					}

					save_txt_statistics(label_statistics, label_seg_statistics);
				}
			}
			
			break;
		}

		//--- sampling point cloud ---
		case operating_mode::Generate_semantic_sampled_points:
		{
			current_mode = operating_mode::Generate_semantic_sampled_points;
			use_existing_mesh_segments = true;
			add_point_color = true;

			sampling_point_density = 7.0f;//for deep learning methods, same as in SUM paper

			//--- For testing data ---//
			std::vector<bool> train_predict
			{
				process_data_selection["train"],
				process_data_selection["test"],
				process_data_selection["predict"],
				process_data_selection["validate"]
			};

			for (int tr_pr_i = 0; tr_pr_i < train_predict.size(); ++tr_pr_i)
			{
				if (train_predict[tr_pr_i])
				{
					changing_to_test_or_predict(tr_pr_i);

					std::cout << "Generate sampled semantic point cloud. " << std::endl;
					std::vector<SFMesh*> mesh_vec;
					std::vector<PTCloud*> pcl_vec;

					for (std::size_t mi = 0; mi < base_names.size(); ++mi)
					{
						SFMesh *smesh_out = new SFMesh;
						PTCloud *cloud = new PTCloud;

						//--- read mesh *.ply data ---
						std::vector<cv::Mat> texture_maps;
						read_mesh_data(smesh_out, mi, texture_maps);

						//--- sampling point cloud on mesh data ---
						sampling_point_cloud_on_mesh(smesh_out, texture_maps, cloud, mi);

						//save sampled point cloud
						cloud->remove_all_properties();
						write_pointcloud_data(cloud, 0, mi);

						delete cloud;
						delete smesh_out;
					}
				}
			}
			
			break;
		}


		//--- Feature diversity measure ---
		case operating_mode::Feature_diversity_meaure:
		{
			current_mode = operating_mode::Feature_diversity_meaure;
			std::vector<float> feas_var;
			std::vector<std::string> all_names;

			//train data
			std::cout << "--------------------- on training data ---------------------" << std::endl;
			train_test_predict_val = 0;
			get_training_data();
			data_path = training_data_path;
			base_names = training_base_names;
			ply_files = training_ply_files;
			file_folders = training_file_folders;
			file_ind_map = training_file_ind_map;
			use_batch_processing = use_batch_processing_on_training;
			all_names.insert(all_names.end(), base_names.begin(), base_names.end());
			compute_feature_diversity(feas_var);

			//test data
			std::cout << "--------------------- on test data ---------------------" << std::endl;
			train_test_predict_val = 1;
			get_testing_data();
			data_path = testing_data_path;
			base_names = testing_base_names;
			ply_files = testing_ply_files;
			file_folders = testing_file_folders;
			file_ind_map = testing_file_ind_map;
			use_batch_processing = use_batch_processing_on_testing;
			all_names.insert(all_names.end(), base_names.begin(), base_names.end());
			compute_feature_diversity(feas_var);

			//test data
			std::cout << "--------------------- on validation data ---------------------" << std::endl;
			train_test_predict_val = 3;
			get_validation_data();
			data_path = validation_data_path;
			base_names = validation_base_names;
			ply_files = validation_ply_files;
			file_folders = validation_file_folders;
			file_ind_map = validation_file_ind_map;
			use_batch_processing = use_batch_processing_on_validation;
			all_names.insert(all_names.end(), base_names.begin(), base_names.end());
			compute_feature_diversity(feas_var);

			//write data
			save_txt_feature_divergence(all_names, feas_var);
			break;
		}

		case operating_mode::Compute_mesh_area:
		{
			current_mode = operating_mode::Compute_mesh_area;

			//train
			train_test_predict_val = 0;
			get_training_data();
			data_path = training_data_path;
			base_names = training_base_names;
			ply_files = training_ply_files;
			file_folders = training_file_folders;
			file_ind_map = training_file_ind_map;
			use_batch_processing = use_batch_processing_on_training;

			std::cout << "--------------------- on training data ---------------------" << std::endl;
			std::vector<std::pair<std::string, float>> mesh_area;
			std::vector<std::pair<std::string, std::vector<float>>> mesh_class_area;
			compute_mesh_area(mesh_area, mesh_class_area);

			std::cout << "--------------------- on test data ---------------------" << std::endl;
			train_test_predict_val = 1;
			get_testing_data();
			data_path = testing_data_path;
			base_names = testing_base_names;
			ply_files = testing_ply_files;
			file_folders = testing_file_folders;
			file_ind_map = testing_file_ind_map;
			use_batch_processing = use_batch_processing_on_testing;
			compute_mesh_area(mesh_area, mesh_class_area);

			save_txt_mesh_areas(mesh_area, mesh_class_area);

			break;
		}

		//--- SOTA Moha Verdi method ---
		case operating_mode::Moha_Verdi_SOTA_pipeline:
		{
			current_mode = operating_mode::Moha_Verdi_SOTA_pipeline;
			processing_mode = 1;

			sota_folder_path = "moha/";
			sota_prefixs = prefixs[4] + prefixs[5];

			enable_MohaVer_region_growing = true;
			enable_joint_labeling = true;

			rf_tree_numbers = 100;
			rf_tree_depth = 50;

			//set up features based on default SUM feature setting
			use_feas[0] = false;
			for (auto basic_fea_i : use_basic_features)
				basic_fea_i.second = false;
			for (auto eigen_fea_i : use_eigen_features)
				eigen_fea_i.second = false;
			use_eigen_features[3] = true;
			use_eigen_features[5] = true;

			use_color_features[0] = false;
			use_color_features[1] = false;
			use_color_features[2] = false;
			use_color_features[6] = false;

			//run(operating_mode::Mesh_feature_extraction);
			run(operating_mode::Train_and_Test_config);
			run(operating_mode::Data_evaluation_for_all_tiles_config);
			break;
		}

		//Evaluation on SOTA method
		case operating_mode::Evaluation_SOTA:
		{
			current_mode = operating_mode::Evaluation_SOTA;
			processing_mode = 1;

			run(operating_mode::Process_semantic_pcl);
			previous_mode = operating_mode::Process_semantic_pcl;
			run(operating_mode::Data_evaluation_for_all_tiles_config);
			previous_mode = operating_mode::Data_evaluation_for_all_tiles_config;
			break;
		}

					//----------------------------------------------------------------------------------------------------------------------------------------------------

		//--- PSSNet pipeline ----
		//--- write selected features on mesh faces for GCN ---
		case operating_mode::PSSNet_pipeline_for_GCN:
		{
			current_mode = operating_mode::PSSNet_pipeline_for_GCN;
			run(operating_mode::Get_labels_for_planar_non_planar_from_semantic);
			run(operating_mode::Pipeline);
			run(operating_mode::PSSNet_oversegmentation);
			run(operating_mode::PSSNet_oversegmentation_evaluation);
			run(operating_mode::PSSNet_graph_construction);
			run(operating_mode::PSSNet_pcl_generation_for_GCN);
			break;
		}

		//--- Get labels for planar and non-planar data from semantic labels ---
		case operating_mode::Get_labels_for_planar_non_planar_from_semantic:
		{
			current_mode = operating_mode::Get_labels_for_planar_non_planar_from_semantic;
			int pre_processing_mode = processing_mode;
			processing_mode = 0;

			std::vector<bool> train_predict
			{
				process_data_selection["train"],
				process_data_selection["test"],
				process_data_selection["predict"],
				process_data_selection["validate"]
			};

			for (int tr_pr_i = 0; tr_pr_i < train_predict.size(); ++tr_pr_i)
			{
				if (train_predict[tr_pr_i] && tr_pr_i != 2)
				{
					changing_to_test_or_predict(tr_pr_i);

					std::cout << "Get labels for planar and non-planar data from semantic labels." << std::endl;

					for (std::size_t mi = 0; mi < base_names.size(); ++mi)
					{
						if (train_predict[tr_pr_i])
						{
							changing_to_test_or_predict(tr_pr_i);

							//process single tile
							for (int mi = 0; mi < base_names.size(); ++mi)
							{
								SFMesh *smesh_original = new SFMesh;

								//read original mesh
								read_labeled_mesh_data(smesh_original, mi);

								//merge semantics
								for (auto fd : smesh_original->faces())
								{
									std::string current_label;
									if (smesh_original->get_face_truth_label[fd] != 0)
									{
										int current_label_ind = smesh_original->get_face_truth_label[fd] - 1;
										if (current_label_ind >= 0 && current_label_ind < labels_name.size())
											current_label = labels_name[current_label_ind];
	
										bool matched = false;
										for (auto la : L1_to_L0_label_map)
										{
											if (la.first.compare(current_label) == 0)
											{
												matched = true;
												smesh_original->get_face_truth_label[fd] = la.second + 1;
												smesh_original->get_face_color[fd] = labels_color_pnp[la.second];
												break;
											}
										}

										if (!matched)
										{
											matched = true;
											smesh_original->get_face_truth_label[fd] = -1;
											smesh_original->get_face_color[fd] = easy3d::vec3(0.0f, 0.0f, 0.0f);
										}
									}
								}

								//write L0 mesh
								write_pnp_mesh_data(smesh_original, mi);

								delete smesh_original;
							}
						}
					}
				}
			}
			
			processing_mode = pre_processing_mode;
			break;
		}

		//--- PSSNet over-segmentation ---
		case operating_mode::PSSNet_oversegmentation:
		{
			current_mode = operating_mode::PSSNet_oversegmentation;

			if (process_data_selection["train"])
			{
				std::cout << "--------------------- PSSNet oversegmented train mesh ---------------------" << std::endl;
				train_test_predict_val = 0;
				get_training_data();

				data_path = training_data_path;
				base_names = training_base_names;
				ply_files = training_ply_files;
				file_folders = training_file_folders;
				file_ind_map = training_file_ind_map;
				use_batch_processing = use_batch_processing_on_training;
				use_existing_mesh_segments = use_existing_mesh_segments_on_training;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_training;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_training;
				use_merged_segments = use_merged_segments_on_training;
				sampling_strategy = sampling_strategy_training;
				run(operating_mode::PSSNet_oversegmentation_backbone);
			}

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- PSSNet oversegmented test mesh  ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();

				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;
				use_existing_mesh_segments = use_existing_mesh_segments_on_testing;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_testing;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_testing;
				use_merged_segments = use_merged_segments_on_testing;
				sampling_strategy = sampling_strategy_testing;
				run(operating_mode::PSSNet_oversegmentation_backbone);
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- PSSNet oversegmented predict mesh  ---------------------" << std::endl;
				train_test_predict_val = 2;
				get_predicting_data();

				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				file_ind_map = predicting_file_ind_map;
				use_batch_processing = use_batch_processing_on_predicting;
				use_existing_mesh_segments = use_existing_mesh_segments_on_predicting;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_predicting;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_predicting;
				use_merged_segments = use_merged_segments_on_predicting;
				sampling_strategy = sampling_strategy_predicting;
				run(operating_mode::PSSNet_oversegmentation_backbone);
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- PSSNet oversegmented validate mesh  ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();

				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				use_existing_mesh_segments = use_existing_mesh_segments_on_validation;
				use_pointcloud_region_growing = use_pointcloud_region_growing_on_validation;
				use_face_pixels_color_aggregation = use_face_pixels_color_aggregation_on_validation;
				use_merged_segments = use_merged_segments_on_validation;
				sampling_strategy = sampling_strategy_validation;
				run(operating_mode::PSSNet_oversegmentation_backbone);
			}

			break;
		}

		//--- Planar and non-planar based region growing ---
		case operating_mode::PSSNet_oversegmentation_backbone:
		{
			current_mode = operating_mode::PSSNet_oversegmentation_backbone;
			std::cout << "	- Planar and non-planar based region growing." << std::endl;

			//--- read mesh with planar and non-planar label ---// 
			if (use_batch_processing)
			{
				std::cout << "Batch mode is not currently supported in PSSNet over-segmentation." << std::endl;
				break;
			}
			else
			{
				for (std::size_t pi = 0; pi < base_names.size(); ++pi)
				{
					PNP_MRF_single_tiles(pi);
				}
			}

			break;
		}

		case operating_mode::PSSNet_oversegmentation_evaluation:
		{
			current_mode = operating_mode::PSSNet_oversegmentation_evaluation;
			std::cout << "Evaluation Operating mode: PSSNet_oversegmentation_evaluation. " << std::endl;
			std::cout << "	Processing test data and ground truth data." << std::endl;
			std::vector<bool> train_predict
			{
				process_data_selection["train"],
				process_data_selection["test"],
				process_data_selection["predict"],
				process_data_selection["validate"]
			};

			for (int tr_pr_i = 0; tr_pr_i < train_predict.size(); ++tr_pr_i)
			{
				if (train_predict[tr_pr_i] && tr_pr_i != 2)
				{
					changing_to_test_or_predict(tr_pr_i);

					int all_face_num = 0;
					all_eval *all_seg_evaluation = new all_eval();

					if (generate_groundtruth_segments)
						read_and_write_oversegmentation_ground_truth();

					for (int ti = 0; ti < base_names.size(); ++ti)
					{
						SFMesh* test_mesh = new SFMesh;
						read_oversegmentation_testdata(test_mesh, ti);

						SFMesh* truth_mesh = new SFMesh;
						read_oversegmentation_truthdata(truth_mesh, ti);

						oversegmentation_evaluation(truth_mesh, test_mesh, ti, all_seg_evaluation);
						all_face_num += test_mesh->faces_size();

						delete truth_mesh;
						delete test_mesh;
					}

					delete all_seg_evaluation;
					std::cout << "all_face_num = " << all_face_num << std::endl;
				}
			}

			break;
		}

		case operating_mode::PSSNet_pcl_generation_for_GCN:
		{
			current_mode = operating_mode::PSSNet_pcl_generation_for_GCN;
			//default setting
			GCN_feature_write_configurations();

			if (process_data_selection["train"])
			{
				std::cout << "--------------------- Write train mesh feature for GCN : L1 ---------------------" << std::endl;

				train_test_predict_val = 0;
				get_training_data();
				data_path = training_data_path;
				base_names = training_base_names;
				ply_files = training_ply_files;
				file_folders = training_file_folders;
				file_ind_map = training_file_ind_map;
				use_batch_processing = use_batch_processing_on_training;
				sampling_strategy = sampling_strategy_training;
				run(operating_mode::PSSNet_pcl_generation_for_GCN_backbone);
			}

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Write test mesh feature for GCN : L1 ---------------------" << std::endl;

				train_test_predict_val = 1;
				get_testing_data();
				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				file_ind_map = testing_file_ind_map;
				use_batch_processing = use_batch_processing_on_testing;
				sampling_strategy = sampling_strategy_testing;
				run(operating_mode::PSSNet_pcl_generation_for_GCN_backbone);
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Write predict mesh feature for GCN : L1 ---------------------" << std::endl;

				train_test_predict_val = 2;
				get_predicting_data();
				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				file_ind_map = predicting_file_ind_map;
				use_batch_processing = use_batch_processing_on_predicting;
				sampling_strategy = sampling_strategy_predicting;
				run(operating_mode::PSSNet_pcl_generation_for_GCN_backbone);
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- Write validate mesh feature for GCN : L1 ---------------------" << std::endl;

				train_test_predict_val = 3;
				get_validation_data();
				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				file_ind_map = validation_file_ind_map;
				use_batch_processing = use_batch_processing_on_validation;
				sampling_strategy = sampling_strategy_validation;
				run(operating_mode::PSSNet_pcl_generation_for_GCN_backbone);
			}

			break;
		}

		//--- write selected features on mesh faces for GCN ---
		case operating_mode::PSSNet_pcl_generation_for_GCN_backbone:
		{
			current_mode = operating_mode::PSSNet_pcl_generation_for_GCN_backbone;
			//process single tile
			for (std::size_t pi = 0; pi < base_names.size(); ++pi)
			{
				if (!only_write_GCN_features)
					process_single_tile(pi);
				feature_selection_for_GCN_single_tiles(pi);
			}

			break;
		}

		case operating_mode::PSSNet_graph_construction:
		{
			current_mode = operating_mode::PSSNet_graph_construction;

			if (process_data_selection["train"])
			{
				std::cout << "--------------------- Training data graph construction ---------------------" << std::endl;
				train_test_predict_val = 0;
				get_training_data();
				data_path = training_data_path;
				base_names = training_base_names;
				ply_files = training_ply_files;
				file_folders = training_file_folders;
				use_batch_processing = use_batch_processing_on_training;
				file_ind_map = training_file_ind_map;
				sampling_strategy = sampling_strategy_training;
				run(operating_mode::PSSNet_graph_construction_backbone);
				process_data_selection["train"] = false;
			}

			if (process_data_selection["test"])
			{
				std::cout << "--------------------- Testing data graph construction ---------------------" << std::endl;
				train_test_predict_val = 1;
				get_testing_data();
				data_path = testing_data_path;
				base_names = testing_base_names;
				ply_files = testing_ply_files;
				file_folders = testing_file_folders;
				use_batch_processing = use_batch_processing_on_testing;
				file_ind_map = testing_file_ind_map;
				sampling_strategy = sampling_strategy_testing;
				run(operating_mode::PSSNet_graph_construction_backbone);
				process_data_selection["test"] = false;
			}

			if (process_data_selection["predict"])
			{
				std::cout << "--------------------- Predicting data graph construction ---------------------" << std::endl;
				train_test_predict_val = 2;
				get_predicting_data();
				data_path = predicting_data_path;
				base_names = predicting_base_names;
				ply_files = predicting_ply_files;
				file_folders = predicting_file_folders;
				use_batch_processing = use_batch_processing_on_predicting;
				file_ind_map = predicting_file_ind_map;
				sampling_strategy = sampling_strategy_predicting;
				run(operating_mode::PSSNet_graph_construction_backbone);
				process_data_selection["predict"] = false;
			}

			if (process_data_selection["validate"])
			{
				std::cout << "--------------------- Validation data graph construction ---------------------" << std::endl;
				train_test_predict_val = 3;
				get_validation_data();
				data_path = validation_data_path;
				base_names = validation_base_names;
				ply_files = validation_ply_files;
				file_folders = validation_file_folders;
				use_batch_processing = use_batch_processing_on_validation;
				file_ind_map = validation_file_ind_map;
				sampling_strategy = sampling_strategy_validation;
				run(operating_mode::PSSNet_graph_construction_backbone);
				process_data_selection["validate"] = false;
			}
			break;
		}

		case operating_mode::PSSNet_graph_construction_backbone:
		{
			current_mode = operating_mode::PSSNet_graph_construction_backbone;

			std::cout << "Operating mode: Graph Construction. " << std::endl;

			if (use_batch_processing)
			{
				std::cout << "Batch mode is not currently supported in PSSNet over-segmentation." << std::endl;
				break;
			}
			else
			{
				for (int mi = 0; mi < base_names.size(); ++mi)
				{
					SFMesh* smesh_seg = new SFMesh;
					PTCloud *cloud_in = new PTCloud, *cloud_ele = new PTCloud;
					std::vector<superfacets> spf_final;
					if (with_node_graphs)
						read_graph_nodes_ply(spf_final, mi);
					else
					{
						read_seg_mesh_with_pnp_info(smesh_seg, mi);

						read_pointcloud_data(smesh_seg, cloud_in, 0, mi);
						read_pointcloud_data(smesh_seg, cloud_ele, 1, mi);

						//get superfacets
						get_superfacets(smesh_seg, cloud_in, cloud_ele, spf_final);

						//saving graph nodes
						if (!with_node_graphs)
							save_graph_nodes_ply(spf_final, cloud_in, mi);
					}

					std::cout << "cloud_in.size() = " << cloud_in->vertices_size() << std::endl;

					//construct spf graph
					std::vector<std::pair<int, int>> spf_edges, spf_sampts_edges;
					std::map<std::pair<int, int>, bool> check_segneg_visited, check_combined_segpt_visited;
					segment_geometric_relationships(smesh_seg, cloud_in, spf_final, spf_edges, spf_sampts_edges, check_segneg_visited, check_combined_segpt_visited);

					if (use_edges_between_boder_points)
					{
						construct_segment_sampled_point_edges(spf_final, spf_edges, spf_sampts_edges, check_combined_segpt_visited);
						save_graph_edges_ply(cloud_in, spf_sampts_edges, mi);
					}
					else
					{
						save_graph_edges_ply(cloud_in, spf_edges, mi, spf_final);
					}

					delete smesh_seg;
					delete cloud_in;
					delete cloud_ele;
				}
			}

			break;
		}

		default:
		{
			std::cerr << std::endl << "No operation model has been chosen!!!" << std::endl;
			break;
		}
        }
    }

	void changing_to_test_or_predict(const int tr_pr_i)
	{
		if (tr_pr_i == 0
			&& process_data_selection["train"])
		{
			std::cout << "--------------------- on training data ---------------------" << std::endl;
			//process_data_selection["train"] = false;
			train_test_predict_val = 0;
			get_training_data();
			data_path = training_data_path;
			base_names = training_base_names;
			ply_files = training_ply_files;
			file_folders = training_file_folders;
			file_ind_map = training_file_ind_map;
			sampling_strategy = sampling_strategy_training;
			use_batch_processing = use_batch_processing_on_training;
		}

		if (tr_pr_i == 1
			&& process_data_selection["test"])
		{
			std::cout << "--------------------- on test data ---------------------" << std::endl;
			//process_data_selection["test"] = false;
			train_test_predict_val = 1;
			get_testing_data();
			data_path = testing_data_path;
			base_names = testing_base_names;
			ply_files = testing_ply_files;
			file_folders = testing_file_folders;
			file_ind_map = testing_file_ind_map;
			sampling_strategy = sampling_strategy_testing;
			use_batch_processing = use_batch_processing_on_testing;
		}

		if (tr_pr_i == 2
			&& process_data_selection["predict"])
		{
			std::cout << "--------------------- on predict data---------------------" << std::endl;
			//process_data_selection["predict"] = false;
			train_test_predict_val = 2;
			get_predicting_data();
			data_path = predicting_data_path;
			base_names = predicting_base_names;
			ply_files = predicting_ply_files;
			file_folders = predicting_file_folders;
			file_ind_map = predicting_file_ind_map;
			sampling_strategy = sampling_strategy_predicting;
			use_batch_processing = use_batch_processing_on_predicting;
		}

		if (tr_pr_i == 3
			&& process_data_selection["validate"])
		{
			std::cout << "--------------------- on validation data ---------------------" << std::endl;
			//process_data_selection["validate"] = false;
			train_test_predict_val = 3;
			get_validation_data();
			data_path = validation_data_path;
			base_names = validation_base_names;
			ply_files = validation_ply_files;
			file_folders = validation_file_folders;
			file_ind_map = validation_file_ind_map;
			sampling_strategy = sampling_strategy_validation;
			use_batch_processing = use_batch_processing_on_validation;
		}
	}
}