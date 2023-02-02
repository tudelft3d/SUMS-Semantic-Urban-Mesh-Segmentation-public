/*
*   Name        : mesh_classifier.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : classification of input features, including random forest classifier and MRF, and results evaluation.
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

#include "mesh_classifier.hpp"
#include "feature_computation.hpp"

namespace semantic_mesh_segmentation
{
	//--- adding label names ---
	void add_labels
	(
		Label_set &labels,
		bool enable_double_lables
	)
	{
		for (size_t li = 0; li < labels_name.size(); ++li)
		{
			if (!ignored_labels_name.empty())
			{
				bool is_ignore = false;
				for (int ig_i = 0; ig_i < ignored_labels_name.size(); ++ig_i)
				{
					if (ignored_labels_name[ig_i] == labels_name[li])
					{
						is_ignore = true;
						break;
					}
				}
				if (!is_ignore)
					labels.add(labels_name[li].c_str());
			}
			else
			{
				labels.add(labels_name[li].c_str());
			}
		}

		if (enable_double_lables)
		{
			for (size_t li2 = 0; li2 < labels_name.size(); ++li2)
			{
				std::string joint_label = labels_name[li2] + "2";
				if (!ignored_labels_name.empty())
				{
					bool is_ignore = false;
					for (int ig_i = 0; ig_i < ignored_labels_name.size(); ++ig_i)
					{
						if (ignored_labels_name[ig_i] == labels_name[li2])
						{
							is_ignore = true;
							break;
						}
					}
					if (!is_ignore)
						labels.add(joint_label.c_str());
				}
				else
				{
					labels.add(joint_label.c_str());
				}
			}
		}
	}

	//--- adding feature names ---
	void add_feature_names
	(
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &segment_eigen_feas,
		std::vector< std::vector<float> > &segment_color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		Feature_set &node_features,
		int &feature_num,
		std::vector<std::string> &feature_bank
	)
	{
		int sc_2_fea_size = multi_scale_ele_radius.size();

		if (use_feas[0] == true)
		{
			for (int fi = 0; fi < basic_feas[0].size(); ++fi)
			{
				if (use_basic_features[fi] == true)
				{
					node_features.add<segment_features>(basic_feas, fi);
					feature_bank.push_back(basic_feature_base_names[fi].first);
					++feature_num;
				}
			}
		}
		//controlled by muti scale2 elevations 
		std::vector<std::pair<std::string, int>> temp_ele_feature_base_names;
		if (!multi_scale_ele_radius.empty())
		{
			for (int sci = 0; sci < sc_2_fea_size; ++sci)
			{
				temp_ele_feature_base_names.push_back(std::pair<std::string, int>("multiscale_2_ele_fea_" + std::to_string(sci), sci));
			}
		}

		if (use_feas[3] == true)
		{
			for (int fi = 0; fi < mulsc_ele_feas[0].size(); ++fi)
			{
				if (use_mulsc_eles[fi] == true)
				{
					node_features.add<segment_features>(mulsc_ele_feas, fi);
					feature_bank.push_back(temp_ele_feature_base_names[fi].first);
					++feature_num;
				}
			}
		}

		for (int fi = 0; fi < segment_eigen_feas[0].size(); ++fi)
		{
			if (use_feas[1] == true && use_eigen_features[fi] == true)
			{
				node_features.add<segment_features>(segment_eigen_feas, fi);
				feature_bank.push_back(eigen_feature_base_names[fi].first);
				++feature_num;
			}
		}

		for (int fi = 0; fi < segment_color_feas[0].size(); ++fi)
		{
			if (use_feas[2] == true && use_color_features[fi] == true)
			{
				node_features.add<segment_features>(segment_color_feas, fi);
				feature_bank.push_back(color_feature_base_names[fi].first);
				++feature_num;
			}
		}
	}

	//--- adding label names ---
	void add_joint_feature_names
	(
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &segment_eigen_feas,
		std::vector< std::vector<float> > &segment_color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		Feature_set &node_features,
		int &feature_num,
		std::vector<std::string> &feature_bank
	)
	{
		int sc_2_fea_size = multi_scale_ele_radius.size() * 2;
		if (use_feas[0] == true)
		{
			for (int fi = 0; fi < basic_feas[0].size(); ++fi)
			{
				int fi_r = fi >= use_basic_features.size() ? fi - use_basic_features.size() : fi;
				if (use_basic_features[fi_r] == true)
				{
					node_features.add<segment_features>(basic_feas, fi);
					if (fi < use_basic_features.size())
						feature_bank.push_back(basic_feature_base_names[fi].first);
					else
						feature_bank.push_back(basic_feature_base_names[fi_r].first + "_2");
					++feature_num;
				}
			}
		}
		//controlled by multi scale2 elevations 
		std::vector<std::pair<std::string, int>> temp_ele_feature_base_names;
		if (!multi_scale_ele_radius.empty())
		{
			for (int sci = 0; sci < sc_2_fea_size; ++sci)
			{
				int sci_r = sci >= use_mulsc_eles.size() ? sci - use_mulsc_eles.size() : sci;
				temp_ele_feature_base_names.push_back(std::pair<std::string, int>("multiscale_2_ele_fea_" + std::to_string(sci), sci));
			}
		}

		if (use_feas[3] == true)
		{
			for (int fi = 0; fi < mulsc_ele_feas[0].size(); ++fi)
			{
				int fi_r = fi >= use_mulsc_eles.size() ? fi - use_mulsc_eles.size() : fi;
				if (use_mulsc_eles[fi_r] == true)
				{
					node_features.add<segment_features>(mulsc_ele_feas, fi);
					if (fi < use_mulsc_eles.size())
						feature_bank.push_back(temp_ele_feature_base_names[fi].first);
					else
						feature_bank.push_back(temp_ele_feature_base_names[fi_r].first + "_2");
					++feature_num;
				}
			}
		}

		for (int fi = 0; fi < segment_eigen_feas[0].size(); ++fi)
		{
			int fi_r = fi >= use_eigen_features.size() ? fi - use_eigen_features.size() : fi;
			if (use_feas[1] == true && use_eigen_features[fi_r] == true)
			{
				node_features.add<segment_features>(segment_eigen_feas, fi);
				if (fi < use_eigen_features.size())
					feature_bank.push_back(eigen_feature_base_names[fi].first);
				else
					feature_bank.push_back(eigen_feature_base_names[fi_r].first + "_2");
				++feature_num;
			}

		}

		for (int fi = 0; fi < segment_color_feas[0].size(); ++fi)
		{
			int fi_r = fi >= use_color_features.size() ? fi - use_color_features.size() : fi;
			if (use_feas[2] == true && use_color_features[fi_r] == true)
			{
				node_features.add<segment_features>(segment_color_feas, fi);
				if (fi < use_color_features.size())
					feature_bank.push_back(color_feature_base_names[fi].first);
				else
					feature_bank.push_back(color_feature_base_names[fi_r].first + "_2");
				++feature_num;
			}
		}
	}

	//----------------------------------------------- Train Module -----------------------------------------------//
	void ETH_RF_Train_Base
	(
		std::vector<Cluster_point> &clusters_train,
		std::vector<int> &ground_truth_label_train,
		std::vector< std::vector<float>> &basic_feas_train,
		std::vector< std::vector<float> > &segment_eigen_feas_train,
		std::vector< std::vector<float> > &segment_color_feas_train,
		std::vector< std::vector<float> > &mulsc_ele_feas_train,
		Label_set &labels_train,
		Feature_set &node_features,
		CGAL::Classification::ETHZ::Random_forest_classifier &classifier
	) 
	{
		std::cout << "Start to train on segment features ..." << std::endl;
		const double t_train = omp_get_wtime();

		add_labels(labels_train, enable_joint_labeling);
		int feature_num = 0;
		std::vector<std::string> feature_bank;
		if (enable_joint_labeling)
		{
			add_joint_feature_names
			(
				basic_feas_train,
				segment_eigen_feas_train,
				segment_color_feas_train,
				mulsc_ele_feas_train,
				node_features,
				feature_num,
				feature_bank
			);
		}
		else
		{
			add_feature_names
			(
				basic_feas_train,
				segment_eigen_feas_train,
				segment_color_feas_train,
				mulsc_ele_feas_train,
				node_features,
				feature_num,
				feature_bank
			);
		}

		std::cout << "	- The input number of facets/points/segments for training is " << clusters_train.size() << std::endl;
		std::cout << "	- Training will use " << feature_num << " features." << std::endl;
		std::cout << "	Start training . . ." << std::endl;
		classifier.train(ground_truth_label_train, true, rf_tree_numbers, rf_tree_depth);

		std::cout << "	- Saving trained model -> ";
		std::ofstream fconfig;
		if (processing_mode == 0)
		{
			fconfig = std::ofstream(root_path
				+ folder_names_level_0[3]
				+ "trained_model.gz"
				, std::ios_base::out | std::ios_base::binary);
		}
		else if (processing_mode == 1)
		{
			fconfig = std::ofstream(root_path
				+ folder_names_level_0[8]
				+ sota_folder_path
				+ folder_names_level_0[3]
				+ "trained_model.gz"
				, std::ios_base::out | std::ios_base::binary);
		}
		else if (processing_mode == 2)
		{
			fconfig = std::ofstream(root_path
				+ folder_names_level_0[11]
				+ folder_names_level_0[3]
				+ "trained_model.gz"
				, std::ios_base::out | std::ios_base::binary);
		}

		classifier.save_configuration(fconfig);
		fconfig.close();
		std::cout << "Done !" << std::endl;

		std::cout << "Training consuming time (s): " << omp_get_wtime() - t_train << '\n' << std::endl;
	}

	//----------------------------------------------- Test Module -----------------------------------------------//
	void ETH_RF_Test_Base
	(
		easy3d::PointCloud *pcl_test,
		std::vector<Cluster_point> &clusters_test,
		std::vector<int> &label_indices_test,
		std::vector<int> &ground_truth_label_test,
		std::vector< std::vector<float>> &basic_feas_test,
		std::vector< std::vector<float> > &segment_eigen_feas_test,
		std::vector< std::vector<float> > &segment_color_feas_test,
		std::vector< std::vector<float> > &mulsc_ele_feas_test,
		Label_set &labels_test,
		Feature_set &node_features,
		CGAL::Classification::ETHZ::Random_forest_classifier &classifier,
		std::vector<int> &predict_label,
		std::vector<float> &predict_prob,
		std::vector<std::vector<float>> &predict_prob_all
	)
	{
		std::cout << "Start to test on segment features ..." << std::endl;
		const double t_test = omp_get_wtime();
		add_labels(labels_test, enable_joint_labeling);
		int feature_num = 0;
		std::vector<std::string> feature_bank;
		if (enable_joint_labeling)
		{
			add_joint_feature_names
			(
				basic_feas_test,
				segment_eigen_feas_test,
				segment_color_feas_test,
				mulsc_ele_feas_test,
				node_features,
				feature_num,
				feature_bank
			);
		}
		else
		{
			add_feature_names
			(
				basic_feas_test,
				segment_eigen_feas_test,
				segment_color_feas_test,
				mulsc_ele_feas_test,
				node_features,
				feature_num,
				feature_bank
			);
		}

		//!!!Warning, if use Intel-TBB here, must give following vectors with predefined size and value!!!
		std::vector<float> label_value(clusters_test.size(), -1.0f); // note that this value = prob, votes = prob*num_trees
		std::vector<float> label_prob;
		int label_size = enable_joint_labeling == false ? labels_name.size() : labels_name.size() * 2;
		label_prob = std::vector<float>(clusters_test.size() * label_size, 0.0f);
		predict_prob_all = std::vector<std::vector<float>>(clusters_test.size(), 
			std::vector<float>((label_size), 0.0f));
			
		label_indices_test = std::vector<int>(clusters_test.size(), 0);
		std::cout << "	- The input number of facets/points/segments for testing is " << clusters_test.size() << std::endl;
		std::cout << "	- Testing will use " << feature_num << " features." << std::endl;
		std::cout << "	Start classification . . ." << std::endl;
		CGAL::Classification::classify<Concurrency_tag>(clusters_test, labels_test, classifier, label_indices_test, label_value, label_prob);

		//parsing the results
		int prob_index = 0;
		for (int i = 0; i < clusters_test.size(); ++i)
		{
			predict_label.push_back(label_indices_test[i]);
			predict_prob.push_back(label_value[i]);
			for (int ci = 0; ci < predict_prob_all[i].size(); ++ci)
			{
				predict_prob_all[i][ci] = label_prob[prob_index++];
			}
		}

		//Compute feature importance
		if (save_feature_importance)
		{
			//only need to write once
			save_feature_importance = false;
			std::vector<std::size_t> feas_nodes_count;
			classifier.get_feature_usage(feas_nodes_count);

			std::vector<std::pair<int, int>> feaid_count;
			int total_nodes = std::accumulate(feas_nodes_count.begin(), feas_nodes_count.end(), 0);
			for (int fea_i = 0; fea_i < feas_nodes_count.size(); ++fea_i)
				feaid_count.emplace_back(fea_i, feas_nodes_count[fea_i]);
			sort(feaid_count.begin(), feaid_count.end(), feature_importance_descending);
			for (int fea_k = 0; fea_k < 5; ++fea_k)
				std::cout << feature_bank[feaid_count[fea_k].first] << ": " << feaid_count[fea_k].second << ", "
				<< std::fixed << std::showpoint << std::setprecision(6) << float(feaid_count[fea_k].second) / float(total_nodes)  << std::endl;

			save_txt_feature_importance(feature_bank, feaid_count, total_nodes);
		}

		std::cout << "Testing consuming time (s): " << omp_get_wtime() - t_test << '\n' << std::endl << std::endl;
	}

	void ETH_RF_Test_with_Trained_Model
	(
		easy3d::PointCloud *pcl_test,
		std::vector<Cluster_point> &clusters_test,
		std::vector<int> &label_indices_test,
		std::vector<int> &ground_truth_label_test,
		std::vector< std::vector<float>> &basic_feas_test,
		std::vector< std::vector<float> > &segment_eigen_feas_test,
		std::vector< std::vector<float> > &segment_color_feas_test,
		std::vector< std::vector<float> > &mulsc_ele_feas_test,
		Label_set &labels_test,
		std::vector<int> &predict_label,
		std::vector<float> &predict_prob,
		std::vector<std::vector<float>> &predict_prob_all,
		Feature_set &node_features,
		CGAL::Classification::ETHZ::Random_forest_classifier &eth_rf_classifier
	)
	{
		ETH_RF_Test_Base
		(
			pcl_test,
			clusters_test,
			label_indices_test,
			ground_truth_label_test,
			basic_feas_test, 
			segment_eigen_feas_test, segment_color_feas_test,
			mulsc_ele_feas_test,
			labels_test,
			node_features,
			eth_rf_classifier,
			predict_label,
			predict_prob,
			predict_prob_all
		);
	}

	//----------------------------------------------- Evaluation Module -----------------------------------------------//
	void parsing_properties_from_classification
	(
		SFMesh *smesh_in,
		std::vector< std::vector<int>> &seg_face_vec,
		std::vector<int> &face_predict_label,
		std::vector<int> &face_truth_label,
		std::vector<float> &face_area_weighted,
		std::vector<int> &predict_label,
		std::vector<float> &predict_prob,
		std::vector<std::vector<float>> &predict_prob_all
	)
	{
		if (smesh_in->get_face_property<vec3>("f:color"))
			smesh_in->get_face_color = smesh_in->get_face_property<vec3>("f:color");
		else
		{
			smesh_in->add_face_property<vec3>("f:color", vec3(0.0f, 0.0f, 0.0f));
			smesh_in->get_face_color = smesh_in->get_face_property<vec3>("f:color");
		}

		smesh_in->add_face_property<bool>("f:is_ignored", false);
		auto is_face_ignored = smesh_in->get_face_property<bool>("f:is_ignored");

		if (!enable_joint_labeling)
		{
			//parsing to face label and probability
			for (int i = 0; i < seg_face_vec.size(); ++i)
			{
				for (int j = 0; j < seg_face_vec[i].size(); ++j)
				{
					SFMesh::Face fd(seg_face_vec[i][j]);
					if (smesh_in->get_face_segment_id[fd] == -1)
						smesh_in->get_face_segment_id[fd] = i;

					if (ignored_labels_name.empty())
					{
						smesh_in->get_face_predict_label[fd] = predict_label[i] + 1;
						smesh_in->get_face_predict_prob[fd] = predict_prob[i];
						smesh_in->get_face_all_predict_prob[fd] = predict_prob_all[i];
						smesh_in->get_face_color[fd] = labels_color[predict_label[i]];
					}
					else
					{
						if (smesh_in->get_face_truth_label[fd] != 0 && smesh_in->get_face_truth_label[fd] != -1)
						{
							bool is_ignore = false;
							for (int ig_i = 0; ig_i < ignored_labels_name.size(); ++ig_i)
							{
								if (ignored_labels_name[ig_i] == labels_name[smesh_in->get_face_truth_label[fd] - 1])
								{
									is_ignore = true;
									break;
								}
							}

							std::vector<float> predict_prob_all_full(labels_name.size(), 0.0f);
							if (!is_ignore)
							{
								for (int pi = 0; pi < predict_prob_all[i].size(); ++pi)
								{
									predict_prob_all_full[pi + new_label_shiftback[pi]] = predict_prob_all[i][pi];
								}

								smesh_in->get_face_predict_label[fd] = predict_label[i] + new_label_shiftback[predict_label[i]] + 1;
								smesh_in->get_face_predict_prob[fd] = predict_prob[i];
								smesh_in->get_face_all_predict_prob[fd] = predict_prob_all_full;
								smesh_in->get_face_color[fd] = labels_color[predict_label[i] + new_label_shiftback[predict_label[i]]];
							}
							else
							{
								is_face_ignored[fd] = true;
								predict_prob_all_full[smesh_in->get_face_truth_label[fd] - 1] = 1.0f;

								smesh_in->get_face_predict_label[fd] = smesh_in->get_face_truth_label[fd];
								smesh_in->get_face_predict_prob[fd] = 1.0f;
								smesh_in->get_face_all_predict_prob[fd] = predict_prob_all_full;
								smesh_in->get_face_color[fd] = labels_color[smesh_in->get_face_truth_label[fd] - 1];
							}
						}
					}
				}
			}
		}

		for (auto fd : smesh_in->faces())
		{
			if (smesh_in->get_face_all_predict_prob[fd].empty())
				smesh_in->get_face_all_predict_prob[fd] = std::vector<float>(labels_name.size(), 0.0f);

			if (train_test_predict_val != 0 && train_test_predict_val != 2)
			{
				if (smesh_in->get_face_truth_label[fd] != 0 && smesh_in->get_face_truth_label[fd] != -1)
				{
					if (ignored_labels_name.empty())
					{
						face_area_weighted.push_back(smesh_in->get_face_area[fd]);
						face_predict_label.push_back(smesh_in->get_face_predict_label[fd] - 1);
						face_truth_label.push_back(smesh_in->get_face_truth_label[fd] - 1);
					}
					else
					{
						if (!is_face_ignored[fd])
						{
							int new_pred_label = smesh_in->get_face_predict_label[fd] - 1 - label_shiftdis[smesh_in->get_face_predict_label[fd] - 1];
							int new_truth_label = smesh_in->get_face_truth_label[fd] - 1 - label_shiftdis[smesh_in->get_face_truth_label[fd] - 1];
							face_area_weighted.push_back(smesh_in->get_face_area[fd]);
							face_predict_label.push_back(new_pred_label);
							face_truth_label.push_back(new_truth_label);
						}
					}
				}
			}
			else if (train_test_predict_val == 2)
			{
				int new_pred_label = smesh_in->get_face_predict_label[fd] - 1 - label_shiftdis[smesh_in->get_face_predict_label[fd] - 1];
				face_predict_label.push_back(new_pred_label);
				face_area_weighted.push_back(smesh_in->get_face_area[fd]);
			}
		}

		smesh_in->remove_face_property(is_face_ignored);
	}

	void ETH_RF_SavingTest_and_or_Evaluation
	(
		SFMesh *smesh_in,
		Label_set &labels,
		std::vector<int> &face_predict_label,
		std::vector<int> &face_truth_label,
		std::vector<float> &face_area_weighted,
		const int m
	)
	{
		if (train_test_predict_val == 1 || train_test_predict_val == 3)
		{
			if (enable_joint_labeling)
			{
				//clear joint labels
				labels.clear();
				//reset label to normal size
				add_labels(labels, false);
			}

			//Evaluation based on each face, not on segment!!!
			std::cerr << "Precision, recall, F1 scores and IoU:" << std::endl;
			CGAL::Classification::Evaluation evaluation(labels, face_truth_label, face_predict_label, face_area_weighted);
			for (std::size_t i = 0; i < labels.size(); ++i)
			{
				std::cerr << " * " << labels[i]->name() << ": "
					<< evaluation.precision(labels[i]) << " ; "
					<< evaluation.recall(labels[i]) << " ; "
					<< evaluation.f1_score(labels[i]) << " ; "
					<< evaluation.intersection_over_union(labels[i]) << std::endl;
			}
			std::cerr << "Mean Accuracy = " << evaluation.mean_accuracy() << std::endl
				<< "Overall Accuracy = " << evaluation.accuracy() << std::endl
				<< "mean F1 score = " << evaluation.mean_f1_score() << std::endl
				<< "mean IoU = " << evaluation.mean_intersection_over_union() << std::endl;

			std::ostringstream evaluation_out;
			if (use_batch_processing)
			{
				evaluation_out
					<< root_path
					<< folder_names_level_0[0]
					<< folder_names_level_1[train_test_predict_val]
					<< prefixs[10] + std::to_string(m)
					<< prefixs[6]
					<< ".txt";
			}
			else
			{
				if (processing_mode == 0)
				{
					evaluation_out
						<< root_path
						<< folder_names_level_0[0]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[m]
						<< prefixs[6]
						<< ".txt";
				}
				else if (processing_mode == 1)
				{
					evaluation_out
						<< root_path
						<< folder_names_level_0[8]
						<< sota_folder_path
						<< folder_names_level_0[0]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[m]
						<< prefixs[6]
						<< ".txt";
				}
				else if (processing_mode == 2)
				{
					evaluation_out
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_level_0[0]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[m]
						<< prefixs[6]
						<< ".txt";
				}
			}

			save_txt_evaluation(labels, evaluation, evaluation_out, m);

		}
		
		write_semantic_mesh_data(smesh_in, m);
	}

	void evaluation_all_test_data
	(
		Label_set &labels, 
		std::vector<int> &face_truth_label,
		std::vector<int> &face_test_label,
		std::vector<float> &face_area_weighted,
		const int m
	)
	{
		//Evaluation based on each face, not on segment!!!
		std::cerr << "Precision, recall, F1 scores and IoU:" << std::endl;
		CGAL::Classification::Evaluation evaluation(labels, face_truth_label, face_test_label, face_area_weighted);
		for (std::size_t i = 0; i < labels.size(); ++i)
		{
			std::cerr << " * " << labels[i]->name() << ": "
				<< evaluation.precision(labels[i]) << " ; "
				<< evaluation.recall(labels[i]) << " ; "
				<< evaluation.f1_score(labels[i]) << " ; "
				<< evaluation.intersection_over_union(labels[i]) << std::endl;
		}
		std::cerr << "Mean Accuracy = " << evaluation.mean_accuracy() << std::endl
			<< "Overall Accuracy = " << evaluation.accuracy() << std::endl
			<< "mean F1 score = " << evaluation.mean_f1_score() << std::endl
			<< "mean IoU = " << evaluation.mean_intersection_over_union() << std::endl;

		std::ostringstream evaluation_out;
		std::string basic_write_path, pref_tmp;
		if (processing_mode == 0) //RF
		{
			std::cout << "writing RF test evaluation " << std::endl;
			basic_write_path = root_path + folder_names_level_0[0] + folder_names_level_1[train_test_predict_val];
			pref_tmp = "evaluation";
		}
		else if (processing_mode == 1 && sota_folder_path != "PSSNet/") //sota
		{
			std::cout << "writing SOTA test evaluation " << std::endl;
			basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[0] + folder_names_level_1[train_test_predict_val];
			pref_tmp = "evaluation";
		}
		else if (processing_mode == 2 || sota_folder_path == "PSSNet/") //PSSNet
		{
			if (previous_mode != operating_mode::Process_semantic_pcl)
			{
				std::cout << "writing PSSNet step-1 test evaluation " << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[0] + folder_names_level_1[train_test_predict_val];
				pref_tmp = "evaluation_pssnet_step_1";
			}
			else
			{
				std::cout << "writing PSSNet step-2 test evaluation " << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[0] + folder_names_level_1[train_test_predict_val];
				pref_tmp = "evaluation_pssnet_step_2";
			}
		}

		if (use_batch_processing)
		{
			evaluation_out
				<< basic_write_path
				<< prefixs[9]
				<< pref_tmp
				<< "_all_test.txt";
		}
		else
		{
			if (m > -1)
			{
				evaluation_out
					<< basic_write_path
					<< base_names[m]
					<< pref_tmp
					<< ".txt";
			}
			else
			{
				evaluation_out
					<< basic_write_path
					<< pref_tmp
					<< "_all_test.txt";
			}
		}

		save_txt_evaluation(labels, evaluation, evaluation_out, -1);
	}
	
	//----------------------------------------------- Joint energy computation -----------------------------------------//
	void joint_labeling_energy //for test data only
	(
		std::map<std::pair<int, int>, std::pair<float, bool>> &spf_edges_maps,
		std::vector<int> &predict_label,
		std::vector<float> &predict_prob,
		std::vector<std::vector<float>> &predict_prob_all,
		std::map<int, std::tuple<int, int, std::vector<float>>> &unary_label_prob,//sites, (pred_label, count joint labels, accum labels_probs)
		std::map<std::pair<int, int>, float> &pairwise_label_prob //neighbors, prob
	)
	{
		std::map<std::pair<int, int>, std::pair<float, bool>> spf_edges_maps_clone;
		spf_edges_maps_clone.insert(spf_edges_maps.begin(), spf_edges_maps.end());
		std::map<std::pair<int, int>, std::pair<float, bool>>::iterator m_it = spf_edges_maps_clone.begin();
		std::map<std::pair<int, int>, std::pair<float, bool>>::iterator m_it_end = spf_edges_maps_clone.end();

		//compute unary and pairwise energy 
		int seg_join_i = 0;
		for (; m_it != m_it_end; ++m_it)
		{
			if (m_it->second.second == false)
			{
				int s_ind = m_it->first.first;
				int t_ind = m_it->first.second;
				m_it->second.second = true;
				spf_edges_maps_clone[std::make_pair(t_ind, s_ind)].second = true;

				int label_s = predict_label[seg_join_i] > labels_name.size() ? predict_label[seg_join_i] - labels_name.size() : predict_label[seg_join_i];
				float prob_s = predict_prob[seg_join_i];
				unary_accum(predict_label, predict_prob_all, unary_label_prob, seg_join_i++, s_ind);
				int label_t = predict_label[seg_join_i] > labels_name.size() ? predict_label[seg_join_i] - labels_name.size() : predict_label[seg_join_i];
				float prob_t = predict_prob[seg_join_i];
				unary_accum(predict_label, predict_prob_all, unary_label_prob, seg_join_i++, t_ind);

				if (label_s != label_t)
					pairwise_label_prob[std::make_pair(s_ind, t_ind)] = 0.5f * (prob_s + prob_t) * m_it->second.first; //prob * common length, larger value means not similar 
				else
					pairwise_label_prob[std::make_pair(s_ind, t_ind)] = 0.0f;
			}
		}
	}

	//----------------------------------------------- Refinement Module -----------------------------------------------//
	using namespace gco;
	void MRF_joint_labeling
	(
		SFMesh* smesh_in,
		std::vector< std::vector<int>> &seg_face_vec,
		std::map<int, std::tuple<int, int, std::vector<float>>> &unary_label_prob,//sites, (labels, count joint labels, accum labels_probs)
		std::map<std::pair<int, int>, float> &pairwise_label_prob, //neighbors, prob
		const int m
	)
	{
		std::cout << "Start MRF refinement ..." << std::endl;
		const double t_total = omp_get_wtime();
		try
		{
			GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(smesh_in->faces_size(), labels_name.size());
			//set unary terms
			int num_labels = labels_name.size();
			std::map<int, std::tuple<int, int, std::vector<float>>>::iterator u_it = unary_label_prob.begin();
			std::map<int, std::tuple<int, int, std::vector<float>>>::iterator u_it_end = unary_label_prob.end();
			for (; u_it != u_it_end; ++u_it)
				for (int c = 0; c < num_labels; ++c)
					gc->setDataCost(u_it->first, c, (1.0f - get<2>(u_it->second)[c] / float(get<1>(u_it->second))) * mrf_energy_amplify); //full cost

			//set pairwise terms(smooth cost)
			int *smooth = new int[num_labels*num_labels];
			for (auto l1 = 0; l1 < num_labels; ++l1)
			{
				for (auto l2 = 0; l2 < num_labels; ++l2)
				{
					if (l1 != l2)
					{
						smooth[l1 + l2 * num_labels] = 1;
					}
					else
					{
						smooth[l1 + l2 * num_labels] = 0;
					}
				}
			}
			gc->setSmoothCost(smooth);

			//set up neighbors
			std::map<std::pair<int, int>, float>::iterator p_it = pairwise_label_prob.begin();
			std::map<std::pair<int, int>, float>::iterator p_it_end = pairwise_label_prob.end();
			for (; p_it != p_it_end; ++p_it)
			{
				//smoothness term
				gc->setNeighbors(p_it->first.first, p_it->first.second, p_it->second * mrf_lambda_mh * mrf_energy_amplify);
			}

			//solving energy
			printf("\nBefore optimization energy is %d", gc->compute_energy());  //TODO(wgao) bad type: int vs. long long (check your warnings)!
			gc->expansion(99);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			printf("\nAfter optimization energy is %d", gc->compute_energy());
			std::cout << std::endl << std::endl;

			//Energy check warning
			if (gc->compute_energy() <= 0)
			{
				std::cout << " ************** Warning !!! ************** " << std::endl;
				std::cout << " Energy is negative, please check energy terms!!!" << std::endl;
			}

			//Update face labels
			u_it = unary_label_prob.begin();
			for (; u_it != u_it_end; ++u_it)
			{
				int label_current = gc->whatLabel(u_it->first);
				for (auto fi : seg_face_vec[u_it->first])
				{
					SFMesh::Face fd(fi);
					smesh_in->get_face_predict_label[fd] = label_current + 1;
					smesh_in->get_face_color[fd] = labels_color[label_current];
				}
			}

			std::cout << "Processing time " << omp_get_wtime() - t_total << " (s)" << std::endl;
			delete[]smooth;
			delete gc;
		}
		catch (GCException e)
		{
			e.Report();
		}
	}

	//----------------------------------------------- Feature Diversity Module -----------------------------------------------//
	float feature_variances_computation
	(
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> & basic_feas,
		std::vector< std::vector<float> > &segment_eigen_feas,
		std::vector< std::vector<float> > &segment_color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas
	)
	{
		int sc_2_fea_size = multi_scale_ele_radius.size();

		std::pair<int, float> feanum_var(0, 0.0f);
		if (use_feas[0] == true)
		{
			for (int fi = 0; fi < basic_feas[0].size(); ++fi)
			{
				if (use_basic_features[fi] == true)
				{
					std::vector<float> tmp_vec(seg_truth.size(), 0);
					for (int seg_i = 0; seg_i < seg_truth.size(); ++seg_i)
						tmp_vec[seg_i] = basic_feas[seg_i][fi];
					feanum_var.second += compute_variance_of_input(tmp_vec);
					++feanum_var.first;
				}
			}
		}
		//controlled by muti scale2 elevations 
		std::vector<std::pair<std::string, int>> temp_ele_feature_base_names;
		if (!multi_scale_ele_radius.empty())
		{
			for (int sci = 0; sci < sc_2_fea_size; ++sci)
			{
				temp_ele_feature_base_names.push_back(std::pair<std::string, int>("multiscale_2_ele_fea_" + std::to_string(sci), sci));
			}
		}

		if (use_feas[3] == true)
		{
			for (int fi = 0; fi < mulsc_ele_feas[0].size(); ++fi)
			{
				if (use_mulsc_eles[fi] == true)
				{
					std::vector<float> tmp_vec(seg_truth.size(), 0);
					for (int seg_i = 0; seg_i < seg_truth.size(); ++seg_i)
						tmp_vec[seg_i] = mulsc_ele_feas[seg_i][fi];
					feanum_var.second += compute_variance_of_input(tmp_vec);
					++feanum_var.first;
				}
			}
		}

		for (int fi = 0; fi < segment_eigen_feas[0].size(); ++fi)
		{
			if (use_feas[1] == true && use_eigen_features[fi] == true)
			{
				std::vector<float> tmp_vec(seg_truth.size(), 0);
				for (int seg_i = 0; seg_i < seg_truth.size(); ++seg_i)
					tmp_vec[seg_i] = segment_eigen_feas[seg_i][fi];
				feanum_var.second += compute_variance_of_input(tmp_vec);
				++feanum_var.first;
			}
		}

		for (int fi = 0; fi < segment_color_feas[0].size(); ++fi)
		{
			if (use_feas[2] == true && use_color_features[fi] == true)
			{
				std::vector<float> tmp_vec(seg_truth.size(), 0);
				for (int seg_i = 0; seg_i < seg_truth.size(); ++seg_i)
					tmp_vec[seg_i] = segment_color_feas[seg_i][fi];
				feanum_var.second += compute_variance_of_input(tmp_vec);
				++feanum_var.first;
			}
		}

		std::cout << "	- Use " << feanum_var.first << " features." << std::endl;
		return feanum_var.second / float(feanum_var.first);
	}

	//----------------------------------------------- PSSNet graph-cut Module -----------------------------------------------//
	void MRF_oversegmentation
	(
		std::vector<float> &unary_terms,
		std::vector<std::pair<int, int>> &pairwise_neighbors,
		std::vector<float> &pairwise_terms,
		std::vector<int> &label_out
	)
	{
		bool results = false;
		int num_segments = unary_terms.size();
		int num_labels = 2;//0: same; 1: different
		try
		{
			gco::GCoptimizationGeneralGraph *gc = new gco::GCoptimizationGeneralGraph(num_segments, num_labels);
			for (int i = 0; i < unary_terms.size(); ++i)
			{
				gc->setDataCost(i, 0, unary_terms[i] * 10E1); //check neighbor segment belong to 0 or not, unary = prob;
				gc->setDataCost(i, 1, (1.0f - unary_terms[i]) * 10E1);
			}

			//set smooth cost
			int *smooth = new int[num_labels*num_labels];
			for (auto l1 = 0; l1 < num_labels; ++l1)
			{
				for (auto l2 = 0; l2 < num_labels; ++l2)
				{
					if (l1 != l2)
					{
						smooth[l1 + l2 * num_labels] = 1;
					}
					else
					{
						smooth[l1 + l2 * num_labels] = 0;
					}
				}
			}
			gc->setSmoothCost(smooth);

			//set up neighbors
			for (int i = 0; i < pairwise_neighbors.size(); ++i)
			{
				gc->setNeighbors(pairwise_neighbors[i].first, pairwise_neighbors[i].second, mrf_lambda_m * pairwise_terms[i] * 10E1);
			}

			//printf("\nBefore optimization energy is %d", gc->compute_energy());  //TODO(wgao) bad type: int vs. long long (check your warnings)!
			gc->expansion(99);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			//printf("\nAfter optimization energy is %d", gc->compute_energy());

			label_out.resize(unary_terms.size());
			for (int i = 0; i < label_out.size(); ++i)
			{
				label_out[i] = gc->whatLabel(i);
			}

			delete[]smooth;
			delete gc;
		}
		catch (gco::GCException e)
		{
			e.Report();
		}
	}

	//For global smooth for faces after planar and non-planar decomposition
	//For global smooth for faces after watershed segmentation
	void MRF_oversegmentation
	(
		SFMesh *smesh_in,
		SFMesh::Face &fi,
		std::vector<int> &labels_temp,
		const int num_segments,
		int &change_count
	)
	{
		try
		{
			GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_segments, labels_temp.size());
			int seg_i = 0;
			for (int li = 0; li < labels_temp.size(); ++li)
			{
				if (labels_temp[li] == smesh_in->get_face_smoothed_seg_id[fi])
				{
					gc->setDataCost(seg_i, li, 0.0f * mrf_energy_amplify);
				}
				else
				{
					gc->setDataCost(seg_i, li, 1.0f * mrf_energy_amplify);
				}
			}
			++seg_i;
			for (auto f_neg_tup : smesh_in->get_face_1ring_neighbor[fi])
			{
				for (int li = 0; li < labels_temp.size(); ++li)
				{
					SFMesh::Face fdx(get<0>(f_neg_tup));
					if (labels_temp[li] == smesh_in->get_face_smoothed_seg_id[fdx])
					{
						gc->setDataCost(seg_i, li, 0.0f * mrf_energy_amplify);
					}
					else
					{
						gc->setDataCost(seg_i, li, 1.0f * mrf_energy_amplify);
					}
				}
				++seg_i;
			}

			//set smooth cost
			int *smooth = new int[labels_temp.size()*labels_temp.size()];
			for (auto l1 = 0; l1 < labels_temp.size(); ++l1)
			{
				for (auto l2 = 0; l2 < labels_temp.size(); ++l2)
				{
					if (l1 != l2)
					{
						smooth[l1 + l2 * labels_temp.size()] = 1;
					}
					else
					{
						smooth[l1 + l2 * labels_temp.size()] = 0;
					}
				}
			}
			gc->setSmoothCost(smooth);

			seg_i = 1;
			float pairwise = 1.0f, angle_current;
			SFMesh::Face fd;
			for (auto f_neg_tup : smesh_in->get_face_1ring_neighbor[fi])
			{
				fd = SFMesh::Face(get<0>(f_neg_tup));
				if (smesh_in->is_boundary(fd) || smesh_in->is_boundary(fi))
				{
					pairwise = mrf_lambda_p * mrf_energy_amplify;
				}
				else
				{
					int current_label_fd = -1;
					if (train_test_predict_val != 0)
					{
						current_label_fd = smesh_in->get_face_predict_label[fi] - 1;
					}
					else
					{
						current_label_fd = smesh_in->get_face_truth_label[fi] - 1;
					}

					if (labels_name_pnp[current_label_fd] == "non_planar")
					{
						pairwise = mrf_lambda_p * mrf_energy_amplify;
					}
					else
					{
						angle_current = vector3D_angle(smesh_in->get_face_planar_segment_plane_normal[fd], smesh_in->get_face_planar_segment_plane_normal[fi]);// ,  smesh_out->get_points_normals[non_vert]
						if (angle_current >= 180.0f)
							angle_current = angle_current - 180.0f;
						if (angle_current > 90.0f)
							angle_current = 180.0f - angle_current;
						angle_current = 1.0f - angle_current / 90.0f;

						pairwise = mrf_lambda_p * angle_current * mrf_energy_amplify;
					}
				}

				gc->setNeighbors(0, seg_i, pairwise);
				++seg_i;
			}

			//printf("\nBefore optimization energy is %d", gc->compute_energy());  //TODO(wgao) bad type: int vs. long long (check your warnings)!
			gc->expansion(99);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			//printf("\nAfter optimization energy is %d", gc->compute_energy());


			if (smesh_in->get_face_smoothed_seg_id[fi] != labels_temp[gc->whatLabel(0)])
			{
				smesh_in->get_face_smoothed_seg_id[fi] = labels_temp[gc->whatLabel(0)];
				++change_count;
			}

			delete[]smooth;
			delete gc;
		}
		catch (GCException e)
		{
			e.Report();
		}
	}

	//purity evaluation
	void segment_purity_evaluation
	(
		SFMesh *smesh_g,//ground truth
		SFMesh *smesh_t, //test data
		std::map<int, float> &label_asa,
		std::map<int, float> &label_ue,
		std::map<int, float> &label_sumarea,
		std::vector<int> &t_ind_vec,
		std::vector<int> &g_ind_vec
	)
	{
		std::map<int, int> component_g_label;
		std::map<int, float> component_area_t;//test component area
		std::map<int, float> component_area_g;//ground component area
		std::map<int, float> componentid_ue_g;//ground component Under-segmentation error
		std::map<int, float> componentid_asa_t;//test component Achievable segmentation accuracy(ASA)
		std::map<int, std::vector<int>> t_included_g_ind;//test id, intersected ground truth id
		std::map<int, std::vector<int>> g_included_t_ind;//ground truth id, intersected test id
		std::map<int, std::map<int, float>> t_included_g_area;//test segment id, intersected ground truth component id and area	map
		std::map<int, std::map<int, float>> g_included_t_area;//ground truth segment id, intersected ground truth component id and area	ma
		for (auto fg : smesh_g->faces())
		{
			//get ground truth area and test component ids
			//label_intersected_area equal to ground truth area of each class
			//because all the ground truth area is covered by test region
			int label_g = smesh_g->get_face_truth_label[fg];
			if (label_g == -1)
				continue;
			else
				--label_g;

			label_sumarea[label_g] += smesh_g->get_face_area[fg];

			int t_ind = smesh_t->get_face_segment_id[fg];
			int g_ind = smesh_g->get_face_segment_id[fg];
			t_ind_vec.push_back(t_ind);
			g_ind_vec.push_back(g_ind);

			//get initial label miou
			auto it_l = label_asa.find(label_g);
			if (it_l == label_asa.end())
			{
				label_asa[label_g] = 0.0f;
				label_ue[label_g] = 0.0f;
			}

			//get ground truth component semantic label
			auto it_g = component_g_label.find(g_ind);
			if (it_g == component_g_label.end())
			{
				component_area_g[g_ind] = smesh_t->get_face_area[fg];
				component_g_label[g_ind] = label_g;
				std::vector<int> t_ind_vec_temp;
				t_ind_vec_temp.push_back(t_ind);
				g_included_t_ind[g_ind] = t_ind_vec_temp;
				std::map<int, float> t_area;
				t_area[t_ind] = smesh_t->get_face_area[fg];
				g_included_t_area[t_ind] = t_area;
				componentid_ue_g[g_ind] = 0.0f;
			}
			else
			{
				component_area_g[g_ind] += smesh_t->get_face_area[fg];
				g_included_t_ind[g_ind].push_back(t_ind);
				g_included_t_area[g_ind][t_ind] += smesh_t->get_face_area[fg];
			}

			//compute test component area and get intersected truth count
			auto it_t = component_area_t.find(t_ind);
			if (it_t == component_area_t.end())
			{
				component_area_t[t_ind] = smesh_t->get_face_area[fg];
				std::vector<int> g_ind_vec_temp;
				g_ind_vec_temp.push_back(g_ind);
				t_included_g_ind[t_ind] = g_ind_vec_temp;
				std::map<int, float> g_area;
				g_area[g_ind] = smesh_t->get_face_area[fg];
				t_included_g_area[t_ind] = g_area;
				componentid_asa_t[t_ind] = -1.0f;
			}
			else
			{
				component_area_t[t_ind] += smesh_t->get_face_area[fg];
				t_included_g_ind[t_ind].push_back(g_ind);
				t_included_g_area[t_ind][g_ind] += smesh_t->get_face_area[fg];
			}
		}
		sort(t_ind_vec.begin(), t_ind_vec.end());
		t_ind_vec.erase(unique(t_ind_vec.begin(), t_ind_vec.end()), t_ind_vec.end());
		sort(g_ind_vec.begin(), g_ind_vec.end());
		g_ind_vec.erase(unique(g_ind_vec.begin(), g_ind_vec.end()), g_ind_vec.end());

		//compute on ground truth segments
		for (int g_ci = 0; g_ci < g_ind_vec.size(); ++g_ci)
		{
			int g_ind = g_ind_vec[g_ci];
			float area_intersected_max = -FLT_MAX;
			for (int t_ci = 0; t_ci < g_included_t_ind[g_ind].size(); ++t_ci)
			{
				int t_id = g_included_t_ind[g_ind][t_ci];
				if (t_included_g_ind[t_id].size() > 1)
				{
					float common_area = t_included_g_area[t_id][g_ind];
					componentid_ue_g[g_ind] += component_area_t[t_id] - common_area;
				}
			}
			label_ue[component_g_label[g_ind]] += componentid_ue_g[g_ind];
		}

		//compute on test segments
		for (int t_ci = 0; t_ci < t_ind_vec.size(); ++t_ci)
		{
			int t_ind = t_ind_vec[t_ci];
			int label_dominant = -1;
			float area_intersected_max = -FLT_MAX;
			for (int g_ci = 0; g_ci < t_included_g_ind[t_ind].size(); ++g_ci)
			{
				int g_id = t_included_g_ind[t_ind][g_ci];
				if (area_intersected_max < t_included_g_area[t_ind][g_id])
				{
					area_intersected_max = t_included_g_area[t_ind][g_id];
					label_dominant = component_g_label[g_id];
				}
			}

			componentid_asa_t[t_ind] = area_intersected_max;
			label_asa[label_dominant] += componentid_asa_t[t_ind];
			//label_ue[label_dominant] += componentid_ue_t[t_ind];
		}
	}

	//boundary evalution
	void boundary_evaluation
	(
		SFMesh *smesh_g,//ground truth
		SFMesh *smesh_t, //test data
		float &boundary_num_t,
		float &boundary_num_g,
		float &intersect_edges,
		int &n_ring
	)
	{
		std::map<int, std::vector<int>> edge_g_neg;
		std::vector<int> edge_vec_g;
		for (auto edx : smesh_g->edges())
		{
			SFMesh::Halfedge h0 = smesh_g->halfedge(edx, 0);
			SFMesh::Halfedge h1 = smesh_g->halfedge(edx, 1);

			SFMesh::Face f0(0), f1(0);
			int f0_id_truth = -1, f1_id_truth = -1, f0_id_test = -2, f1_id_test = -2;
			if (!smesh_g->is_boundary(h0))
			{
				f0 = smesh_g->face(h0);
				f0_id_truth = smesh_g->get_face_segment_id[f0];
				f0_id_test = smesh_t->get_face_segment_id[f0];
			}
			if (!smesh_g->is_boundary(h1))
			{
				f1 = smesh_g->face(h1);
				f1_id_truth = smesh_g->get_face_segment_id[f1];
				f1_id_test = smesh_t->get_face_segment_id[f1];
			}

			SFMesh::Vertex vs = smesh_t->vertex(edx, 0);
			SFMesh::Vertex vt = smesh_t->vertex(edx, 1);
			float tmp_length = (smesh_t->get_points_coord[vt] - smesh_t->get_points_coord[vs]).length();
			if (f0_id_test != f1_id_test)
			{
				smesh_t->get_edge_boundary_predict[edx] = 1;
				SFMesh::Vertex vs = smesh_t->vertex(edx, 0);
				SFMesh::Vertex vt = smesh_t->vertex(edx, 1);
				boundary_num_t += tmp_length;
			}

			if (f0_id_truth != f1_id_truth)
			{
				smesh_g->get_edge_boundary_truth[edx] = 1;
				edge_vec_g.push_back(edx.idx());
				boundary_num_g += tmp_length;
			}
			else
				continue;

			//find n rings neighbor of ground truth edges
			std::map<int, bool> checked_neighbor;
			edge_g_neg[edx.idx()] = std::vector<int>();
			if (n_ring != 0)
				find_n_rings_neighbor_of_vertex(smesh_g, edx, n_ring, edge_g_neg[edx.idx()], checked_neighbor);
		}

		//evaluation of boundary
		for (int ei_g = 0; ei_g < edge_vec_g.size(); ++ei_g)
		{
			//check exactly overlap intersection
			int g_eid = edge_vec_g[ei_g];
			SFMesh::Edge edx_g(g_eid);
			SFMesh::Vertex vs_g = smesh_t->vertex(edx_g, 0);
			SFMesh::Vertex vt_g = smesh_t->vertex(edx_g, 1);
			float tmp_length = (smesh_t->get_points_coord[vt_g] - smesh_t->get_points_coord[vs_g]).length();
			if (smesh_t->get_edge_boundary_predict[edx_g] == 1)
			{
				intersect_edges += tmp_length;
				continue;
			}

			//check intersection over n-ring neighbors
			for (int ei_t = 0; ei_t < edge_g_neg[g_eid].size(); ++ei_t)
			{
				int t_eid = edge_g_neg[g_eid][ei_t];
				SFMesh::Edge edx_t(t_eid);
				SFMesh::Vertex vs_n = smesh_t->vertex(edx_t, 0);
				SFMesh::Vertex vt_n = smesh_t->vertex(edx_t, 1);
				if (smesh_t->get_edge_boundary_predict[edx_t] == 1 && t_eid != g_eid)
				{
					intersect_edges += tmp_length;
					break;
				}
			}
		}

	}

	//
	void oversegmentation_evaluation
	(
		SFMesh *smesh_g,//ground truth
		SFMesh *smesh_t, //test data
		const int mi,
		all_eval *all_seg_evaluation
	)
	{
		//-------------Segment purity evaluation-------------//
		//ground truth dominant segment id, intersected area, truth segment total area  
		std::map<int, float> label_asa, label_ue, label_sumarea;
		std::vector<int> t_ind_vec, g_ind_vec;//valid test and ground truth data, not include unknown area
		segment_purity_evaluation
		(
			smesh_g,
			smesh_t,
			label_asa,
			label_ue,
			label_sumarea,
			t_ind_vec,
			g_ind_vec
		);
		std::vector<float> asa_out(labels_name_pnp.size() + 1, 0.0f);
		compute_asa(all_seg_evaluation, asa_out, label_asa, label_ue, label_sumarea);

		std::vector<float> gc_tc_out(3, 0.0f);
		int test_count = t_ind_vec.size(), ground_count = g_ind_vec.size();
		compute_gc_tc(test_count, ground_count, gc_tc_out, all_seg_evaluation);

		//-------------Boundary evaluation-------------//
		float boundary_num_t = 0.0f, boundary_num_g = 0.0f, intersect_edges = 0.0f;
		int n_rings = 2;
		boundary_evaluation
		(
			smesh_g,//ground truth
			smesh_t, //test data
			boundary_num_t,
			boundary_num_g,
			intersect_edges,
			n_rings
		);

		std::vector<float> br_out(5, 0.0f);
		compute_br(boundary_num_g, boundary_num_t, intersect_edges, br_out, all_seg_evaluation);

		std::ostringstream evaluation_out;
		evaluation_out
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_level_0[0]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi] + "_overseg"
			<< prefixs[6]
			<< ".txt";

		save_txt_evaluation(asa_out, gc_tc_out, br_out, evaluation_out, mi);

		int last_tile = base_names.size() - 1;
		if (mi == last_tile)
		{
			compute_asa(all_seg_evaluation);
			test_count = all_seg_evaluation->segment_count_gctc[0];
			ground_count = all_seg_evaluation->segment_count_gctc[1];
			compute_gc_tc(test_count, ground_count, gc_tc_out);
			//compute_br(boundary_num_g, boundary_num_t, intersect_edges, br_out);

			std::cout << "Ground truth edges = " << boundary_num_g << std::endl <<
				";Predicted edges = " << boundary_num_t << std::endl <<
				";Intersected edges = " << intersect_edges << std::endl <<
				"BR = " << all_seg_evaluation->boundary_evaluation[3] <<
				";\tBP = " << all_seg_evaluation->boundary_evaluation[4] << std::endl;

			std::ostringstream evaluation_all;
			evaluation_all
				<< root_path
				<< folder_names_level_0[11]
				<< folder_names_level_0[0]
				<< partition_folder_path
				<< "all_seg"
				<< prefixs[6]
				<< ".txt";

			save_txt_evaluation
			(
				all_seg_evaluation->asa_out,
				all_seg_evaluation->segment_count_gctc,
				all_seg_evaluation->boundary_evaluation,
				evaluation_all, mi
			);
		}
	}
}