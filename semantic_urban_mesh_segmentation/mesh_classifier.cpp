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
		else
		{
			fconfig = std::ofstream(root_path
				+ folder_names_level_0[8]
				+ sota_folder_path
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
				else
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
		else if (processing_mode == 1) //sota
		{
			std::cout << "writing SOTA test evaluation " << std::endl;
			basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[0] + folder_names_level_1[train_test_predict_val];
			pref_tmp = "evaluation";
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
}