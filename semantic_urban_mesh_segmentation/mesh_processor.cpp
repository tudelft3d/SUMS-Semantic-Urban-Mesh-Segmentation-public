/*
*   Name        : mesh_processor.cpp
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

#include "mesh_processor.hpp"
namespace semantic_mesh_segmentation
{
	void get_training_data()
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Get all training data." << std::endl;
		base_names.clear();
		ply_files.clear();
		file_folders.clear();
		training_base_names.clear();
		training_ply_files.clear();
		training_file_folders.clear();
		training_file_ind_map.clear();

		std::ostringstream temp_path;
		temp_path << root_path << folder_names_level_0[2] << folder_names_level_1[0];
		training_data_path = temp_path.str().data();
		getAllFiles(training_data_path, file_formats[0], training_ply_files, training_file_folders);//get .ply filenames

		int ind = 0;
		for (auto str_i : training_ply_files)
		{
			training_base_names.push_back(get_file_based_name(str_i));
			training_file_ind_map[training_base_names[ind]] = ind;
			++ind;
		}
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void get_testing_data()
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Get all test data." << std::endl;
		base_names.clear();
		ply_files.clear();
		file_folders.clear();
		testing_base_names.clear();
		testing_ply_files.clear();
		testing_file_folders.clear();
		testing_file_ind_map.clear();

		std::ostringstream temp_path;
		temp_path << root_path << folder_names_level_0[2] << folder_names_level_1[1];
		testing_data_path = temp_path.str().data();
		getAllFiles(testing_data_path, file_formats[0], testing_ply_files, testing_file_folders);//get .ply filenames

		int ind = 0;
		for (auto str_i : testing_ply_files)
		{
			testing_base_names.push_back(get_file_based_name(str_i));
			testing_file_ind_map[testing_base_names[ind]] = ind;
			++ind;
		}
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void get_predicting_data()
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Get all predict data." << std::endl;
		base_names.clear();
		ply_files.clear();
		file_folders.clear();
		predicting_base_names.clear();
		predicting_ply_files.clear();
		predicting_file_folders.clear();
		predicting_file_ind_map.clear();

		std::ostringstream temp_path;
		temp_path << root_path << folder_names_level_0[2] << folder_names_level_1[2];
		predicting_data_path = temp_path.str().data();
		getAllFiles(predicting_data_path, file_formats[0], predicting_ply_files, predicting_file_folders);//get .ply filenames

		int ind = 0;
		for (auto str_i : predicting_ply_files)
		{
			predicting_base_names.push_back(get_file_based_name(str_i));
			predicting_file_ind_map[predicting_base_names[ind]] = ind;
			++ind;
		}
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void get_validation_data()
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Get all validation data." << std::endl;
		base_names.clear();
		ply_files.clear();
		file_folders.clear();
		validation_base_names.clear();
		validation_ply_files.clear();
		validation_file_folders.clear();
		validation_file_ind_map.clear();

		std::ostringstream temp_path;
		temp_path << root_path << folder_names_level_0[2] << folder_names_level_1[3];
		validation_data_path = temp_path.str().data();
		getAllFiles(validation_data_path, file_formats[0], validation_ply_files, validation_file_folders);//get .ply filenames

		int ind = 0;
		for (auto str_i : validation_ply_files)
		{
			validation_base_names.push_back(get_file_based_name(str_i));
			validation_file_ind_map[validation_base_names[ind]] = ind;
			++ind;
		}
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	bool check_sub_names
	(
		int &batch_index,
		std::vector<std::set<std::string>> &all_sub_names,
		std::string &name_2
	)
	{
		bool found = false;
		auto it_tmp = all_sub_names[batch_index].find(name_2);
		if (it_tmp != all_sub_names[batch_index].end())
			found = true;
		return found;
	}

	void update_base_names
	(
		int &batch_index,
		std::vector<std::set<std::string>> &all_base_names,
		std::string &name_1
	)
	{
		auto it_tmp = all_base_names[batch_index].find(name_1);
		if (it_tmp == all_base_names[batch_index].end())
		{
			all_base_names[batch_index].insert(name_1);
		}
	}

	void update_sub_names
	(
		int &batch_index,
		std::vector<std::set<std::string>> &all_sub_names,
		std::string &name_2
	)
	{
		auto it_tmp = all_sub_names[batch_index].find(name_2);
		if (it_tmp == all_sub_names[batch_index].end())
		{
			all_sub_names[batch_index].insert(name_2);
		}
	}

	void fill_pre_batch_or_activate_new
	(
		int &batch_index,
		std::string &ba_i,
		std::vector<std::set<std::string>> &all_base_names,
		std::vector<std::set<std::string>> &all_sub_names,
		std::map<int, bool> &activated_batch_map,
		const std::string name_1,
		const std::string name_2,
		const int batch_num,
		std::map<int, int> &batch_size_map
	)
	{
		int last_activated_index = -1;
		for (int b_ind = 0; b_ind < batch_num; ++b_ind)
		{
			//if can be find in sub name lists
			//and previous batch is still activated
			//then move back to fill the previous batch
			if (activated_batch_map[b_ind])
			{
				last_activated_index = batch_index;
				auto it_tmp = all_sub_names[b_ind].find(name_2);
				if (it_tmp != all_sub_names[b_ind].end() &&
					all_base_names[b_ind].size() < sub_batch_size)
				{
					batch_index = b_ind;
					break;
				}
			}

			//if cannot find in sub name lists
			//and previous batch is not activated
			//then move to new empty batch
			if (b_ind == batch_num - 1)
			{
				std::map<int, int>::iterator it_m = batch_size_map.begin();
				for (; it_m != batch_size_map.end(); ++it_m)
				{
					if (it_m->second == 0)
					{
						last_activated_index = it_m->first;
						break;
					}
				}

				//check if move to previous batch for fill in the sub name lists
				//otherwise, move to new batch
				if (all_sub_names[last_activated_index - 1].size() < sub_batch_size &&
					all_base_names[last_activated_index - 1].size() < sub_batch_size)
				{
					std::string fist_sub_name = *(all_sub_names[last_activated_index - 1].begin());
					auto it_tmp = all_sub_names[last_activated_index - 1].find(name_2);
					if (it_tmp == all_sub_names[last_activated_index - 1].end() && std::stoi(name_2) > std::stoi(fist_sub_name))
					{
						last_activated_index -= 1;
					}
				}

				batch_index = last_activated_index;
			}
		}
	}

	void square_batch_separation(std::vector<std::vector<std::pair<int, std::string>>> &all_batches_out)
	{
		//batch_size = 2;
		//sub_batch_size = 2;
		//base_names.clear();
		//base_names = {"a_1", "a_2", "a_3", "a_4", "a_5", "a_6",				// normal case
		//			  "b_1", "b_2", "b_4", "b_5", "b_6",						// missing tiles
		//			  "c_1", "c_2", "c_3", "c_4", "c_5", "c_6", "c_7",			// additional tiles
		//			  "d_1_1", "d_1_2", "d_2", "d_3", "d_4", "d_5", "d_6" };    // sub-tiles
		/*
		batch_size = 2; e.g. {a, b}, {c, d}
		sub_batch_size = 2; e.g. {a1, a2}, {b1, b2}
		square batch = {{a1, a2}, {b1, b2}}

			batch:0  |   batch:1  |  batch:2  |  batch:3  |  batch:4  |  batch:5
			a1		 |	 a3		  |	 a5		  |	 c1		  |	  c3	  |	 c5
			a2		 |	 a4		  |	 a6		  |	 c2		  |	  c4	  |	 c6
			b1		 |	 		  |	 b5		  |	 d1_1	  |	  d3	  |	 c7
			b2		 |	 b4		  |	 b6		  |	 d1_2	  |	  d4	  |	 d5
					 |            |           |  d2	      |	          |  d6
		*/

		//suppose we have mesh tile named as name1_name2_name3
		std::vector<std::vector<std::pair<int, std::string>>> all_batches; //store mesh names and index per square batch
		std::vector<std::set<std::string>> all_base_names, all_sub_names;  //store mesh non repeat sub names per square batch 
		int batch_num = std::ceil(float(base_names.size()) / float(sub_batch_size * batch_size)); //compute max batch number
		all_batches.resize(batch_num);
		all_base_names.resize(batch_num);
		all_sub_names.resize(batch_num);

		//initialize all batch size map and activated batch map, sub tiles not count
		std::map<int, int> batch_size_map;
		std::map<int, bool> activated_batch_map;

		for (int i = 0; i < batch_num; ++i)
		{
			batch_size_map[i] = 0; //each tile should smaller than sub_batch_size
			activated_batch_map[i] = false;
		}

		int tile_count = 0, batch_index = 0, sub_batch_count = 0;
		std::string pre_name_1 = "", pre_name_2 = "";

		for (auto ba_i : base_names)
		{
			//std::cout << "tile_count = " << tile_count << std::endl;
			//get tmp as separate sub names in do while loop
			char *basic_name_c = new char[ba_i.length() + 1];
			strcpy(basic_name_c, ba_i.c_str());
			const char *delim = batch_delim_separate_char.c_str();//"_";
			char *tmp = strtok(basic_name_c, delim);
			//define temp variable that only valid in current mesh name processing
			int name_pos = 0;
			std::string name_1 = "", name_2 = "";
			while (tmp != NULL)
			{
				bool is_ignore = false;
				for (auto ig_i : batch_ignored_str)
				{
					if (tmp == ig_i)
					{
						tmp = strtok(NULL, delim);
						is_ignore = true;
					}
				}
				if (is_ignore)
					continue;

				if (name_pos == 0)
				{
					//process name_1
					name_1 = tmp;
				}
				else if (name_pos == 1)
				{
					//process name_2 and add tile to batch
					name_2 = tmp;
					if (pre_name_1 == "" || pre_name_1 == name_1) //normal case
					{
						//add current tile into the corresponding batch
						if (sub_batch_count < sub_batch_size ||
							pre_name_2 == name_2)//let subsequent sub tiles in
						{
							if (!check_sub_names(batch_index, all_sub_names, name_2) &&
								all_sub_names[batch_index].size() >= sub_batch_size)
							{
								fill_pre_batch_or_activate_new
								(
									batch_index,
									ba_i,
									all_base_names,
									all_sub_names,
									activated_batch_map,
									name_1,
									name_2,
									batch_num,
									batch_size_map
								);
							}

							if (pre_name_2 != name_2)//ignore sub tiles
							{
								++batch_size_map[batch_index];
								++sub_batch_count;
							}
						}
						else
						{
							//update batch_index, either pre-batch or new batch index
							fill_pre_batch_or_activate_new
							(
								batch_index,
								ba_i,
								all_base_names,
								all_sub_names,
								activated_batch_map,
								name_1,
								name_2,
								batch_num,
								batch_size_map
							);
							sub_batch_count = 1;
							++batch_size_map[batch_index];
						}

						//add data to the new batch
						all_batches[batch_index].emplace_back(tile_count, ba_i);

						//add new sub names, the list of sub name < sub_batch_size
						update_base_names(batch_index, all_base_names, name_1);
						update_sub_names(batch_index, all_sub_names, name_2);

						//batch activation
						if (!activated_batch_map[batch_index])
							activated_batch_map[batch_index] = true;
					}
					else //move next name_1
					{
						//update batch_index, either pre-batch or new batch index
						fill_pre_batch_or_activate_new
						(
							batch_index,
							ba_i,
							all_base_names,
							all_sub_names,
							activated_batch_map,
							name_1,
							name_2,
							batch_num,
							batch_size_map
						);

						sub_batch_count = 1;
						all_batches[batch_index].emplace_back(tile_count, ba_i);
						++batch_size_map[batch_index];
						//add new sub names, the list of sub name < sub_batch_size
						update_base_names(batch_index, all_base_names, name_1);
						update_sub_names(batch_index, all_sub_names, name_2);
					}
				}

				tmp = strtok(NULL, delim);
				if (name_pos == 1)
				{
					pre_name_1 = name_1;
					pre_name_2 = name_2;

					//batch deactivation
					for (int i = 0; i < batch_num; ++i)
					{
						if (activated_batch_map[i] &&
							batch_size_map[i] > (sub_batch_size * batch_size - sub_batch_size))
						{
							activated_batch_map[i] = false;
						}
					}
					break;
				}
				++name_pos;
			}

			++tile_count;
			delete[]basic_name_c;
		}

		for (auto batch_i : all_batches)
		{
			if (!batch_i.empty())
			{
				all_batches_out.push_back(batch_i);
			}
		}

		//write batch in *.txt
		save_txt_batches(all_batches_out);

		std::cout << "batch (squared area) number = " << all_batches_out.size() << "; tiles = " << tile_count << std::endl;
	}

	void smote_data_augmentation()
	{
		//initialize
		Py_Initialize();
		if (!Py_IsInitialized())
		{
			std::cout << "python init fail" << std::endl;
		}
		PyObject* pModule = NULL;
		PyObject* pFunc = NULL;

		//add python file path
		std::string path = seg_aug_py_path;
		std::string chdir_cmd = std::string("sys.path.append(\"") + path + "\")";
		const char* cstr_cmd = chdir_cmd.c_str();

		PyRun_SimpleString("import sys");
		PyRun_SimpleString(cstr_cmd);

		int label_size = labels_name.size();
		if (!ignored_labels_name.empty())
			label_size = labels_name.size() - ignored_labels_name.size();

		std::string py_file = "seg_aug";
		std::string in_folder_path = root_path + folder_names_level_0[1] + folder_names_level_1[0];
		std::string out_folder_path = root_path + folder_names_level_0[1] + folder_names_level_1[4];
		std::string label_name_str = std::to_string(label_size);
		std::string used_k_neighbors_str = std::to_string(used_k_neighbors);
		std::wstring py_file_wcmd(py_file.begin(), py_file.end());
		std::wstring in_folder_wcmd(in_folder_path.begin(), in_folder_path.end());
		std::wstring out_folder_wcmd(out_folder_path.begin(), out_folder_path.end());
		std::wstring label_name_wcmd(label_name_str.begin(), label_name_str.end());
		std::wstring used_k_neighbors_wcmd(used_k_neighbors_str.begin(), used_k_neighbors_str.end());
		const wchar_t* py_file_cstr_cmd = py_file_wcmd.c_str();

		int argc;
		wchar_t * argv[4];
		argc = 4;
		argv[0] = const_cast<wchar_t*>(in_folder_wcmd.c_str());
		argv[1] = const_cast<wchar_t*>(out_folder_wcmd.c_str());
		argv[2] = const_cast<wchar_t*>(label_name_wcmd.c_str());
		argv[3] = const_cast<wchar_t*>(used_k_neighbors_wcmd.c_str());

		Py_SetProgramName(py_file_cstr_cmd);
		PySys_SetArgv(argc, argv);

		//load module
		PyRun_SimpleString("import seg_aug");
		Py_Finalize();
	}

	void conversion_from_ground_truth
	(
		SFMesh *smesh_in,
		std::vector<float> &label_seg_statistics
	)
	{
		//get segments
		int max_seg_id = 0;
		std::map<int, std::vector<int>> seg_faces;
		for (auto f : smesh_in->faces())
		{
			int seg_id = smesh_in->get_face_segment_id[f];
			max_seg_id = max_seg_id < seg_id ? seg_id : max_seg_id;

			if (seg_faces.find(seg_id) == seg_faces.end())
			{
				seg_faces[seg_id] = std::vector<int>();
				seg_faces[seg_id].push_back(f.idx());
			}
			else
			{
				seg_faces[seg_id].push_back(f.idx());
			}
		}

		std::vector< std::vector<int>> seg_face_vec(max_seg_id + 1, std::vector<int>());
		std::map<int, std::vector<int>>::iterator mit = seg_faces.begin();
		std::map<int, std::vector<int>>::iterator mit_end = seg_faces.end();
		for (; mit != mit_end; ++mit)
		{
			if (mit->first >= 0 && mit->first < max_seg_id + 1)
				seg_face_vec[mit->first] = mit->second;
		}

		//get segment label from truth
		for (int si = 0; si < seg_face_vec.size(); ++si)
		{
			std::vector<int> majority_labels(labels_name.size() + 1, 0);
			for (int fi = 0; fi < seg_face_vec[si].size(); ++fi)
			{
				SFMesh::Face fdx(seg_face_vec[si][fi]);
				int fdx_label_truth = smesh_in->get_face_truth_label[fdx];

				if (fdx_label_truth != 0 && fdx_label_truth != -1)
					++majority_labels[fdx_label_truth];
				else
					++majority_labels[0];

				smesh_in->get_face_segment_id[fdx] = si;
			}
			int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			++label_seg_statistics[maxElementIndex];
		}
	}

	//Node feature random
	void augmented_node_feauture_random
	(
		std::vector<float> & tmp_basic_feas,
		std::vector<float> &tmp_segment_eigen_feas,
		std::vector<float> &tmp_segment_color_feas,
		std::vector<float> &tmp_mulsc_ele_feas
	)
	{
		int pos_or_neg = rand() % (2);//random 0, 1
		float temf = rand() % (100 + 1);//random 0 - 100
		if (pos_or_neg == 0)
			pos_or_neg = -1;
		else
			pos_or_neg = 1;

		temf /= 10000.0f;//+- 0.01

		for (int shp_i = 0; shp_i < tmp_basic_feas.size(); ++shp_i)
			tmp_basic_feas[shp_i] += pos_or_neg * temf;
		for (int mscele2_i = 0; mscele2_i < tmp_mulsc_ele_feas.size(); ++mscele2_i)
		{
			tmp_mulsc_ele_feas[mscele2_i] += pos_or_neg * temf;
		}

		for (int geo_i = 0; geo_i < tmp_segment_eigen_feas.size(); ++geo_i)
			tmp_segment_eigen_feas[geo_i] += pos_or_neg * temf;

		for (int tex_i = 0; tex_i < tmp_segment_color_feas.size(); ++tex_i)
			tmp_segment_color_feas[tex_i] += pos_or_neg * temf;
	}

	//-- Joint labeling feature concatenation ---
	void joint_labels_feature_concatenation
	(
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &segment_eigen_feas,
		std::vector< std::vector<float> > &segment_color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		std::map<std::pair<int, int>, std::pair<float, bool>> &spf_edges_maps
	)
	{
		std::vector<int> seg_truth_join;
		std::vector< std::vector<float> > basic_feas_join, mulsc_ele_feas_join;
		std::vector< std::vector<float> > segment_eigen_feas_join, segment_color_feas_join;
		std::map<std::pair<int, int>, std::pair<float, bool>> spf_edges_maps_clone;
		spf_edges_maps_clone.insert(spf_edges_maps.begin(), spf_edges_maps.end());
		std::map<std::pair<int, int>, std::pair<float, bool>>::iterator m_it = spf_edges_maps_clone.begin();
		std::map<std::pair<int, int>, std::pair<float, bool>>::iterator m_it_end = spf_edges_maps_clone.end();

		//superedge_ij, superedge_ji are different sites
		for (; m_it != m_it_end; ++m_it)
		{
			if (m_it->second.second == false)
			{
				int s_ind = m_it->first.first;
				int t_ind = m_it->first.second;
				m_it->second.second = true;
				spf_edges_maps_clone[std::make_pair(t_ind, s_ind)].second = true;

				int label_s = seg_truth[s_ind];
				int label_t = seg_truth[t_ind];

				int label_sj = label_s != label_t ? label_s + label_t : label_s;
				int label_tj = label_s != label_t ? label_s + label_t : label_t;

				seg_truth_join.push_back(label_sj);
				seg_truth_join.push_back(label_tj);

				std::vector<float> basic_feas_s = basic_feas[s_ind];
				std::vector<float> basic_feas_t = basic_feas[t_ind];
				basic_feas_s.insert(basic_feas_s.end(), basic_feas[t_ind].begin(), basic_feas[t_ind].end());
				basic_feas_t.insert(basic_feas_t.end(), basic_feas[s_ind].begin(), basic_feas[s_ind].end());
				basic_feas_join.push_back(basic_feas_s);
				basic_feas_join.push_back(basic_feas_t);

				std::vector<float> mulsc_ele_feas_s = mulsc_ele_feas[s_ind];
				std::vector<float> mulsc_ele_feas_t = mulsc_ele_feas[t_ind];
				mulsc_ele_feas_s.insert(mulsc_ele_feas_s.end(), mulsc_ele_feas[t_ind].begin(), mulsc_ele_feas[t_ind].end());
				mulsc_ele_feas_t.insert(mulsc_ele_feas_t.end(), mulsc_ele_feas[s_ind].begin(), mulsc_ele_feas[s_ind].end());
				mulsc_ele_feas_join.push_back(mulsc_ele_feas_s);
				mulsc_ele_feas_join.push_back(mulsc_ele_feas_t);

				std::vector<float> segment_eigen_feas_s = segment_eigen_feas[s_ind];
				std::vector<float> segment_eigen_feas_t = segment_eigen_feas[t_ind];
				segment_eigen_feas_s.insert(segment_eigen_feas_s.end(), segment_eigen_feas[t_ind].begin(), segment_eigen_feas[t_ind].end());
				segment_eigen_feas_t.insert(segment_eigen_feas_t.end(), segment_eigen_feas[s_ind].begin(), segment_eigen_feas[s_ind].end());
				segment_eigen_feas_join.push_back(segment_eigen_feas_s);
				segment_eigen_feas_join.push_back(segment_eigen_feas_t);

				std::vector<float> segment_color_feas_s = segment_color_feas[s_ind];
				std::vector<float> segment_color_feas_t = segment_color_feas[t_ind];
				segment_color_feas_s.insert(segment_color_feas_s.end(), segment_color_feas[t_ind].begin(), segment_color_feas[t_ind].end());
				segment_color_feas_t.insert(segment_color_feas_t.end(), segment_color_feas[s_ind].begin(), segment_color_feas[s_ind].end());
				segment_color_feas_join.push_back(segment_color_feas_s);
				segment_color_feas_join.push_back(segment_color_feas_t);
			}
		}

		//parsing back
		seg_truth.clear(); seg_truth.assign(seg_truth_join.begin(), seg_truth_join.end());
		basic_feas.clear(); basic_feas.assign(basic_feas_join.begin(), basic_feas_join.end());
		segment_eigen_feas.clear(); segment_eigen_feas.assign(segment_eigen_feas_join.begin(), segment_eigen_feas_join.end());
		segment_color_feas.clear(); segment_color_feas.assign(segment_color_feas_join.begin(), segment_color_feas_join.end());
		mulsc_ele_feas.clear(); mulsc_ele_feas.assign(mulsc_ele_feas_join.begin(), mulsc_ele_feas_join.end());
	}

	//Compute the set of connected components of a given mesh, return largest connected component id
	int separate_connected_components
	(
		SFMesh *smesh_out,
		int &component_num
	)
	{
		std::vector<std::pair<int, float>> spf_temp;//id, area
		std::vector< std::pair<int, SFMesh::Face>> CCV;
		CCV.clear();
		std::stack<SFMesh::Face> sf;
		SFMesh::Face fpt = *smesh_out->faces().begin();

		for (auto f : smesh_out->faces())
		{
			if (!smesh_out->get_face_component_visit[f])
			{
				smesh_out->get_face_component_visit[f] = true;
				CCV.emplace_back(0, f);

				std::pair<int, float> current_temp(CCV.size(), smesh_out->get_face_area[f]);
				spf_temp.emplace_back(current_temp);

				sf.push(f);
				while (!sf.empty())
				{
					fpt = sf.top();
					++CCV.back().first;
					sf.pop();

					smesh_out->get_face_component_id[fpt] = CCV.size();

					spf_temp[CCV.size() - 1].second += smesh_out->get_face_area[fpt];

					SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_out->halfedges(fpt);
					SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
					do
					{
						SFMesh::Halfedge ho = smesh_out->opposite_halfedge(*h_fit);
						if (smesh_out->is_boundary(ho) == false)
						{
							SFMesh::Edge eo = smesh_out->edge(ho);
							SFMesh::Face l = smesh_out->face(ho);
							if (!smesh_out->get_face_component_visit[l])
							{
								smesh_out->get_face_component_visit[l] = true;
								sf.push(l);
							}
						}
						++h_fit;
					} while (h_fit != h_end);
				}
			}
		}

		std::pair<int, float> laspf_area(-1, 0.0f);
		for (auto ci : spf_temp)
		{
			if (ci.second > laspf_area.second)
			{
				laspf_area.first = ci.first;
				laspf_area.second = ci.second;
			}
		}

		component_num = CCV.size();
		return laspf_area.first;
	}

	void construct_superfacet_neighbors
	(
		SFMesh* smesh_in,
		std::map<std::pair<int, int>, std::pair<float, bool>> &spf_edges_maps, // key: seg id pairs; value: common length, visited flag
		std::vector<int> seg_ids
	)
	{
		std::map<int, int> segid_ind;
		for (int i = 0; i < seg_ids.size(); ++i)
			segid_ind[seg_ids[i]] = i;

		//ccv
		int component_num = 0, largest_component = -1;
		largest_component = separate_connected_components(smesh_in, component_num);

		for (auto ei : smesh_in->edges())
		{
			int id_1 = -1, id_2 = -1,
				cmp_id_1 = -1, cmp_id_2 = -1;
			SFMesh::Vertex vs = smesh_in->vertex(ei, 0);
			SFMesh::Vertex vt = smesh_in->vertex(ei, 1);
			float edge_dis = Distance_3D(smesh_in->get_points_coord[vs], smesh_in->get_points_coord[vt]);

			SFMesh::Halfedge h0 = smesh_in->halfedge(ei, 0);
			if (!smesh_in->is_boundary(h0))
			{
				id_1 = segid_ind[smesh_in->get_face_segment_id[smesh_in->face(h0)]];
				cmp_id_1 = smesh_in->get_face_component_id[smesh_in->face(h0)];
			}

			SFMesh::Halfedge h1 = smesh_in->halfedge(ei, 1);
			if (!smesh_in->is_boundary(h1))
			{
				id_2 = segid_ind[smesh_in->get_face_segment_id[smesh_in->face(h1)]];
				cmp_id_2 = smesh_in->get_face_component_id[smesh_in->face(h1)];
			}

			//normal segment border facets or isolated component facets (consider self neighbor)
			if ((id_1 != id_2 || (cmp_id_1 != largest_component && cmp_id_2 != largest_component))
				&& (id_1 != -1 && id_2 != -1))//boundary faces
			{
				std::pair<int, int> st_pair = std::make_pair(id_1, id_2);
				std::pair<int, int> ts_pair = std::make_pair(id_2, id_1);

				if (spf_edges_maps.find(st_pair) == spf_edges_maps.end() &&
					spf_edges_maps.find(ts_pair) == spf_edges_maps.end())
				{
					spf_edges_maps[st_pair] = std::make_pair(edge_dis, false);
					spf_edges_maps[ts_pair] = std::make_pair(edge_dis, false);
				}
				else
				{
					spf_edges_maps[st_pair].first += edge_dis;
					spf_edges_maps[ts_pair].first += edge_dis;
				}
			}
		}
	}


	//--- processing training batch tiles ---
	void training_feature_process_batch_tiles
	(
		std::vector<int> &seg_truth_train,
		std::vector< std::vector<float>> &basic_feas_train,
		std::vector< std::vector<float> > &segment_eigen_feas_train,
		std::vector< std::vector<float> > &segment_color_feas_train,
		std::vector< std::vector<float> > &mulsc_ele_feas_train,
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	)
	{
		std::string sfc_in = prefixs[9] + std::to_string(batch_index);
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float>> basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float>> segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		SFMesh* smesh_all = new SFMesh;
		mesh_configuration(smesh_all);
		int ind = 0, vert_ind = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;

			//--- read mesh *.ply data ---
			read_mesh_data(smesh_tmp, tile_i.first);

			//--- merge mesh ---
			merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size);
			training_mesh_area += smesh_tmp->mesh_area;
			tex_size += smesh_tmp->texture_names.size();
			delete smesh_tmp;
			++ind;
		}

		if (!enable_augment)
		{
			seg_truth.clear();
			get_mesh_labels(smesh_all, seg_truth, seg_face_vec);
		}

		//parsing to training datasets
		basic_feas_train.insert(basic_feas_train.end(), basic_feas.begin(), basic_feas.end());
		segment_eigen_feas_train.insert(segment_eigen_feas_train.end(), segment_eigen_feas.begin(), segment_eigen_feas.end());
		segment_color_feas_train.insert(segment_color_feas_train.end(), segment_color_feas.begin(), segment_color_feas.end());
		mulsc_ele_feas_train.insert(mulsc_ele_feas_train.end(), mulsc_ele_feas.begin(), mulsc_ele_feas.end());
		seg_truth_train.insert(seg_truth_train.end(), seg_truth.begin(), seg_truth.end());

		delete smesh_all;
		delete pcl_out;
	}

	//--- processing training single tiles ---
	void training_feature_process_single_tiles
	(
		std::vector<int> &seg_truth_train,
		std::vector< std::vector<float>> &basic_feas_train,
		std::vector< std::vector<float> > &segment_eigen_feas_train,
		std::vector< std::vector<float> > &segment_color_feas_train,
		std::vector< std::vector<float> > &mulsc_ele_feas_train,
		const int pi
	)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas, mulsc_ele_feas
		);

		//get truth label directly from mesh 
		SFMesh *tmp_mesh = new SFMesh;
		read_mesh_data(tmp_mesh, pi);
		if (!enable_augment)
		{
			seg_truth.clear();
			get_mesh_labels(tmp_mesh, seg_truth, seg_face_vec);
		}

		//for joint-labeling, construct pairwise adjacent superfacet and feature concatenate
		if (enable_joint_labeling)
		{
			std::map<std::pair<int, int>, std::pair<float, bool>> spf_edges_maps;
			construct_superfacet_neighbors(tmp_mesh, spf_edges_maps, seg_ids);

			//concatenate features
			joint_labels_feature_concatenation
			(
				seg_truth,
				basic_feas, segment_eigen_feas, segment_color_feas,
				mulsc_ele_feas,
				spf_edges_maps
			);
		}

		//parsing to training datasets
		basic_feas_train.insert(basic_feas_train.end(), basic_feas.begin(), basic_feas.end());
		segment_eigen_feas_train.insert(segment_eigen_feas_train.end(), segment_eigen_feas.begin(), segment_eigen_feas.end());
		segment_color_feas_train.insert(segment_color_feas_train.end(), segment_color_feas.begin(), segment_color_feas.end());
		mulsc_ele_feas_train.insert(mulsc_ele_feas_train.end(), mulsc_ele_feas.begin(), mulsc_ele_feas.end());
		seg_truth_train.insert(seg_truth_train.end(), seg_truth.begin(), seg_truth.end());

		training_mesh_area += tmp_mesh->mesh_area;

		delete tmp_mesh;
		delete pcl_out;
	}

	//--- processing testing batch tiles ---
	void testing_feature_process_batch_tiles_with_model
	(
		Label_set &labels_test,
		Feature_set &node_features,
		CGAL::Classification::ETHZ::Random_forest_classifier &eth_rf_classifier,
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	)
	{
		std::string sfc_in = prefixs[9] + std::to_string(batch_index);
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		SFMesh* smesh_all = new SFMesh;
		mesh_configuration(smesh_all);
		int ind = 0, vert_ind = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;

			//--- read mesh *.ply data ---
			std::vector<cv::Mat> texture_maps;
			read_mesh_data(smesh_tmp, tile_i.first, texture_maps, batch_index);

			//--- merge mesh ---
			merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size);
			test_mesh_area += smesh_tmp->mesh_area;
			tex_size += smesh_tmp->texture_names.size();
			delete smesh_tmp;
			++ind;
		}
		seg_truth.clear();
		get_mesh_labels(smesh_all, seg_truth, seg_face_vec);

		if (!ignored_labels_name.empty() && train_test_predict_val != 2)
			ignore_truth_labels(seg_truth);
		else if (train_test_predict_val == 2)
			check_ignored_truth_labels();

		std::vector<Cluster_point> cluster_test = convert_to_CGAL_Cluster(basic_feas);
		std::vector<int> label_indices_test;
		std::vector<int> predict_label;
		std::vector<float> predict_prob;
		std::vector<std::vector<float>> predict_prob_all;

		ETH_RF_Test_with_Trained_Model
		(
			pcl_out,
			cluster_test,
			label_indices_test,
			seg_truth,
			basic_feas,
			segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas,
			labels_test,
			predict_label, predict_prob, predict_prob_all,
			node_features, eth_rf_classifier
		);


		std::vector<int> face_predict_label, face_label_indices, face_truth_label;
		std::vector<float> face_predict_prob, face_area_weighted;
		parsing_properties_from_classification
		(
			smesh_all,
			seg_face_vec,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			predict_label,
			predict_prob,
			predict_prob_all
		);

		ETH_RF_SavingTest_and_or_Evaluation
		(
			smesh_all,
			labels_test,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			batch_index
		);
		delete pcl_out;
		delete smesh_all;
	}

	//--- processing testing single tiles ---
	void testing_feature_process_single_tiles_with_model
	(
		Label_set &labels_test,
		Feature_set &node_features,
		CGAL::Classification::ETHZ::Random_forest_classifier &eth_rf_classifier,
		const int pi
	)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		SFMesh *tmp_mesh = new SFMesh;
		seg_truth.clear();
		read_mesh_data(tmp_mesh, pi);
		get_mesh_labels(tmp_mesh, seg_truth, seg_face_vec);
		test_mesh_area += tmp_mesh->mesh_area;

		if (!ignored_labels_name.empty() && train_test_predict_val != 2)
			ignore_truth_labels(seg_truth);
		else if (train_test_predict_val == 2)
			check_ignored_truth_labels();

		std::vector<Cluster_point> cluster_test = convert_to_CGAL_Cluster(basic_feas);
		std::vector<int> label_indices_test;
		std::vector<int> predict_label;
		std::vector<float> predict_prob;
		std::vector<std::vector<float>> predict_prob_all;

		ETH_RF_Test_with_Trained_Model
		(
			pcl_out,
			cluster_test,
			label_indices_test,
			seg_truth,
			basic_feas,
			segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas,
			labels_test,
			predict_label, predict_prob, predict_prob_all,
			node_features, eth_rf_classifier
		);

		std::vector<int> face_predict_label, face_label_indices, face_truth_label;
		std::vector<float> face_predict_prob, face_area_weighted;
		parsing_properties_from_classification
		(
			tmp_mesh,
			seg_face_vec,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			predict_label,
			predict_prob,
			predict_prob_all
		);

		ETH_RF_SavingTest_and_or_Evaluation
		(
			tmp_mesh,
			labels_test,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			pi
		);
		delete pcl_out;
		delete tmp_mesh;
	}

	//--- processing testing batch tiles ---
	void testing_feature_process_batch_tiles_without_model
	(
		CGAL::Classification::ETHZ::Random_forest_classifier &classifier,
		Feature_set &node_features,
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	)
	{
		std::string sfc_in = prefixs[9] + std::to_string(batch_index);
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float>> basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth, basic_feas,
			segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		SFMesh* smesh_all = new SFMesh;
		mesh_configuration(smesh_all);
		int ind = 0, vert_ind = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;

			//--- read mesh *.ply data ---
			std::vector<cv::Mat> texture_maps;
			read_mesh_data(smesh_tmp, tile_i.first, texture_maps, batch_index);

			//--- merge mesh ---
			merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size);
			test_mesh_area += smesh_tmp->mesh_area;
			tex_size += smesh_tmp->texture_names.size();
			delete smesh_tmp;
			++ind;
		}
		seg_truth.clear();
		get_mesh_labels(smesh_all, seg_truth, seg_face_vec);

		if (!ignored_labels_name.empty() && train_test_predict_val != 2)
			ignore_truth_labels(seg_truth);
		else if (train_test_predict_val == 2)
			check_ignored_truth_labels();

		std::vector<Cluster_point> cluster_test = convert_to_CGAL_Cluster(basic_feas);
		std::vector<int> label_indices_test;
		Label_set labels_test;
		std::vector<int> predict_label;
		std::vector<float> predict_prob;
		std::vector<std::vector<float>> predict_prob_all;
		node_features.clear();
		ETH_RF_Test_Base
		(
			pcl_out,
			cluster_test,
			label_indices_test,
			seg_truth,
			basic_feas,
			segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas,
			labels_test,
			node_features,
			classifier,
			predict_label, predict_prob, predict_prob_all
		);

		std::vector<int> face_predict_label, face_label_indices, face_truth_label;
		std::vector<float> face_predict_prob, face_area_weighted;
		parsing_properties_from_classification
		(
			smesh_all,
			seg_face_vec,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			predict_label,
			predict_prob,
			predict_prob_all
		);

		ETH_RF_SavingTest_and_or_Evaluation
		(
			smesh_all,
			labels_test,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			batch_index
		);
		delete pcl_out;
		delete smesh_all;
	}

	//--- processing testing single tiles ---
	void testing_feature_process_single_tiles_without_model
	(
		CGAL::Classification::ETHZ::Random_forest_classifier &classifier,
		Feature_set &node_features,
		const int pi
	)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		SFMesh *tmp_mesh = new SFMesh;
		seg_truth.clear();
		read_mesh_data(tmp_mesh, pi);
		get_mesh_labels(tmp_mesh, seg_truth, seg_face_vec);

		if (!ignored_labels_name.empty() && train_test_predict_val != 2)
			ignore_truth_labels(seg_truth);
		else if (train_test_predict_val == 2)
			check_ignored_truth_labels();

		test_mesh_area += tmp_mesh->mesh_area;
		//for joint-labeling, construct pairwise adjacent superfacet and feature concatenate
		std::map<std::pair<int, int>, std::pair<float, bool>> spf_edges_maps;
		if (enable_joint_labeling)
		{
			//construct superfacet neighbors
			construct_superfacet_neighbors(tmp_mesh, spf_edges_maps, seg_ids);

			//concatenate features
			//To do: check if superfacet id and centers are corresponding!
			joint_labels_feature_concatenation
			(
				seg_truth,
				basic_feas, segment_eigen_feas, segment_color_feas,
				mulsc_ele_feas,
				spf_edges_maps
			);
		}

		std::vector<Cluster_point> cluster_test = convert_to_CGAL_Cluster(basic_feas);

		std::vector<int> label_indices_test;
		std::vector<int> predict_label;
		std::vector<float> predict_prob;
		std::vector<std::vector<float>> predict_prob_all;
		Label_set labels_test;

		node_features.clear();
		ETH_RF_Test_Base
		(
			pcl_out,
			cluster_test,
			label_indices_test,
			seg_truth,
			basic_feas,
			segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas,
			labels_test,
			node_features,
			classifier,
			predict_label, predict_prob, predict_prob_all
		);

		//MRF refinement
		if (enable_joint_labeling)
		{
			std::map<int, std::tuple<int, int, std::vector<float>>> unary_label_prob;//sites, (count joint labels, accumulate labels_probs)
			std::map<std::pair<int, int>, float> pairwise_label_prob; //neighbors, prob
			joint_labeling_energy //for test data only
			(
				spf_edges_maps,
				predict_label,
				predict_prob,
				predict_prob_all,
				unary_label_prob,
				pairwise_label_prob //neighbors, prob
			);

			//MRF optimization
			MRF_joint_labeling
			(
				tmp_mesh,
				seg_face_vec,
				unary_label_prob,
				pairwise_label_prob, //neighbors, prob
				pi
			);
		}

		std::vector<int> face_predict_label, face_truth_label;
		std::vector<float> face_predict_prob, face_area_weighted;
		parsing_properties_from_classification
		(
			tmp_mesh,
			seg_face_vec,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			predict_label,
			predict_prob,
			predict_prob_all
		);

		ETH_RF_SavingTest_and_or_Evaluation
		(
			tmp_mesh,
			labels_test,
			face_predict_label,
			face_truth_label,
			face_area_weighted,
			pi
		);

		delete pcl_out;
		delete tmp_mesh;
	}

	//--- parsing semantic from color ---
	void parsing_semantics_from_color
	(
		SFMesh* smesh_in,
		PointCloud* spg_semantic_pcl
	)
	{
		smesh_in->get_face_color = smesh_in->get_face_property<vec3>("f:color");
		auto spg_color = spg_semantic_pcl->get_vertex_property<vec3>("v:color");

		//get predict information from SPG point cloud
		float delta = 0.05;
		for (auto fdx : smesh_in->faces())
		{
			std::map<int, int> label_count;
			std::tuple<int, int, vec3> max_labelc(-1, 0, vec3());
			for (auto pi : smesh_in->get_face_sampled_points[fdx])
			{
				PTCloud::Vertex ptx(pi);
				int label_pred = 0;
				for (auto li_color : labels_color)
				{
					if (
						std::abs(li_color.x - spg_color[ptx].x) < delta &&
						std::abs(li_color.y - spg_color[ptx].y) < delta &&
						std::abs(li_color.z - spg_color[ptx].z) < delta
						)
					{
						if (!label_count[label_pred])
							label_count[label_pred] = 1;
						else
							++label_count[label_pred];
						break;
					}
					++label_pred;
				}
				if (label_pred == labels_color.size())
				{
					if (!label_count[-1])
						label_count[-1] = 1;
					else
						++label_count[-1];
				}

				if (label_count[label_pred] > get<1>(max_labelc))
				{
					get<0>(max_labelc) = label_pred;
					get<1>(max_labelc) = label_count[label_pred];
					get<2>(max_labelc) = spg_color[ptx];
				}
			}
			smesh_in->get_face_predict_label[fdx] = get<0>(max_labelc) + 1;
			smesh_in->get_face_color[fdx] = get<2>(max_labelc);
		}
	}

	//--- processing training batch tiles ---
	void feature_diversity_process_batch_tiles
	(
		std::vector<float> &batches_feas_var,
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	)
	{
		std::string sfc_in = prefixs[9] + std::to_string(batch_index);
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float>> basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float>> segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		float fea_var = feature_variances_computation
		(
			seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);
		batches_feas_var.push_back(fea_var);

		delete pcl_out;
	}

	//--- processing training single tiles ---
	void feature_diversity_process_single_tiles
	(
		std::vector<float> &tiles_feas_var,
		const int pi
	)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector< std::vector<int>> seg_face_vec;
		std::vector< std::vector<float>> basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);

		float fea_var = feature_variances_computation
		(
			seg_truth,
			basic_feas, segment_eigen_feas, segment_color_feas,
			mulsc_ele_feas
		);
		tiles_feas_var.push_back(fea_var);

		delete pcl_out;
	}

	void nearst_neighbor_labels_assign
	(
		SFMesh* smesh_in,
		PointCloud* spg_semantic_pcl,
		PTCloud *tmp_pcl
	)
	{
		std::string label_s = "v:" + label_string;
		auto get_pcl_labels = spg_semantic_pcl->get_vertex_property<int>(label_s);
		auto get_points_label = tmp_pcl->get_vertex_property<int>("v:label");

		easy3d::KdTree *tree_3d = new easy3d::KdTree;
		Build_kdtree(spg_semantic_pcl, tree_3d);
#pragma omp parallel for schedule(dynamic)
		for (int pt_i = 0; pt_i < tmp_pcl->vertices_size(); ++pt_i)
		{
			PTCloud::Vertex pt(pt_i);
			int nearst_id = tree_3d->find_closest_point(tmp_pcl->get_points_coord[pt]);
			PointCloud::Vertex p_near(nearst_id);
			get_points_label[pt] = get_pcl_labels[p_near];
		}

		//get predict information from SOTA point cloud
		for (auto fdx : smesh_in->faces())
		{
			std::map<int, int> label_count;
			std::tuple<int, int, vec3> max_labelc(-1, 0, vec3());
			for (auto pi : smesh_in->get_face_sampled_points[fdx])
			{
				PTCloud::Vertex ptx(pi);
				int label_pred = int(get_points_label[ptx]) - label_minus;//for randlanet
				if (!label_count[label_pred])
					label_count[label_pred] = 1;
				else
					++label_count[label_pred];

				if (label_count[label_pred] > get<1>(max_labelc))
				{
					get<0>(max_labelc) = label_pred;
					get<1>(max_labelc) = label_count[label_pred];
					get<2>(max_labelc) = labels_color[label_pred];
				}
			}
			smesh_in->get_face_predict_label[fdx] = get<0>(max_labelc) + 1;
			smesh_in->get_face_color[fdx] = get<2>(max_labelc);
		}

		delete tree_3d;
	}

	void filter_unclassified_points
	(
		PointCloud* semantic_pcl
	)
	{
		std::string label_s = "v:" + label_string;
		auto get_pcl_labels = semantic_pcl->get_vertex_property<int>(label_s);
		auto get_pcl_coord = semantic_pcl->get_vertex_property<easy3d::vec3>("v:point");
		auto get_pcl_color = semantic_pcl->get_vertex_property<easy3d::vec3>("v:color");

		PointCloud* semantic_pcl_cp = new PointCloud;
		semantic_pcl_cp->add_vertex_property<int>("v:label", -1);
		auto get_cp_pcl_labels = semantic_pcl_cp->get_vertex_property<int>("v:label");
		semantic_pcl_cp->add_vertex_property<easy3d::vec3>("v:color", easy3d::vec3());
		auto get_cp_pcl_color = semantic_pcl_cp->get_vertex_property<easy3d::vec3>("v:color");
		for (auto vd : semantic_pcl->vertices())
		{
			if (get_pcl_labels[vd] > 0)
			{
				auto cur_vd = semantic_pcl_cp->add_vertex(get_pcl_coord[vd]);
				get_cp_pcl_labels[cur_vd] = get_pcl_labels[vd];
				get_cp_pcl_color[cur_vd] = get_pcl_color[vd];
			}
		}

		easy3d::KdTree* tree_cp = new easy3d::KdTree;
		Build_kdtree(semantic_pcl_cp, tree_cp);

		for (auto vd : semantic_pcl->vertices())
		{
			if (get_pcl_labels[vd] == 0)
			{
				auto vd_nearest_i = tree_cp->find_closest_point(get_pcl_coord[vd]);
				easy3d::PointCloud::Vertex vd_nearest(vd_nearest_i);
				get_pcl_labels[vd] = get_cp_pcl_labels[vd_nearest];
				get_pcl_color[vd] = get_cp_pcl_color[vd_nearest];
			}
		}

		delete tree_cp;
		delete semantic_pcl_cp;
	}

	void translate_point_clouds
	(
		PointCloud* semantic_pcl,
		PointCloud* sampled_pcl
	)
	{
		auto get_sampled_pcl_coord = sampled_pcl->get_vertex_property<easy3d::vec3>("v:point");
		easy3d::vec3 trans_pt = sampled_pcl->points()[0];
		if (translation_strategy == 2)
		{
			easy3d::vec3 min_coord(FLT_MAX, FLT_MAX, FLT_MAX);
			for (auto vd : sampled_pcl->vertices())
			{
				min_coord.x = min_coord.x < get_sampled_pcl_coord[vd].x ? min_coord.x : get_sampled_pcl_coord[vd].x;
				min_coord.y = min_coord.y < get_sampled_pcl_coord[vd].y ? min_coord.y : get_sampled_pcl_coord[vd].y;
				min_coord.z = min_coord.z < get_sampled_pcl_coord[vd].z ? min_coord.z : get_sampled_pcl_coord[vd].z;
			}
			trans_pt.x = min_coord.x;
			trans_pt.y = min_coord.y;
			trans_pt.z = min_coord.z;
		}
		std::string label_s = "v:" + label_string;
		auto get_pcl_labels = semantic_pcl->get_vertex_property<int>(label_s);
		auto get_pcl_coord = semantic_pcl->get_vertex_property<easy3d::vec3>("v:point");
		auto get_pcl_color = semantic_pcl->get_vertex_property<easy3d::vec3>("v:color");

		PointCloud* semantic_pcl_trans = new PointCloud;
		semantic_pcl_trans->add_vertex_property<int>("v:label", -1);
		auto get_trans_pcl_labels = semantic_pcl_trans->get_vertex_property<int>("v:label");
		semantic_pcl_trans->add_vertex_property<easy3d::vec3>("v:color", easy3d::vec3());
		auto get_trans_pcl_color = semantic_pcl_trans->get_vertex_property<easy3d::vec3>("v:color");
		for (auto vd : semantic_pcl->vertices())
		{
			auto cur_vd = semantic_pcl_trans->add_vertex(get_pcl_coord[vd] + trans_pt);
			get_trans_pcl_labels[cur_vd] = get_pcl_labels[vd];
			get_trans_pcl_color[cur_vd] = get_pcl_color[vd];
		}

		auto get_trans_pcl_coord = semantic_pcl_trans->get_vertex_property<easy3d::vec3>("v:point");
		semantic_pcl->clear();
		for (auto vd : semantic_pcl_trans->vertices())
		{
			auto cur_vd = semantic_pcl->add_vertex(get_trans_pcl_coord[vd]);
			get_pcl_labels[cur_vd] = get_trans_pcl_labels[vd];
			get_pcl_color[cur_vd] = get_trans_pcl_color[vd];
		}

		delete semantic_pcl_trans;
	}

	void face_labels_assign_based_on_radius_votes_and_knn
	(
		SFMesh* smesh_in,
		PointCloud* semantic_pcl
	)
	{
		std::string label_s = "v:" + label_string;
		auto get_pcl_coord = semantic_pcl->get_vertex_property<easy3d::vec3>("v:point");
		auto get_pcl_labels = semantic_pcl->get_vertex_property<int>(label_s);

		easy3d::KdTree* tree_3d = new easy3d::KdTree;
		Build_kdtree(semantic_pcl, tree_3d);

		//get predict information from SOTA point cloud
		for (auto fdx : smesh_in->faces())
		{
			bool has_pt_in_triangle = false;
			std::vector<int> fd_label_votes;
			fd_label_votes.resize(labels_name.size(), 0);

			easy3d::vec3 fd_cen(0.0f, 0.0f, 0.0f);
			for (auto vd : smesh_in->vertices(fdx))
				fd_cen += smesh_in->get_points_coord[vd];
			fd_cen /= 3.0f;
			
			Plane plane_tri = Plane(Point_3(fd_cen.x, fd_cen.y, fd_cen.z),
				Vector_3(smesh_in->get_face_normals[fdx].x, smesh_in->get_face_normals[fdx].y, smesh_in->get_face_normals[fdx].z));
			double max_radius = 0.0f;
			std::vector<double> U_vec, V_vec;
			for (auto vd : smesh_in->vertices(fdx))
			{
				double dis_tmp = easy3d::distance2(smesh_in->get_points_coord[vd], fd_cen);
				max_radius = max_radius > dis_tmp ? max_radius : dis_tmp;
				Point_2 p_proj_cgal = plane_tri.to_2d(Point_3(smesh_in->get_points_coord[vd].x, smesh_in->get_points_coord[vd].y, smesh_in->get_points_coord[vd].z));
				U_vec.push_back(p_proj_cgal.x());
				V_vec.push_back(p_proj_cgal.y());
			}

			if (sampling_eval != 0)
			{
				std::vector<int> neighbors;
				std::vector<float> squared_distanace;
				tree_3d->find_points_in_radius(fd_cen, max_radius, neighbors, squared_distanace);
				if (!neighbors.empty())
				{
					for (int ni = 0; ni < neighbors.size(); ++ni)
					{
						easy3d::PointCloud::Vertex vd_nearst(neighbors[ni]);
						easy3d::vec3 p_nearest = get_pcl_coord[vd_nearst];
						Point_2 p_proj_cgal = plane_tri.to_2d(Point_3(p_nearest.x, p_nearest.y, p_nearest.z));
						std::vector<double> P = { p_proj_cgal.x(), p_proj_cgal.y() };
						if (PointinTriangle(U_vec, V_vec, P))
						{
							has_pt_in_triangle = true;
							fd_label_votes[get_pcl_labels[vd_nearst] + ignored_labels_name.size() - label_minus] += 1;
						}
					}
				}
			}
						
			if (!has_pt_in_triangle)
			{
				auto vd_nearest_i = tree_3d->find_closest_point(fd_cen);
				easy3d::PointCloud::Vertex vd_nearest(vd_nearest_i);
				fd_label_votes[get_pcl_labels[vd_nearest] + ignored_labels_name.size() - label_minus] += 1;
			}

			auto max_element_iter = std::max_element(fd_label_votes.begin(), fd_label_votes.end());
			smesh_in->get_face_predict_label[fdx] = std::distance(fd_label_votes.begin(), max_element_iter);
			smesh_in->get_face_color[fdx] = labels_color[smesh_in->get_face_predict_label[fdx]];
		}

		delete tree_3d;
	}

	void parsing_semantics_from_labelstring
	(
		SFMesh* smesh_in,
		PointCloud* spg_semantic_pcl
	)
	{
		std::string label_s = "v:" + label_string;
		bool use_float = false;
		if (sota_folder_path == "pointnet/" || sota_folder_path == "pointnet2/")
			use_float = true;

		smesh_in->add_vertex_property<bool>("v:vertex_visited", false);
		auto get_vert_visited = smesh_in->get_vertex_property<bool>("v:vertex_visited");
		//get predict information from SOTA point cloud
		int pi = 0;
		for (auto fdx : smesh_in->faces())
		{
			std::map<int, int> label_count;
			std::tuple<int, int, vec3> max_labelc(-1, 0, vec3());

			for (auto pi : smesh_in->get_face_sampled_points[fdx])
			{
				PTCloud::Vertex ptx(pi);
				if (sampling_strategy == -1 || sampling_strategy == 1 || sampling_strategy == 2)
				{
					if (!use_float)
					{
						if (spg_semantic_pcl->get_vertex_property<int>("v:truth")[ptx] == -1)
						{
							continue;
						}
					}
					else
					{
						if (spg_semantic_pcl->get_vertex_property<float>("v:truth")[ptx] == -1)
						{
							continue;
						}
					}
				}

				int label_pred;
				if (!use_float)
				{
					label_pred = int(spg_semantic_pcl->get_vertex_property<int>(label_s)[ptx]) - label_minus;
				}
				else
				{
					label_pred = int(spg_semantic_pcl->get_vertex_property<float>(label_s)[ptx]) - label_minus;
				}

				if (!label_count[label_pred])
					label_count[label_pred] = 1;
				else
					++label_count[label_pred];

				if (label_count[label_pred] > get<1>(max_labelc))
				{
					get<0>(max_labelc) = label_pred;
					get<1>(max_labelc) = label_count[label_pred];
					get<2>(max_labelc) = labels_color[label_pred];
				}
			}

			smesh_in->get_face_predict_label[fdx] = get<0>(max_labelc) + 1;
			smesh_in->get_face_color[fdx] = get<2>(max_labelc);
		}
	}

	vec3 hsv_to_rgb(const vec3& c)
	{
		double h = c.x;
		double s = c.y;
		double v = c.z;

		s /= 100.;
		v /= 100.;
		double C = v * s;
		int hh = (int)(h / 60.);
		double X = C * (1 - CGAL::abs(hh % 2 - 1));
		double r = 0, g = 0, b = 0;

		if (hh >= 0 && hh < 1)
		{
			r = C;
			g = X;
		}
		else if (hh >= 1 && hh < 2)
		{
			r = X;
			g = C;
		}
		else if (hh >= 2 && hh < 3)
		{
			g = C;
			b = X;
		}
		else if (hh >= 3 && hh < 4)
		{
			g = X;
			b = C;
		}
		else if (hh >= 4 && hh < 5)
		{
			r = X;
			b = C;
		}
		else
		{
			r = C;
			b = X;
		}
		double m = v - C;
		r += m;
		g += m;
		b += m;

		vec3 out = vec3(r, g, b);
		return out;
	}

	void error_map
	(
		SFMesh *smesh_original,
		const int mi
	)
	{
		for (auto &f : smesh_original->faces())
		{
			if (smesh_original->get_face_truth_label[f] - 1 == smesh_original->get_face_predict_label[f]
				|| smesh_original->get_face_truth_label[f] == 0
				|| smesh_original->get_face_truth_label[f] == -1)
				smesh_original->get_face_error_color[f].x = 120.0;
			else
				smesh_original->get_face_error_color[f].x = 0.0f;

			smesh_original->get_face_error_color[f].y = 100.0f;
			smesh_original->get_face_error_color[f].z = 100.0f;

			smesh_original->get_face_error_color[f] = hsv_to_rgb(smesh_original->get_face_error_color[f]);

			smesh_original->get_face_color[f] = smesh_original->get_face_error_color[f];
		}

		write_error_mesh_data(smesh_original, mi);
	}


	//--- merge point cloud ---
	void merge_semantic_pointcloud
	(
		PointCloud*cloud_tmp,
		PTCloud *cloud_all,
		SFMesh *smesh_all,
		int &pre_face_size
	)
	{
		auto get_points_coord = cloud_tmp->get_vertex_property<vec3>("v:point");
		auto get_points_color = cloud_tmp->get_vertex_property<vec3>("v:color");
		for (auto ptx : cloud_tmp->vertices())
		{
			cloud_all->add_vertex(get_points_coord[ptx]);
			cloud_all->get_points_color[*(--cloud_all->vertices_end())] = get_points_color[ptx];

			int fi = cloud_all->get_points_face_belong_id[*(--cloud_all->vertices_end())];
			SFMesh::Face fdx(fi);

			smesh_all->get_face_sampled_points_id_index_map[fdx][(*(--cloud_all->vertices_end())).idx()] = smesh_all->get_face_sampled_points[fdx].size();
			smesh_all->get_face_sampled_points[fdx].push_back((*(--cloud_all->vertices_end())).idx());
		}
	}

	void processing_semantic_pcl_input
	(
		SFMesh* smesh_in,
		PointCloud* spg_semantic_pcl,
		PTCloud* tmp_pcl
	)
	{
		if (label_string == "") //use color to parsing semantics
			parsing_semantics_from_color(smesh_in, spg_semantic_pcl);
		else
		{
			if (equal_cloud)
				parsing_semantics_from_labelstring(smesh_in, spg_semantic_pcl);
			else
				nearst_neighbor_labels_assign(smesh_in, spg_semantic_pcl, tmp_pcl);
		}
	}

	void processing_semantic_pcl_with_texture_masks
	(
		SFMesh* mesh_in,
		PointCloud* semantic_pcl,
		std::vector<cv::Mat>& texture_mask_maps_pred,
		std::vector<cv::Mat>& texture_mask_maps_full,
		const std::vector<cv::Mat>& texture_maps
	)
	{
		std::string label_s = "v:" + label_string;
		auto get_pcl_label = semantic_pcl->get_vertex_property<int>(label_s);
		easy3d::KdTree* tree_3d = new easy3d::KdTree;
		Build_kdtree(semantic_pcl, tree_3d);

		// initialize texture mask
		for (int ti = 0; ti < texture_maps.size(); ++ti)
		{
			cv::Mat pred_mask = cv::Mat::zeros(texture_maps[ti].rows, texture_maps[ti].cols, texture_maps[ti].type());
			pred_mask.setTo(cv::Scalar(255, 255, 255));
			texture_mask_maps_pred.push_back(pred_mask);

			cv::Mat full_mask = cv::Mat::zeros(texture_maps[ti].rows, texture_maps[ti].cols, texture_maps[ti].type());
			full_mask.setTo(cv::Scalar(255, 255, 255));
			texture_mask_maps_full.push_back(full_mask);
		}

		// paring labels
		for (auto& fd : mesh_in->faces())
		{
			mesh_in->get_face_property<easy3d::vec3>("f:color")[fd] = easy3d::vec3();
			int texture_id = mesh_in->get_face_texnumber[fd];
			int width = texture_maps[texture_id].cols;
			int height = texture_maps[texture_id].rows;

			std::vector<easy3d::vec2> uv_triangle;
			std::vector<double> U_vec, V_vec, UL_vec, VL_vec;
			std::vector<easy3d::vec3> coord3d_triangle;
			for (auto& vd : mesh_in->vertices(fd))
				coord3d_triangle.push_back(mesh_in->get_points_coord[vd]);

			U_vec.push_back(mesh_in->get_face_texcoord[fd][0]);
			U_vec.push_back(mesh_in->get_face_texcoord[fd][2]);
			U_vec.push_back(mesh_in->get_face_texcoord[fd][4]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][1]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][3]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][5]);

			uv_triangle.emplace_back(U_vec[0], V_vec[0]);
			uv_triangle.emplace_back(U_vec[1], V_vec[1]);
			uv_triangle.emplace_back(U_vec[2], V_vec[2]);

			UL_vec.insert(UL_vec.end(), U_vec.begin(), U_vec.end());
			VL_vec.insert(VL_vec.end(), V_vec.begin(), V_vec.end());

			std::pair<int, int> uv_dis;
			std::pair<double, double> uv_min;
			enlarge_uv_triangle(UL_vec, VL_vec, uv_dis, uv_min, width, height);
			std::vector<int> fd_label_votes;
			fd_label_votes.resize(labels_name.size(), 0);

			for (int u_i = 0; u_i < uv_dis.first; ++u_i)
			{
				for (int v_i = 0; v_i < uv_dis.second; ++v_i)
				{
					std::vector<double> P =
					{
						uv_min.first + double(u_i) / double(width),
						uv_min.second + double(v_i) / double(height)
					};

					if (P[0] > 1.0f || P[0] < 0.0f || P[1] > 1.0f || P[1] < 0.0f)
					{
						P.clear();
						continue;
					}

					if (PointinTriangle(UL_vec, VL_vec, P))
					{
						easy3d::vec2 newcoord(P[0], P[1]);
						easy3d::vec3 current_3d;

						uv_to_3D_coordinates(uv_triangle, coord3d_triangle, newcoord, current_3d);
						int vnearst_i = tree_3d->find_closest_point(current_3d);
						easy3d::PointCloud::Vertex vd_nearst(vnearst_i);
						int cur_label = get_pcl_label[vd_nearst] + ignored_labels_name.size() - label_minus; //-1 : remove unclassified, start from 0 for terrian
						if (cur_label < labels_name.size())
						{
							fd_label_votes[cur_label] += 1;
							easy3d::vec3 tex_label_color = 255.0f * labels_color[cur_label];
							texture_mask_maps_full[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_pred[texture_id].rows - 1, newcoord[0] * texture_mask_maps_pred[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
						}
						else
						{
							easy3d::vec3 tex_label_color = 255.0f * tex_labels_color[cur_label - labels_color.size()];
							texture_mask_maps_pred[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_pred[texture_id].rows - 1, newcoord[0] * texture_mask_maps_pred[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
							texture_mask_maps_full[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_pred[texture_id].rows - 1, newcoord[0] * texture_mask_maps_pred[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
							fd_label_votes[tex_fd_label[cur_label - labels_color.size()]] += 1;
						}
					}
				}
			}

			auto max_element_iter = std::max_element(fd_label_votes.begin(), fd_label_votes.end());
			mesh_in->get_face_property<int>("f:label_predict")[fd] = std::distance(fd_label_votes.begin(), max_element_iter);
			mesh_in->get_face_property<easy3d::vec3>("f:color")[fd] = labels_color[mesh_in->get_face_property<int>("f:label_predict")[fd]];
		}

		delete tree_3d;
	}

	void processing_semantic_pcl_with_texture_sp
	(
		SFMesh* mesh_in,
		PointCloud* semantic_pcl,
		easy3d::PointCloud* tex_sp_pcl,
		std::vector<cv::Mat>& texture_mask_maps_pred,
		std::vector<cv::Mat>& texture_mask_maps_full,
		const std::vector<cv::Mat>& texture_maps,
		const std::vector<cv::Mat>& texture_sps
	)
	{
		// build map
		std::string label_s = "v:" + label_string;
		auto get_pcl_sp_id = tex_sp_pcl->get_vertex_property<int>("v:sp_id");
		auto get_pcl_label = semantic_pcl->get_vertex_property<int>(label_s);
		std::map<int, int> spid_vd_map;
		for (auto vd : tex_sp_pcl->vertices())
			spid_vd_map[get_pcl_sp_id[vd]] = vd.idx();

		// initialize texture mask
		for (int ti = 0; ti < texture_maps.size(); ++ti)
		{
			cv::Mat pred_mask = cv::Mat::zeros(texture_maps[ti].rows, texture_maps[ti].cols, texture_maps[ti].type());
			pred_mask.setTo(cv::Scalar(255, 255, 255));
			texture_mask_maps_pred.push_back(pred_mask);

			cv::Mat full_mask = cv::Mat::zeros(texture_maps[ti].rows, texture_maps[ti].cols, texture_maps[ti].type());
			full_mask.setTo(cv::Scalar(255, 255, 255));
			texture_mask_maps_full.push_back(full_mask);
		}

		// paring labels
		for (auto& fd : mesh_in->faces())
		{
			mesh_in->get_face_property<easy3d::vec3>("f:color")[fd] = easy3d::vec3();
			int texture_id = mesh_in->get_face_texnumber[fd];
			int width = texture_maps[texture_id].cols;
			int height = texture_maps[texture_id].rows;

			std::vector<easy3d::vec2> uv_triangle;
			std::vector<double> U_vec, V_vec, UL_vec, VL_vec;
			std::vector<easy3d::vec3> coord3d_triangle;
			for (auto& vd : mesh_in->vertices(fd))
				coord3d_triangle.push_back(mesh_in->get_points_coord[vd]);

			U_vec.push_back(mesh_in->get_face_texcoord[fd][0]);
			U_vec.push_back(mesh_in->get_face_texcoord[fd][2]);
			U_vec.push_back(mesh_in->get_face_texcoord[fd][4]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][1]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][3]);
			V_vec.push_back(mesh_in->get_face_texcoord[fd][5]);

			uv_triangle.emplace_back(U_vec[0], V_vec[0]);
			uv_triangle.emplace_back(U_vec[1], V_vec[1]);
			uv_triangle.emplace_back(U_vec[2], V_vec[2]);

			UL_vec.insert(UL_vec.end(), U_vec.begin(), U_vec.end());
			VL_vec.insert(VL_vec.end(), V_vec.begin(), V_vec.end());

			std::pair<int, int> uv_dis;
			std::pair<double, double> uv_min;
			enlarge_uv_triangle(UL_vec, VL_vec, uv_dis, uv_min, width, height);
			std::vector<int> fd_label_votes;
			fd_label_votes.resize(labels_name.size(), 0);

			for (int u_i = 0; u_i < uv_dis.first; ++u_i)
			{
				for (int v_i = 0; v_i < uv_dis.second; ++v_i)
				{
					std::vector<double> P =
					{
						uv_min.first + double(u_i) / double(width),
						uv_min.second + double(v_i) / double(height)
					};

					if (P[0] > 1.0f || P[0] < 0.0f || P[1] > 1.0f || P[1] < 0.0f)
					{
						P.clear();
						continue;
					}

					if (PointinTriangle(UL_vec, VL_vec, P))
					{
						easy3d::vec2 newcoord(P[0], P[1]);
						easy3d::vec3 current_3d;

						uv_to_3D_coordinates(uv_triangle, coord3d_triangle, newcoord, current_3d);

						int current_spid = (int)texture_sps[texture_id].at<int>((1 - newcoord[1]) * texture_maps[texture_id].rows - 1, newcoord[0] * texture_maps[texture_id].cols);
						easy3d::PointCloud::Vertex current_spvd(spid_vd_map[current_spid]);
						int cur_label = get_pcl_label[current_spvd] + ignored_labels_name.size() - label_minus; //-1 : remove unclassified, start from 0 for terrian
	
						if (cur_label < labels_name.size())
						{
							fd_label_votes[cur_label] += 1;

							easy3d::vec3 tex_label_color = 255.0f * labels_color[cur_label];
							texture_mask_maps_full[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_full[texture_id].rows - 1, newcoord[0] * texture_mask_maps_full[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
						}
						else
						{
							easy3d::vec3 tex_label_color = 255.0f * tex_labels_color[cur_label - labels_color.size()];
							texture_mask_maps_pred[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_pred[texture_id].rows - 1, newcoord[0] * texture_mask_maps_pred[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
							texture_mask_maps_full[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_maps_full[texture_id].rows - 1, newcoord[0] * texture_mask_maps_full[texture_id].cols) =
								cv::Vec3b(int(tex_label_color.z), int(tex_label_color.y), int(tex_label_color.x));
							fd_label_votes[tex_fd_label[cur_label - labels_color.size()]] += 1;
						}
					}
				}
			}

			auto max_element_iter = std::max_element(fd_label_votes.begin(), fd_label_votes.end());
			mesh_in->get_face_property<int>("f:label_predict")[fd] = std::distance(fd_label_votes.begin(), max_element_iter);
			mesh_in->get_face_property<easy3d::vec3>("f:color")[fd] = labels_color[mesh_in->get_face_property<int>("f:label_predict")[fd]];
		}
	}

	//--- processing semantic pcl batch tiles ---
	void semantic_pcl_process_batch_tiles
	(
		std::vector<std::pair<int, std::string>> &batch_base_names,
		const int batch_index
	)
	{
		std::cout << "Start to extract from batch " << std::to_string(batch_index) << std::endl;
		SFMesh* smesh_all = new SFMesh;
		PTCloud *cloud_all_tmp = new PTCloud;
		mesh_configuration(smesh_all);
		pointcloud_configuration(cloud_all_tmp);

		int ind = 0, vert_ind = 0, pre_face_size = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;
			PTCloud *cloud_tmp = new PTCloud;

			//--- read mesh *.ply data ---
			read_mesh_data(smesh_tmp, tile_i.first);

			//--- read sampled point cloud *.ply data ---
			read_pointcloud_data(smesh_tmp, cloud_tmp, 0, tile_i.first);

			//read .ply semantic point cloud
			PointCloud* spg_semantic_pcl_tmp = read_semantic_pointcloud_data(tile_i.first);//super-point graph data
			if (spg_semantic_pcl_tmp == nullptr)
			{
				std::cerr << "File loading failed" << std::endl;
				throw std::exception();
			}
			merge_semantic_pointcloud(spg_semantic_pcl_tmp, cloud_all_tmp, smesh_all, pre_face_size);

			pre_face_size += smesh_tmp->faces_size();

			//--- merge mesh ---
			merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size);
			test_mesh_area += smesh_tmp->mesh_area;
			tex_size += smesh_tmp->texture_names.size();

			delete smesh_tmp;
			delete cloud_tmp;
			delete spg_semantic_pcl_tmp;
			++ind;
		}

		processing_semantic_pcl_input(smesh_all, cloud_all_tmp);

		//if (train_test_predict_val != 2)
		//	spg_evaluation(smesh_all, batch_index);

		write_semantic_mesh_data(smesh_all, batch_index);

		delete smesh_all;
		delete cloud_all_tmp;
	}

	//--- processing semantic pcl single tiles ---
	void semantic_pcl_process_single_tiles
	(
		const int mi
	)
	{
		SFMesh* smesh_in = new SFMesh;
		std::cout << "Start to extract from mesh " << base_names[mi] << std::endl;
		//read training mesh data
		read_mesh_data(smesh_in, mi);

		//parsing point cloud info from original sampled point cloud
		PTCloud *tmp_pcl = new PTCloud;
		read_pointcloud_data(smesh_in, tmp_pcl, 0, mi);

		//read .ply semantic point cloud
		PointCloud* spg_semantic_pcl = read_semantic_pointcloud_data(mi);//super-point graph data
		if (spg_semantic_pcl == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		processing_semantic_pcl_input(smesh_in, spg_semantic_pcl, tmp_pcl);

		write_semantic_mesh_data(smesh_in, mi);
		delete smesh_in;
		delete tmp_pcl;
		delete spg_semantic_pcl;
	}

	//--- processing semantic pcl single tiles ---
	void semantic_pcl_process_single_tiles_v2
	(
		const int mi
	)
	{
		SFMesh* smesh_in = new SFMesh;
		std::vector<cv::Mat> texture_maps, texture_mask_maps_pred, texture_mask_maps_full, texture_sps;
		std::cout << "Start to extract from mesh " << base_names[mi] << std::endl;

		//read .ply semantic point cloud
		PointCloud* semantic_pcl = read_semantic_pointcloud_data(mi);
		if (semantic_pcl == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		easy3d::PointCloud* sampled_pcl = read_sampled_pointcloud_data(mi);
		//translate point clouds
		if (translation_strategy > 0)
			translate_point_clouds(semantic_pcl, sampled_pcl);

		//filter unclassified if there are any
		if (filter_unknow)
			filter_unclassified_points(semantic_pcl);

		if (translation_strategy > 0)
			write_semantic_pointcloud_data(semantic_pcl, "_trans", mi);

		if (with_texture_mask)
		{
			read_mesh_data(smesh_in, mi, texture_maps);

			if (sampling_eval == 3)
			{
				read_texsp_bin(smesh_in, texture_maps, texture_sps, mi);
				processing_semantic_pcl_with_texture_sp(smesh_in, semantic_pcl, sampled_pcl, texture_mask_maps_pred, texture_mask_maps_full, texture_maps, texture_sps);
			}
			else
			{
				processing_semantic_pcl_with_texture_masks(smesh_in, semantic_pcl, texture_mask_maps_pred, texture_mask_maps_full,  texture_maps);
			}

			write_semantic_mesh_data(smesh_in, mi, texture_mask_maps_pred, texture_mask_maps_full);
		}
		else
		{
			read_mesh_data(smesh_in, mi);
			face_labels_assign_based_on_radius_votes_and_knn(smesh_in, semantic_pcl);
			write_semantic_mesh_data(smesh_in, mi);
		}

		delete smesh_in;
		delete semantic_pcl;
		delete sampled_pcl;
	}

	void collect_semantic_labels
	(
		std::vector<int>& face_truth_label,
		std::vector<int>& face_test_label,
		std::vector<float>& face_area_weighted,
		const int mi
	)
	{
		//read original mesh
		SFMesh* smesh_test = new SFMesh;
		read_test_mesh_data(smesh_test, mi);

		for (auto fd : smesh_test->faces())
		{
			if (smesh_test->get_face_truth_label[fd] > 0)
			{
				if (!ignored_labels_name.empty())
				{
					if (!label_ignore[smesh_test->get_face_truth_label[fd] - 1])
					{
						int new_pred_label = smesh_test->get_face_predict_label[fd] - 1 - label_shiftdis[smesh_test->get_face_predict_label[fd] - 1];
						int new_truth_label = smesh_test->get_face_truth_label[fd] - 1 - label_shiftdis[smesh_test->get_face_truth_label[fd] - 1];

						face_truth_label.push_back(new_truth_label);
						face_test_label.push_back(new_pred_label);
						face_area_weighted.push_back(FaceArea(smesh_test, fd));
					}
				}
				else
				{
					face_truth_label.push_back(smesh_test->get_face_truth_label[fd] - 1);
					face_test_label.push_back(smesh_test->get_face_predict_label[fd]);
					face_area_weighted.push_back(FaceArea(smesh_test, fd));
				}
			}
		}

		//output error map
		if (save_error_map)
			error_map(smesh_test, mi);

		delete smesh_test;
	}

	void collect_semantic_labels_with_texture_mask
	(
		std::vector<int>& pix_truth_label,
		std::vector<int>& pix_test_label,
		const int mi
	)
	{
		SFMesh* mesh_in = new SFMesh;
		read_test_mesh_data(mesh_in, mi);

		std::vector<cv::Mat> gt_texture_mask_maps, pred_texture_mask_maps;
		read_mesh_texture_masks(mesh_in, gt_texture_mask_maps, mi);
		read_mesh_texture_masks(mesh_in, pred_texture_mask_maps, mi, true);

		// initialize texture mask
		std::vector<cv::Mat> texture_mask_error_map;
		if (save_error_map) 
		{
			for (int ti = 0; ti < pred_texture_mask_maps.size(); ++ti)
			{
				cv::Mat ti_mask = cv::Mat::zeros(pred_texture_mask_maps[ti].rows, pred_texture_mask_maps[ti].cols, pred_texture_mask_maps[ti].type());
				ti_mask.setTo(cv::Scalar(0, 255, 0));
				texture_mask_error_map.push_back(ti_mask);
			}
		}

		// paring labels
		for (auto& fd : mesh_in->faces())
		{
			if (mesh_in->get_face_truth_label[fd] - 1 >= 0)
			{
				mesh_in->get_face_property<easy3d::vec3>("f:color")[fd] = easy3d::vec3();
				int texture_id = mesh_in->get_face_texnumber[fd];
				int width = gt_texture_mask_maps[texture_id].cols;
				int height = gt_texture_mask_maps[texture_id].rows;

				std::vector<easy3d::vec2> uv_triangle;
				std::vector<double> U_vec, V_vec, UL_vec, VL_vec;
				std::vector<easy3d::vec3> coord3d_triangle;
				for (auto& vd : mesh_in->vertices(fd))
					coord3d_triangle.push_back(mesh_in->get_points_coord[vd]);

				U_vec.push_back(mesh_in->get_face_texcoord[fd][0]);
				U_vec.push_back(mesh_in->get_face_texcoord[fd][2]);
				U_vec.push_back(mesh_in->get_face_texcoord[fd][4]);
				V_vec.push_back(mesh_in->get_face_texcoord[fd][1]);
				V_vec.push_back(mesh_in->get_face_texcoord[fd][3]);
				V_vec.push_back(mesh_in->get_face_texcoord[fd][5]);

				uv_triangle.emplace_back(U_vec[0], V_vec[0]);
				uv_triangle.emplace_back(U_vec[1], V_vec[1]);
				uv_triangle.emplace_back(U_vec[2], V_vec[2]);

				UL_vec.insert(UL_vec.end(), U_vec.begin(), U_vec.end());
				VL_vec.insert(VL_vec.end(), V_vec.begin(), V_vec.end());

				std::pair<int, int> uv_dis;
				std::pair<double, double> uv_min;
				enlarge_uv_triangle(UL_vec, VL_vec, uv_dis, uv_min, width, height);

				for (int u_i = 0; u_i < uv_dis.first; ++u_i)
				{
					for (int v_i = 0; v_i < uv_dis.second; ++v_i)
					{
						std::vector<double> P =
						{
							uv_min.first + double(u_i) / double(width),
							uv_min.second + double(v_i) / double(height)
						};

						if (P[0] > 1.0f || P[0] < 0.0f || P[1] > 1.0f || P[1] < 0.0f)
						{
							P.clear();
							continue;
						}

						if (PointinTriangle(UL_vec, VL_vec, P))
						{
							easy3d::vec2 newcoord(P[0], P[1]);
							easy3d::vec3 current_3d;

							uv_to_3D_coordinates(uv_triangle, coord3d_triangle, newcoord, current_3d);

							if (std::isnan(current_3d.x) || std::isnan(current_3d.y) || std::isnan(current_3d.z))
								continue;

							float Rf = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[2];
							float Gf = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[1];
							float Bf = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[0];

							bool label_equal = false, has_gt_tex_label = false, has_pred_tex_label = false, may_has_tex_label = false;
							int gt_pix_label = mesh_in->get_face_truth_label[fd] - 1 - ignored_labels_name.size(); //remove: 0: unclassified; 1: terrian
							int pred_pix_label = mesh_in->get_face_predict_label[fd] - ignored_labels_name.size();

							float gt_Rf_mask = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[2];
							float gt_Gf_mask = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[1];
							float gt_Bf_mask = (float)gt_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * gt_texture_mask_maps[texture_id].rows - 1, newcoord[0] * gt_texture_mask_maps[texture_id].cols)[0];
							float pred_Rf_mask = (float)pred_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * pred_texture_mask_maps[texture_id].rows - 1, newcoord[0] * pred_texture_mask_maps[texture_id].cols)[2];
							float pred_Gf_mask = (float)pred_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * pred_texture_mask_maps[texture_id].rows - 1, newcoord[0] * pred_texture_mask_maps[texture_id].cols)[1];
							float pred_Bf_mask = (float)pred_texture_mask_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * pred_texture_mask_maps[texture_id].rows - 1, newcoord[0] * pred_texture_mask_maps[texture_id].cols)[0];
							
							for (int tex_ci = 0; tex_ci < tex_labels_color.size(); ++tex_ci)
							{
								if (std::abs(gt_Rf_mask - 255.0f * tex_labels_color[tex_ci][0]) <= 1.0f &&
									std::abs(gt_Gf_mask - 255.0f * tex_labels_color[tex_ci][1]) <= 1.0f &&
									std::abs(gt_Bf_mask - 255.0f * tex_labels_color[tex_ci][2]) <= 1.0f)
								{
									gt_pix_label = labels_color.size() + tex_ci - 1;
									has_gt_tex_label = true;
								}
							}

							for (int tex_ci = 0; tex_ci < tex_labels_color.size(); ++tex_ci)
							{
								if (std::abs(pred_Rf_mask - 255.0f * tex_labels_color[tex_ci][0]) <= 1.0f &&
									std::abs(pred_Gf_mask - 255.0f * tex_labels_color[tex_ci][1]) <= 1.0f &&
									std::abs(pred_Bf_mask - 255.0f * tex_labels_color[tex_ci][2]) <= 1.0f)
								{
									pred_pix_label = labels_color.size() + tex_ci - 1;
									has_pred_tex_label = true;
								}
							}

							if (may_has_tex_label && (has_gt_tex_label || has_pred_tex_label))
							{
								pix_truth_label.push_back(gt_pix_label);
								pix_test_label.push_back(pred_pix_label);

								if (gt_pix_label == pred_pix_label)
									label_equal = true;
							}
							else
							{
								pix_truth_label.push_back(gt_pix_label);
								pix_test_label.push_back(pred_pix_label);

								if (gt_pix_label == pred_pix_label)
									label_equal = true;
							}

							if (!label_equal)
								texture_mask_error_map[texture_id].at<cv::Vec3b>((1 - newcoord[1]) * texture_mask_error_map[texture_id].rows - 1, newcoord[0] * texture_mask_error_map[texture_id].cols) =
								cv::Vec3b(int(0), int(0), int(255));
						}
					}
				}
			}
		}

		if (save_error_map)
			write_error_mesh_texture_masks(mesh_in, texture_mask_error_map, mi);

		delete mesh_in;
	}

	//--- collect input meshes ---//
	void add_mesh_to_merge
	(
		SFMesh* mesh_merged,
		std::vector<cv::Mat>& texture_maps,
		std::vector<cv::Mat>& texture_mask_maps,
		const int mi,
		const int vert_size
	)
	{
		SFMesh* smesh_in = new SFMesh;
		std::cout << "Start to extract from mesh " << base_names[mi] << std::endl;
		//read training mesh data
		read_mesh_data(smesh_in, mi, texture_maps);

		//add mesh
		for (auto& vd : smesh_in->vertices())
			mesh_merged->add_vertex(smesh_in->get_points_coord[vd]);

		float mesh_total_area = 0.0f;
		for (auto& fd : smesh_in->faces())
		{
			std::vector<easy3d::SurfaceMesh::Vertex> verts;
			easy3d::vec3 fd_cen(0.0f, 0.0f, 0.0f);
			for (auto& vd : smesh_in->vertices(fd))
			{
				easy3d::SurfaceMesh::Vertex v_to_add(vd.idx() + vert_size);
				verts.push_back(v_to_add);
				fd_cen += smesh_in->get_points_coord[vd];
			}
			fd_cen /= 3.0f;
			auto cur_fd = mesh_merged->add_face(verts);

			mesh_merged->get_face_truth_label[cur_fd] = smesh_in->get_face_truth_label[fd];
			mesh_merged->get_face_area[cur_fd] = smesh_in->get_face_area[fd];
			mesh_total_area += mesh_merged->get_face_area[cur_fd];
			mesh_merged->get_face_normals[cur_fd] = smesh_in->get_face_normals[fd];
			mesh_merged->get_face_center[cur_fd] = fd_cen;
			mesh_merged->get_face_segment_id[cur_fd] = smesh_in->get_face_segment_id[fd];
			mesh_merged->get_face_color[cur_fd] = smesh_in->get_face_color[fd];

			if (with_texture)
			{
				mesh_merged->get_face_texnumber[cur_fd] = smesh_in->get_face_texnumber[fd] + mesh_merged->textures.size();
				mesh_merged->get_face_texcoord[cur_fd] = smesh_in->get_face_texcoord[fd];
			}
		}
		mesh_merged->mesh_area = mesh_total_area;

		if (with_texture)
		{
			mesh_merged->textures.insert(mesh_merged->textures.end(), smesh_in->textures.begin(), smesh_in->textures.end());
			for (int ti = 0; ti < smesh_in->textures.size(); ++ti)
			{
				auto tex_i = smesh_in->textures[ti];
				if (!tex_i.empty() && tex_i[tex_i.size() - 1] == '\r')
					tex_i.erase(tex_i.size() - 1);
				mesh_merged->texture_names.push_back(tex_i);

				if (with_texture_mask)
				{
					std::ostringstream texture_mask_str_ostemp;
					std::string mask_path_tmp;
					std::vector<std::string> texture_name_splits = Split(tex_i, ".", false);
					if (file_folders.size() > 1)
						mask_path_tmp = file_folders[mi] + "mask_" + texture_name_splits[0] + ".png";
					else
						mask_path_tmp = file_folders[0] + "mask_" + texture_name_splits[0] + ".png";

					texture_mask_str_ostemp << mask_path_tmp;
					std::string texture_mask_str_temp = texture_mask_str_ostemp.str().data();
					char* mask_texturePath_temp = (char*)texture_mask_str_temp.data();
					cv::Mat mask_texture_map = cv::imread(mask_texturePath_temp);
					if (mask_texture_map.empty())
					{
						std::cout << "read texture mask failure or no texture provide!" << std::endl;
						cv::Mat dummy(128, 128, CV_8UC3, cv::Scalar(0, 255, 0));
						mask_texture_map = dummy;
					}
					texture_mask_maps.emplace_back(mask_texture_map);
				}
			}
		}

		delete smesh_in;
	}

	void extract_semantic_mesh
	(
		SFMesh* mesh_merged,
		std::vector<std::vector<SFMesh::Face>>& label_component_faces, 
		std::vector<float>& component_area
	)
	{
		if (!mesh_merged->get_face_property<int>("f:semantic_component_id"))
			mesh_merged->add_face_property<int>("f:semantic_component_id", -1);
		auto fd_semantic_cid = mesh_merged->get_face_property<int>("f:semantic_component_id");
		for (int i = 0; i < component_label_name.size(); ++i)
		{
			float sum_area = 0.0f;
			std::map<int, int> vd_map;
			std::vector<SFMesh::Face> collected_faces;
			for (auto& fd : mesh_merged->faces())
			{
				bool is_match = false;
				int fd_label = mesh_merged->get_face_truth_label[fd] - 1;
				if (fd_label >= 0)
				{
					std::string cur_label_name = labels_name[fd_label];
					for (int j = 0; j < component_label_name[i].size(); ++j)
					{
						int res = cur_label_name.compare(component_label_name[i][j]);
						if (res == 0)
						{
							is_match = true;
							break;
						}
					}
				}
				else if (add_unclassified)
				{
					is_match = true;
				}

				if (is_match)
				{
					collected_faces.push_back(fd);
					fd_semantic_cid[fd] = i;
					sum_area += mesh_merged->get_face_area[fd];
				}
			}

			component_area.push_back(sum_area);
			label_component_faces.push_back(collected_faces);
		}
	}

	void extract_connected_component_from_sampled_cloud
	(
		SFMesh* mesh_merged,
		std::vector<SFMesh::Face>& label_component_faces_i,
		easy3d::PointCloud* sampled_pcl,
		std::vector<std::vector<SFMesh::Face>> &geo_component_faces
	)
	{
		auto fd_semantic_cid = mesh_merged->get_face_property<int>("f:semantic_component_id");
		if (!mesh_merged->get_face_property<int>("f:geometry_component_id"))
			mesh_merged->add_face_property<int>("f:geometry_component_id", -1);
		auto fd_cid = mesh_merged->get_face_property<int>("f:geometry_component_id");
		if (!mesh_merged->get_face_property<bool>("f:visited"))
			mesh_merged->add_face_property<bool>("f:visited", false);
		auto fd_visited = mesh_merged->get_face_property<bool>("f:visited");
		auto fd_sampled_pts = mesh_merged->get_face_property<std::vector<easy3d::vec3>>("f:sampled_points");

		easy3d::KdTree tree_3d;
		tree_3d.begin();
		tree_3d.add_point_cloud(sampled_pcl);
		tree_3d.end();

		auto vt_fid = sampled_pcl->get_vertex_property<int>("v:face_id");
		for (auto& fd : label_component_faces_i)
		{
			if (fd_visited[fd])
				continue;
			std::vector<SFMesh::Face> cur_component;
			std::stack<SFMesh::Face> stack;
			stack.push(fd);
			while (!stack.empty())
			{
				easy3d::SurfaceMesh::Face top = stack.top();
				fd_visited[top] = true;
				fd_cid[top] = cur_component.size();
				cur_component.push_back(top);
				stack.pop();
				if (top.is_valid() && top.idx() != -1)
				{
					std::map<int, bool> local_fd_visited;
					for (auto vd : mesh_merged->vertices(top))
					{
						for (auto fd_neg : mesh_merged->faces(vd))
						{
							if (!fd_visited[fd_neg] &&
								local_fd_visited.find(fd_neg.idx()) == local_fd_visited.end() && 
								fd_semantic_cid[top] == fd_semantic_cid[fd_neg])
							{
								stack.push(fd_neg);
								local_fd_visited[fd_neg.idx()] = true;
							}
						}
					}

					if (mesh_merged->is_boundary(top))
					{
						for (auto pt : fd_sampled_pts[top])
						{
							std::vector<int> neighbors;
							tree_3d.find_points_in_radius(pt, std::pow(adjacent_radius, 2), neighbors);

							for (auto vi : neighbors)
							{
								easy3d::PointCloud::Vertex v_neg(vi);
								easy3d::SurfaceMesh::Face fd_neg(vt_fid[v_neg]);
								if (!fd_visited[fd_neg] &&
									local_fd_visited.find(fd_neg.idx()) == local_fd_visited.end() && 
									fd_semantic_cid[top] == fd_semantic_cid[fd_neg])
								{
									stack.push(fd_neg);
									local_fd_visited[fd_neg.idx()] = true;
								}
							}
						}
					}
				}
			}
			geo_component_faces.push_back(cur_component);
		}
	}

	SFMesh* copy_mesh(SFMesh* mesh_in)
	{
		auto get_point_coord = mesh_in->get_vertex_property<easy3d::vec3>("v:point");
		SFMesh* mesh_cp = new SFMesh;
		input_mesh_configuration(mesh_cp);
		for (auto& vd : mesh_in->vertices())
			mesh_cp->add_vertex(get_point_coord[vd]);

		for (auto& fd : mesh_in->faces())
		{
			std::vector<SFMesh::Vertex> verts;
			for (auto& vd : mesh_in->vertices(fd))
			{
				verts.push_back(vd);
			}
			auto cur_fd = mesh_cp->add_face(verts);
			if (with_texture)
			{
				mesh_cp->get_face_texnumber[cur_fd] = mesh_in->get_face_texnumber[fd];
				mesh_cp->get_face_texcoord[cur_fd] = mesh_in->get_face_texcoord[fd];
			}
			mesh_cp->get_face_truth_label[cur_fd] = mesh_in->get_face_truth_label[fd];
			mesh_cp->get_face_area[cur_fd] = mesh_in->get_face_area[fd];
			mesh_cp->get_face_normals[cur_fd] = mesh_in->get_face_normals[fd];
			mesh_cp->get_face_center[cur_fd] = mesh_in->get_face_center[fd];
			mesh_cp->get_face_segment_id[cur_fd] = mesh_in->get_face_segment_id[fd];
			mesh_cp->get_face_color[cur_fd] = mesh_in->get_face_color[fd];
		}

		if (with_texture)
		{
			mesh_cp->textures.insert(mesh_cp->textures.end(), mesh_in->textures.begin(), mesh_in->textures.end());
			mesh_cp->texture_names.insert(mesh_cp->texture_names.end(), mesh_in->texture_names.begin(), mesh_in->texture_names.end());
		}
		return mesh_cp;
	}

	SFMesh* construct_component_mesh
	(
		SFMesh* mesh_merged,
		std::vector<SFMesh::Face>& geo_component_faces_i
	)
	{
		// delete non-used faces
		SFMesh* c_mesh = copy_mesh(mesh_merged);
		c_mesh->add_face_property<bool>("f:to_keep", false);
		auto fd_to_keep = c_mesh->get_face_property<bool>("f:to_keep");
		for (auto& fd : geo_component_faces_i)
		{
			fd_to_keep[fd] = true;
		}

		for (auto& fd : c_mesh->faces())
		{
			if (!fd_to_keep[fd])
				c_mesh->delete_face(fd);
		}
		c_mesh->garbage_collection();
		c_mesh->remove_face_property(fd_to_keep);

		const int n_vert_pre = c_mesh->n_vertices();
		int pre_n_vert = 0, cur_n_vert = c_mesh->n_vertices();
		do {
			pre_n_vert = cur_n_vert;
			for (auto& vd : c_mesh->vertices())
			{
				if (!c_mesh->is_manifold(vd))
					c_mesh->delete_vertex(vd);
			}
			c_mesh->garbage_collection();
			cur_n_vert = c_mesh->n_vertices();
		} while (pre_n_vert != cur_n_vert);
		std::cout << "	- Remove " << n_vert_pre - c_mesh->n_vertices() << " non-manifold vertices." << std::endl;

		// process all useful properties
		easy3d::PointCloud* fd_cen_pcl = new easy3d::PointCloud;
		for (auto fd : mesh_merged->faces())
		{
			fd_cen_pcl->add_vertex(mesh_merged->get_face_center[fd]);
		}
		easy3d::KdTree tree_3d;
		tree_3d.begin();
		tree_3d.add_point_cloud(fd_cen_pcl);
		tree_3d.end();

		std::map<int, bool> used_tex_count;
		for (int i = 0; i < c_mesh->textures.size(); ++i)
			used_tex_count[i] = false;

		auto get_point_coord = c_mesh->get_vertex_property<easy3d::vec3>("v:point");
		for (auto& fd : c_mesh->faces())
		{
			easy3d::vec3 fd_cen(0.0f, 0.0f, 0.0f);
			for (auto& vd : c_mesh->vertices(fd))
			{
				fd_cen += get_point_coord[vd];
			}
			fd_cen /= 3.0f;

			int f_i = tree_3d.find_closest_point(fd_cen);
			SFMesh::Face fd_orig(f_i);
			if (with_texture)
			{
				c_mesh->get_face_texnumber[fd] = mesh_merged->get_face_texnumber[fd_orig];
				c_mesh->get_face_texcoord[fd] = mesh_merged->get_face_texcoord[fd_orig];
				used_tex_count[c_mesh->get_face_texnumber[fd]] = true;
			}
			c_mesh->get_face_truth_label[fd] = mesh_merged->get_face_truth_label[fd_orig];
			//c_mesh->get_face_area[fd] = mesh_merged->get_face_area[fd_orig];
			//c_mesh->get_face_normals[fd] = mesh_merged->get_face_normals[fd_orig];
			//c_mesh->get_face_center[fd] = mesh_merged->get_face_center[fd_orig];
			c_mesh->get_face_segment_id[fd] = mesh_merged->get_face_segment_id[fd_orig];
			c_mesh->get_face_color[fd] = mesh_merged->get_face_color[fd_orig];
		}

		// shift texture number
		if (with_texture)
		{
			c_mesh->textures.clear();
			c_mesh->texture_names.clear();

			std::map<int, int> used_tex_shifted_c;
			for (auto u_m : used_tex_count)
			{
				if (u_m.second)
				{
					used_tex_shifted_c[u_m.first] = u_m.first - c_mesh->textures.size();
					c_mesh->textures.push_back(mesh_merged->textures[u_m.first]);
					c_mesh->texture_names.push_back(mesh_merged->texture_names[u_m.first]);
				}
			}

			if (used_tex_shifted_c.size() != used_tex_count.size())
			{
				for (auto& fd : c_mesh->faces())
				{
					c_mesh->get_face_texnumber[fd] -= used_tex_shifted_c[c_mesh->get_face_texnumber[fd]];
				}
			}
		}

		delete fd_cen_pcl;
		return c_mesh;
	}

	std::pair<std::string, int> get_main_class(SFMesh* mesh_merged, std::vector<SFMesh::Face>& geo_component_face_i, std::vector<int> &output_index)
	{
		// count component labels
		std::map<int, int> label_count;
		for (auto fd : geo_component_face_i)
		{
			int f_li = mesh_merged->get_face_truth_label[fd] - 1;
			if (label_count.find(f_li) == label_count.end())
				label_count[f_li] = 1;
			else
				label_count[f_li] += 1;
		}

		// find max count label
		std::pair<int, int> max_label_count(-1, -1);
		for (auto l_pair : label_count)
		{
			if (l_pair.second > max_label_count.second)
			{
				max_label_count.first = l_pair.first;
				max_label_count.second = l_pair.second;
			}
		}

		// match component name
		std::string cur_label_name = labels_name[max_label_count.first];
		int used_name_i = -1;
		for (int i = 0; i < component_label_name.size(); ++i)
		{
			for (int j = 0; j < component_label_name[i].size(); ++j)
			{
				int res = cur_label_name.compare(component_label_name[i][j]);
				if (res == 0)
				{
					used_name_i = i;
					break;
				}
			}
		}

		std::string main_c_name = merged_component_name[used_name_i];
		output_index[used_name_i] += 1;
		auto name_ind_pair = std::make_pair(main_c_name, output_index[used_name_i]);
		return name_ind_pair;
	}

	void compute_feature_diversity
	(
		std::vector<float> &feas_var
	)
	{
		if (use_batch_processing)
		{
			//Batch separation
			std::vector<std::vector<std::pair<int, std::string>>> all_batches;
			read_txt_batches(all_batches);
			;
			//Batch processing
			for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
			{
				std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
				feature_diversity_process_batch_tiles
				(
					feas_var,
					all_batches[bi],
					bi
				);
			}
		}
		else
		{
			std::vector<float> tiles_feas_var;
			for (std::size_t pi = 0; pi < base_names.size(); ++pi)
			{
				feature_diversity_process_single_tiles
				(
					feas_var,
					pi
				);
			}
		}
	}

	//--- compute mesh area ---
	void compute_mesh_area
	(
		std::vector<std::pair<std::string, float>> &mesh_area,
		std::vector<std::pair<std::string, std::vector<float>>> &mesh_class_area
	)
	{
		if (use_batch_processing)
		{
			//Batch separation
			std::vector<std::vector<std::pair<int, std::string>>> all_batches;
			read_txt_batches(all_batches);

			//Batch processing
			for (std::size_t bi = 0; bi < all_batches.size(); ++bi)
			{
				std::cout << " Not supported yet! " << std::endl;
				break;
				std::cout << " ********* Processing batch: " << bi << " *********" << std::endl;
				for (auto tile_i : all_batches[bi])
				{
					SFMesh *smesh_tmp = new SFMesh;
					//--- read mesh *.ply data ---
					read_mesh_data(smesh_tmp, tile_i.first);
					mesh_area.push_back(std::make_pair(tile_i.second, smesh_tmp->mesh_area));
					mesh_class_area.push_back(std::make_pair(tile_i.second, smesh_tmp->class_area));
					std::cout << tile_i.second << " area: " << std::fixed << std::showpoint << std::setprecision(6)
						<< smesh_tmp->mesh_area << std::endl;
					delete smesh_tmp;
				}
			}
		}
		else
		{
			for (std::size_t pi = 0; pi < base_names.size(); ++pi)
			{
				SFMesh *tmp_mesh = new SFMesh;
				read_mesh_data(tmp_mesh, pi);
				mesh_area.push_back(std::make_pair(base_names[pi], tmp_mesh->mesh_area));
				mesh_class_area.push_back(std::make_pair(base_names[pi], tmp_mesh->class_area));
				std::cout << base_names[pi] << " area: " << std::fixed << std::showpoint << std::setprecision(6)
					<< tmp_mesh->mesh_area << std::endl;
				delete tmp_mesh;
			}
		}
	}

}