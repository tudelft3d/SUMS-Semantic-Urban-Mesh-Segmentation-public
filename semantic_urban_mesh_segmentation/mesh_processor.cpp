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
		if (processing_mode == 2)
			temp_path << root_path << folder_names_level_0[11] << folder_names_level_0[2] << folder_names_level_1[0];
		else
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
		if (processing_mode == 2)
			temp_path << root_path << folder_names_level_0[11] << folder_names_level_0[2] << folder_names_level_1[1];
		else
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
		if (processing_mode == 2)
			temp_path << root_path << folder_names_level_0[11] << folder_names_level_0[2] << folder_names_level_1[3];
		else
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
		if (processing_mode == 2)
		{
			in_folder_path = root_path + folder_names_level_0[11] + folder_names_level_0[1] + folder_names_level_1[0];
			out_folder_path = root_path + folder_names_level_0[11] + folder_names_level_0[1] + folder_names_level_1[4];
		}
			
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
		std::map<std::pair<int, int>, std::pair<float, bool>> &spf_edges_maps // key: seg id pairs; value: common length, visited flag
	)
	{
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
				id_1 = smesh_in->get_face_segment_id[smesh_in->face(h0)];
				cmp_id_1 = smesh_in->get_face_component_id[smesh_in->face(h0)];
			}

			SFMesh::Halfedge h1 = smesh_in->halfedge(ei, 1);
			if (!smesh_in->is_boundary(h1))
			{
				id_2 = smesh_in->get_face_segment_id[smesh_in->face(h1)];
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
			construct_superfacet_neighbors(tmp_mesh, spf_edges_maps);

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
			construct_superfacet_neighbors(tmp_mesh, spf_edges_maps);

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
			if (smesh_original->get_face_truth_label[f] == smesh_original->get_face_predict_label[f]
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