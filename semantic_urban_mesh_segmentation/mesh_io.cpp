/*
*   Name        : mesh_io.cpp
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

#include "mesh_io.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	//--------------------------------------------Read data -------------------------------------------------//
	//visited all files and folder
	/*
	path:	input folder path
	files:	 files in the folder or subfolder
	*/
	void getAllFiles(const std::string& folder_path, const std::string&file_format, std::vector<std::string> &file_names, std::vector<std::string> &folder_names)
	{
		_finddata_t file;
		std::string filename = folder_path + "*.*";// file_format;//"\\*.jpg"
		intptr_t flag = _findfirst(filename.c_str(), &file);
		std::map<std::string, bool> is_folder_visited;
		if (flag == -1L)
		{
			std::cerr << "There is no such folder or file." << std::endl;
			throw std::exception();
		}
		else
		{
			do
			{
				//if subdirectory
				if (file.attrib & _A_SUBDIR)
				{
					if ((strcmp(file.name, ".") != 0) && (strcmp(file.name, "..") != 0))
					{
						std::string newPath = folder_path + file.name + "/";
						getAllFiles(newPath, file_format, file_names, folder_names);
					}
				}
				else
				{
					//separate char
					char *delim = ".";
					char *tmp = strtok(file.name, delim);
					bool is_input_format = false;
					while (tmp != NULL)
					{
						if (tmp == file_format)
							is_input_format = true;
						tmp = strtok(NULL, delim);
					}

					std::string file_name;
					if (is_input_format)
					{
						file_name = folder_path + file.name + delim + file_format;
						file_names.push_back(file_name);

						auto it = is_folder_visited.find(folder_path);
						if (it == is_folder_visited.end())
						{
							is_folder_visited[folder_path] = true;
							folder_names.push_back(folder_path);
						}
					}
				}
			} while (_findnext(flag, &file) == 0);
		}
		_findclose(flag);
	}

	void rply_input
	(
		SFMesh* smesh_out,
		char* file
	)
	{
		const double t_total = omp_get_wtime();
		bool success = easy3d::MeshIO::load(file, smesh_out);

		//Detected non-manifold vertices, To Do!!!!

		if (success)
			std::cout << "mesh read successful, loading time: " << omp_get_wtime() - t_total << "s" << std::endl;
		else
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
	}

	void read_pointcloud_data
	(
		PTCloud* pcl_out,
		const int sampled_or_ele,//0: sampled; 1: ele
		const int mi
	)
	{
		if (sampled_or_ele == 0)
			std::cout << "Start to reading " << prefixs[2] << " pointcloud " << base_names[mi] << std::endl;
		else
			std::cout << "Start to reading " << prefixs[10] << " pointcloud " << base_names[mi] << std::endl;

		std::string temp_str;
		if (sampled_or_ele == 0)
			temp_str = prefixs[2];
		else
			temp_str = prefixs[10];

		std::ostringstream str_ostemp;
		str_ostemp
			<< root_path
			<< folder_names_level_0[5]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[0]
			<< temp_str
			<< ".ply";

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();
		easy3d::PointCloud* pcl_temp = easy3d::PointCloudIO::load(Path_temp);
		if (pcl_temp == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}

		pcl_out->get_points_coord = pcl_out->get_vertex_property<vec3>("v:point");
		pcl_out->add_vertex_property<vec3>("v:normal");
		pcl_out->get_points_normals = pcl_out->get_vertex_property<vec3>("v:normal");
		if (sampled_or_ele != 1)
		{
			pcl_out->add_vertex_property<vec3>("v:color");
			pcl_out->get_points_color = pcl_out->get_vertex_property<vec3>("v:color");
		}

		for (auto ptx : pcl_temp->vertices())
		{
			pcl_out->add_vertex(pcl_temp->get_vertex_property<vec3>("v:point")[ptx]);
			pcl_out->get_points_coord[ptx] = pcl_temp->get_vertex_property<vec3>("v:point")[ptx];
			pcl_out->get_points_color[ptx] = pcl_temp->get_vertex_property<vec3>("v:color")[ptx];
			pcl_out->get_points_face_belong_id[ptx] = pcl_temp->get_vertex_property<int>("v:points_face_belong_id")[ptx];
			pcl_out->get_points_ground_truth[ptx] = pcl_temp->get_vertex_property<int>("v:label")[ptx];
		}
		delete pcl_temp;
	}

	void read_pointcloud_data
	(
		SFMesh *smesh_in,
		PTCloud* pcl_out,
		const int sampled_or_ele,//0: sampled; 1: ele
		const int mi
	)
	{
		if (sampled_or_ele == 0)
			std::cout << "Start to reading " << prefixs[2] << " pointcloud " << base_names[mi] << std::endl;
		else
			std::cout << "Start to reading " << prefixs[10] << " pointcloud " << base_names[mi] << std::endl;

		std::string temp_str;
		if (sampled_or_ele == 0)
			temp_str = prefixs[2];
		else
			temp_str = prefixs[10];

		std::ostringstream str_ostemp;
		if (processing_mode == 0 || (processing_mode == 1 && sota_folder_path == "PSSNet/")) //RF
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[5]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[mi]
				<< prefixs[0]
				<< temp_str
				<< ".ply";
		}
		else //SOTA
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[8]
				<< sota_folder_path
				<< folder_names_level_0[9]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[mi]
				<< prefixs[0]
				<< temp_str
				<< ".ply";
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();
		easy3d::PointCloud* pcl_temp = easy3d::PointCloudIO::load(Path_temp);
		if (pcl_temp == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}

		pcl_out->get_points_coord = pcl_out->get_vertex_property<vec3>("v:point");
		if (!pcl_out->get_vertex_property<vec3>("v:normal"))
			pcl_out->add_vertex_property<vec3>("v:normal", vec3());
		pcl_out->get_points_normals = pcl_out->get_vertex_property<vec3>("v:normal");
		if (sampled_or_ele != 1)
		{
			if (!pcl_out->get_vertex_property<vec3>("v:color"))
				pcl_out->add_vertex_property<vec3>("v:color", vec3());
			pcl_out->get_points_color = pcl_out->get_vertex_property<vec3>("v:color");
		}

		for (auto ptx : pcl_temp->vertices())
		{
			pcl_out->add_vertex(pcl_temp->get_vertex_property<vec3>("v:point")[ptx]);
			pcl_out->get_points_coord[ptx] = pcl_temp->get_vertex_property<vec3>("v:point")[ptx];
			if (sampled_or_ele != 1)
			{
				//for sampled point cloud
				pcl_out->get_points_color[ptx] = pcl_temp->get_vertex_property<vec3>("v:color")[ptx];
				if (processing_mode == 0 || processing_mode == 2) //RF
				{
					pcl_out->get_points_rgb_x[ptx] = pcl_temp->get_vertex_property<float>("v:points_rgb_x")[ptx];
					pcl_out->get_points_rgb_y[ptx] = pcl_temp->get_vertex_property<float>("v:points_rgb_y")[ptx];
					pcl_out->get_points_rgb_z[ptx] = pcl_temp->get_vertex_property<float>("v:points_rgb_z")[ptx];
					pcl_out->get_points_hsv_x[ptx] = pcl_temp->get_vertex_property<float>("v:points_hsv_x")[ptx];
					pcl_out->get_points_hsv_y[ptx] = pcl_temp->get_vertex_property<float>("v:points_hsv_y")[ptx];
					pcl_out->get_points_hsv_z[ptx] = pcl_temp->get_vertex_property<float>("v:points_hsv_z")[ptx];
				}

				pcl_out->get_points_face_belong_id[ptx] = pcl_temp->get_vertex_property<int>("v:points_face_belong_id")[ptx];
				int fi = pcl_out->get_points_face_belong_id[ptx];
				SFMesh::Face fdx(fi);

				pcl_out->get_points_normals[ptx] = smesh_in->get_face_normals[fdx];
				if (smesh_in->get_face_truth_label)
					pcl_out->get_points_ground_truth[ptx] = smesh_in->get_face_truth_label[fdx];

				pcl_out->get_points_tile_index[ptx] = smesh_in->get_face_tile_index[fdx];

				if (smesh_in->is_boundary(fdx))
					pcl_out->get_points_on_mesh_border[ptx] = true;

				if (sampled_or_ele == 0)
				{
					if (sampling_strategy != -1 && sampling_strategy != 2 && sampling_strategy != 4)
					{
						smesh_in->get_face_sampled_points_id_index_map[fdx][ptx.idx()] = smesh_in->get_face_sampled_points[fdx].size();
						smesh_in->get_face_sampled_points[fdx].push_back(ptx.idx());
					}
					else
					{
						pcl_out->get_points_face_belong_ids[ptx] = pcl_temp->get_vertex_property<std::vector<int>>("v:points_face_belong_ids")[ptx];
						for (auto fi2 : pcl_out->get_points_face_belong_ids[ptx])
						{
							SFMesh::Face fdx2(fi2);
							smesh_in->get_face_sampled_points_id_index_map[fdx2][ptx.idx()] = smesh_in->get_face_sampled_points[fdx2].size();
							smesh_in->get_face_sampled_points[fdx2].push_back(ptx.idx());
						}
					}
				}
			}
			else
			{
				//for elevation point cloud
				pcl_out->get_points_face_belong_id[ptx] = pcl_temp->get_vertex_property<int>("v:points_face_belong_id")[ptx];
				pcl_out->get_points_face_ele_belong_ids[ptx] = pcl_temp->get_vertex_property<std::vector<int>>("v:points_face_ele_belong_ids")[ptx];
				int fi = pcl_out->get_points_face_belong_id[ptx];
				SFMesh::Face fdx(fi);
				if (!pcl_out->get_points_face_ele_belong_ids[ptx].empty())
				{
					vec3 temp_point_normal(0.0f, 0.0f, 0.0f);
					float area_accu = 0.0f;
					for (int i = 0; i < pcl_out->get_points_face_ele_belong_ids[ptx].size(); ++i)
					{
						int fi_2 = pcl_out->get_points_face_ele_belong_ids[ptx][i];
						SFMesh::Face fdx2(fi_2);

						//pcl_out->get_points_tile_index[ptx] = smesh_in->get_face_tile_index[fdx];
						area_accu += smesh_in->get_face_area[fdx2];
						vec3 temp_nw = smesh_in->get_face_normals[fdx2] * smesh_in->get_face_area[fdx2];
						temp_point_normal.x += temp_nw.x;
						temp_point_normal.y += temp_nw.y;
						temp_point_normal.z += temp_nw.z;

						smesh_in->get_face_ele_sampled_points_id_index_map[fdx2][ptx.idx()] = smesh_in->get_face_ele_sampled_points[fdx2].size();
						smesh_in->get_face_ele_sampled_points[fdx2].push_back(ptx.idx());
					}
					temp_point_normal /= area_accu;
					pcl_out->get_points_normals[ptx] = temp_point_normal;
				}
				else
				{
					pcl_out->get_points_normals[ptx] = smesh_in->get_face_normals[fdx];
				}
			}
		}
		delete pcl_temp;
	}

	easy3d::PointCloud* read_feature_pointcloud_data
	(
		const std::string s1_test
	)
	{
		std::ostringstream str_ostemp;
		if (enable_augment && train_test_predict_val == 0 && !use_GCN_features)
		{
			std::cout << "	Start to read augmented features " << s1_test << std::endl;
			if (processing_mode == 2 || current_mode == operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
				str_ostemp
				<< root_path
				<< folder_names_level_0[11]
				<< folder_names_level_0[1]
				<< folder_names_level_1[4]
				<< s1_test
				<< prefixs[0]
				<< prefixs[3]
				<< prefixs[14]
				<< ".ply";
			else
				str_ostemp
				<< root_path
				<< folder_names_level_0[1]
				<< folder_names_level_1[4]
				<< s1_test
				<< prefixs[0]
				<< prefixs[3]
				<< prefixs[14]
				<< ".ply";
		}
		else
		{
			std::cout << "	Start to read features " << s1_test << std::endl;

			if (processing_mode == 2 || current_mode == operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
			{
				if (use_GCN_features)
					str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_pssnet[6]
						<< folder_names_level_1[train_test_predict_val]
						<< s1_test
						<< prefixs[0]
						<< prefixs[3]
						<< ".ply";
				else
					str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_level_0[1]
						<< folder_names_level_1[train_test_predict_val]
						<< s1_test
						<< prefixs[0]
						<< prefixs[3]
						<< ".ply";
			}
			else
			{
				str_ostemp
					<< root_path
					<< folder_names_level_0[1]
					<< folder_names_level_1[train_test_predict_val]
					<< s1_test
					<< prefixs[0]
					<< prefixs[3]
					<< ".ply";
			}
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		return(easy3d::PointCloudIO::load(Path_temp));
	}

	void read_mesh_data
	(
		SFMesh *smesh_out,
		const int mi,
		std::vector<cv::Mat> &texture_maps,
		const int batch_index
	)
	{
		//get over-segmented mesh
		std::ostringstream mesh_seg_str_ostemp;
		SFMesh *mesh_seg = new SFMesh;
		if (use_existing_mesh_segments)
		{
			if (current_mode == operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
				partition_folder_path = folder_names_level_0[11] + folder_names_pssnet[1];

			if (!use_batch_processing)
			{
				mesh_seg_str_ostemp
					<< root_path
					<< partition_folder_path
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< partition_prefixs
					<< ".ply";

				std::cout << "loading existing mesh segments: " << base_names[mi] << std::endl;
			}
			else
			{
				mesh_seg_str_ostemp
					<< root_path
					<< partition_folder_path
					<< folder_names_level_1[train_test_predict_val]
					<< prefixs[9]
					<< mi
					<< partition_prefixs
					<< ".ply";
				std::cout << "loading existing mesh segments: " << prefixs[9] << mi << std::endl;
			}

			std::string mesh_seg_str_temp = mesh_seg_str_ostemp.str().data();
			char * mesh_seg_Path_temp = (char *)mesh_seg_str_temp.data();
			rply_input(mesh_seg, mesh_seg_Path_temp);
		}

		//get original mesh
		std::cout << "loading mesh: " << base_names[mi] << std::endl << std::endl;

		std::ostringstream mesh_orig_temp_os;
		mesh_orig_temp_os
			<< ply_files[mi];
		std::string mesh_orig_temp_str = mesh_orig_temp_os.str().data();
		char * mesh_orig_temp_char = (char *)mesh_orig_temp_str.data();
		rply_input(smesh_out, mesh_orig_temp_char);

		add_mesh_properties_from_input(smesh_out, mi, mesh_seg, texture_maps, batch_index);

		std::cout << "  The total number of input triangle facets:  " << smesh_out->faces_size() << '\n' << std::endl;
		std::cout << "  The total number of input edges:  " << smesh_out->edges_size() << '\n' << std::endl;
		std::cout << "  The total number of input vertices:  " << smesh_out->vertices_size() << '\n' << std::endl;
		std::cout << "  The total number of input texture image:  " << texture_maps.size() << '\n' << std::endl;
		delete mesh_seg;
	}

	void read_labeled_mesh_data
	(
		SFMesh *smesh_out,
		const int mi
	)
	{
		std::cout << "loading training mesh: " << ply_files[mi] << std::endl;
		std::ostringstream mesh_str_ostemp;

		//read .ply mesh
		if (processing_mode != 2)
		{
			mesh_str_ostemp << ply_files[mi];
		}
		else
		{
			std::string basic_write_path, pref_tmp;
			if (train_test_predict_val == 0)
			{
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[2] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[15];

			}
			else
			{
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}

			if (use_batch_processing)
			{
				mesh_str_ostemp
					<< basic_write_path
					<< prefixs[9] + std::to_string(mi)
					<< pref_tmp
					<< ".ply";
				std::cout << "	Loading semantic mesh: " << prefixs[9] + std::to_string(mi) << std::endl;
			}
			else
			{
				mesh_str_ostemp
					<< basic_write_path
					<< base_names[mi]
					<< pref_tmp
					<< ".ply";

				std::cout << "	Loading semantic mesh: " << base_names[mi] << std::endl;
			}
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		rply_input(smesh_out, meshPath_temp);

		//mesh vertex properties
		smesh_out->get_points_coord = smesh_out->get_vertex_property<vec3>("v:point");

		//mesh face properties
		smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:" + label_definition);
		if (!smesh_out->get_face_property<vec3>("f:normal"))
			smesh_out->add_face_property<vec3>("f:normal");
		smesh_out->get_face_normals = smesh_out->get_face_property<vec3>("f:normal");

		//get texture information
		if (!smesh_out->get_face_property<vec3>("f:color"))
			smesh_out->add_face_property<vec3>("f:color", vec3());
		smesh_out->get_face_color = smesh_out->get_face_property<vec3>("f:color");

		//read .jpg texture image
		if (!smesh_out->textures.empty())
		{
			if (!smesh_out->get_face_property<int>("f:texnumber"))
				smesh_out->add_face_property<int>("f:texnumber", 0);
			smesh_out->get_face_texnumber = smesh_out->get_face_property<int>("f:texnumber");

			if (!smesh_out->get_face_property<std::vector<float>>("f:texcoord"))
				smesh_out->add_face_property<std::vector<float>>("f:texcoord", std::vector<float>({ 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f }));
			smesh_out->get_face_texcoord = smesh_out->get_face_property<std::vector<float>>("f:texcoord");
		}

		//mesh face properties
		if (train_test_predict_val == 0)
		{
			if (smesh_out->get_face_property<int>("f:label"))
				smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:label");
			else
			{
				smesh_out->add_face_property<int>("f:label", -1);
				smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:label");
			}
		}
		else
		{
			smesh_out->get_face_predict_label = smesh_out->get_face_property<int>("f:face_predict");

			if (smesh_out->get_face_property<float>("f:label_probabilities"))
				smesh_out->get_face_predict_prob = smesh_out->get_face_property<float>("f:label_probabilities");
			else
			{
				smesh_out->add_face_property<float>("f:label_probabilities", 0.0f);
				smesh_out->get_face_predict_prob = smesh_out->get_face_property<float>("f:label_probabilities");
			}
		}

		for (auto f : smesh_out->faces())
		{
			smesh_out->get_face_normals[f] = smesh_out->compute_face_normal(f);
			smesh_out->get_face_area[f] = smesh_out->FaceArea(f);
			for (auto vd : smesh_out->vertices(f))
			{
				smesh_out->get_face_center[f] += smesh_out->get_points_coord[vd];
			}
			smesh_out->get_face_center[f] /= 3.0f;
		}

		std::cout << "  The total number of input triangle facets:  " << smesh_out->faces_size() << '\n' << std::endl;
		std::cout << "  The total number of input edges:  " << smesh_out->edges_size() << '\n' << std::endl;
		std::cout << "  The total number of input vertices:  " << smesh_out->vertices_size() << '\n' << std::endl;
		std::cout << "  The total number of input texture image:  " << smesh_out->textures.size() << '\n' << std::endl;
	}

	easy3d::PointCloud* read_semantic_pointcloud_data
	(
		const int pi
	)
	{
		std::cout << "Start to reading semantic point cloud " << base_names[pi] << std::endl;
		std::ostringstream str_ostemp;
		if (sota_folder_path == "PSSNet/")
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[11]
				<< folder_names_pssnet[8]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[pi]
				<< sota_prefixs
				<< ".ply";
		}
		else
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[8]
				<< sota_folder_path
				<< folder_names_level_0[10]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[pi]
				<< sota_prefixs
				<< ".ply";
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();
		return(easy3d::PointCloudIO::load(Path_temp));
	}

	void read_test_mesh_data
	(
		SFMesh *smesh_out,
		const int mi
	)
	{
		std::ostringstream mesh_str_ostemp;

		//read .ply mesh
		std::string basic_write_path, pref_tmp;
		if (processing_mode == 0) //RF
		{
			std::cout << "loading RF test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 1 && sota_folder_path != "PSSNet/") //SOTA
		{
			std::cout << "loading SOTA test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];

			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 2 || sota_folder_path == "PSSNet/") //PSSNet
		{
			if (previous_mode != operating_mode::Process_semantic_pcl)
			{
				std::cout << "loading PSSNet step-1 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
			else
			{
				std::cout << "loading PSSNet step-2 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_pssnet[8] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
		}

		if (use_batch_processing)
		{
			mesh_str_ostemp
				<< basic_write_path
				<< prefixs[9] + std::to_string(mi)
				<< pref_tmp
				<< ".ply";
			std::cout << "Reading testing mesh: " << prefixs[9] + std::to_string(mi) << std::endl;
		}
		else
		{
			mesh_str_ostemp
				<< basic_write_path
				<< base_names[mi]
				<< pref_tmp
				<< ".ply";
			std::cout << "Reading testing mesh: " << base_names[mi] << std::endl;
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		rply_input(smesh_out, meshPath_temp);

		//mesh vertex properties
		smesh_out->get_points_coord = smesh_out->get_vertex_property<vec3>("v:point");

		//mesh face properties
		if (smesh_out->get_face_property<int>("f:" + label_definition))
			smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:" + label_definition);
		else
		{
			smesh_out->add_face_property<int>("f:" + label_definition, -1);
			smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:" + label_definition);
		}

		if (smesh_out->get_face_property<int>("f:face_predict"))
			smesh_out->get_face_predict_label = smesh_out->get_face_property<int>("f:face_predict");
		else
		{
			smesh_out->add_face_property<int>("f:face_predict", -1);
			smesh_out->get_face_predict_label = smesh_out->get_face_property<int>("f:face_predict");
		}

		if (smesh_out->get_face_property<vec3>("f:color"))
			smesh_out->get_face_color = smesh_out->get_face_property<vec3>("f:color");
		else
		{
			smesh_out->add_face_property<vec3>("f:color", vec3());
			smesh_out->get_face_color = smesh_out->get_face_property<vec3>("f:color");
		}

		//get texture information
		if (!smesh_out->textures.empty())
		{
			if (!smesh_out->get_face_property<int>("f:texnumber"))
				smesh_out->add_face_property<int>("f:texnumber", 0);
			smesh_out->get_face_texnumber = smesh_out->get_face_property<int>("f:texnumber");

			if (!smesh_out->get_face_property<std::vector<float>>("f:texcoord"))
				smesh_out->add_face_property<std::vector<float>>("f:texcoord", std::vector<float>({ 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f }));
			smesh_out->get_face_texcoord = smesh_out->get_face_property<std::vector<float>>("f:texcoord");
			for (auto tex_i : smesh_out->textures)
			{
				if (!tex_i.empty() && tex_i[tex_i.size() - 1] == '\r')
					tex_i.erase(tex_i.size() - 1);

				smesh_out->texture_names.push_back(tex_i);
			}
		}
		std::cout << "  The total number of input triangle facets:  " << smesh_out->faces_size() << '\n' << std::endl;
		std::cout << "  The total number of input edges:  " << smesh_out->edges_size() << '\n' << std::endl;
		std::cout << "  The total number of input vertices:  " << smesh_out->vertices_size() << '\n' << std::endl;
		//std::cout << "  The total number of input texture image:  " << texture_maps_temp.size() << '\n' << std::endl;
	}


	//For nodes attributes that are constructed
	void read_graph_nodes_ply
	(
		std::vector<superfacets> &spf_current,
		const int mi
	)
	{
		std::cout << " Reading graph nodes ";
		std::ostringstream temp;
		temp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[3]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[16] << "_nodes"
			<< ".ply";

		std::string str_temp = temp.str().data();
		char * Path_temp = (char *)str_temp.data();
		easy3d::PointCloud* spf_pcl = easy3d::PointCloudIO::load(Path_temp);

		int spf_ind = 0;
		for (auto ptx : spf_pcl->vertices())
		{
			superfacets spf_i;
			spf_i.id = spf_pcl->get_vertex_property<int>("v:segment_id")[ptx];
			spf_i.avg_center = spf_pcl->get_vertex_property<vec3>("v:point")[ptx];
			spf_i.plane_parameter =
				vec4(spf_pcl->get_vertex_property<std::vector<float>>("v:segment_plane_parameters")[ptx][0],
					spf_pcl->get_vertex_property<std::vector<float>>("v:segment_plane_parameters")[ptx][1],
					spf_pcl->get_vertex_property<std::vector<float>>("v:segment_plane_parameters")[ptx][2],
					spf_pcl->get_vertex_property<std::vector<float>>("v:segment_plane_parameters")[ptx][3]);
			spf_i.neighbor_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_neighbor_ids")[ptx];
			spf_i.label = spf_pcl->get_vertex_property<int>("v:segment_non_planar_label")[ptx];
			if (spf_i.label == 1)
				spf_i.is_non_planar = true;
			spf_i.local_ground_id = spf_pcl->get_vertex_property<int>("v:segment_local_ground_id")[ptx];
			spf_i.exterior_mat_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_exterior_ids")[ptx];

			spf_i.sampled_points_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_sampled_pts_ids")[ptx];
			for (int pi = 0; pi < spf_i.sampled_points_ids.size(); ++pi)
			{
				vec3 temp_p;
				temp_p.x = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_x")[ptx][pi];
				temp_p.y = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_y")[ptx][pi];
				temp_p.z = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_z")[ptx][pi];
				spf_i.sampled_pts_coords.push_back(temp_p);
			}

			spf_i.index = spf_ind;
			superfacet_id_index_map[spf_i.id] = spf_ind;
			spf_current.push_back(spf_i);
			++spf_ind;
		}
		std::cout << "= " << spf_current.size() << std::endl;
		delete spf_pcl;
	}

	//read two mesh: 1. segment mesh; 2. planar non-planar; 
	void read_seg_mesh_with_pnp_info
	(
		SFMesh* smesh_seg,
		const int mi
	)
	{
		//read over-segmentation result
		std::ostringstream mesh_str_ostemp;
		std::cout << "Start to get mesh  " << base_names[mi] << std::endl;
		//read .ply mesh
		mesh_str_ostemp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[1]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[4]
			<< prefixs[7]
			<< ".ply";

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		rply_input(smesh_seg, meshPath_temp);

		smesh_seg->get_points_coord = smesh_seg->get_vertex_property<vec3>("v:point");
		smesh_seg->get_face_segment_id = smesh_seg->get_face_property<int>("f:face_segment_id");
		if (!smesh_seg->get_face_property<vec3>("f:normal"))
			smesh_seg->add_face_property<vec3>("f:normal");
		smesh_seg->get_face_normals = smesh_seg->get_face_property<vec3>("f:normal");
		for (auto fi : smesh_seg->faces())
		{
			smesh_seg->get_face_normals[fi] = smesh_seg->compute_face_normal(fi);
			smesh_seg->get_face_area[fi] = smesh_seg->FaceArea(fi);
		}

		if (!use_batch_processing)
		{
			SFMesh *tmp_mesh = new SFMesh;
			read_labeled_mesh_data(tmp_mesh, mi);
			if (train_test_predict_val == 0)
			{
				if (!smesh_seg->get_face_property<int>("f:label"))
					smesh_seg->add_face_property<int>("f:label", -1);
				smesh_seg->get_face_truth_label = smesh_seg->get_face_property<int>("f:label");

#pragma omp parallel for schedule(dynamic)
				for (int fi = 0; fi < tmp_mesh->faces_size(); ++fi)
				{
					SFMesh::Face fdx(fi);
					smesh_seg->get_face_truth_label[fdx] = tmp_mesh->get_face_truth_label[fdx];
				}
			}
			else if (train_test_predict_val != 0)
			{
				if (!smesh_seg->get_face_property<int>("f:face_predict"))
					smesh_seg->add_face_property<int>("f:face_predict", -1);
				smesh_seg->get_face_predict_label = smesh_seg->get_face_property<int>("f:face_predict");
#pragma omp parallel for schedule(dynamic)
				for (int fi = 0; fi < tmp_mesh->faces_size(); ++fi)
				{
					SFMesh::Face fdx(fi);
					smesh_seg->get_face_predict_label[fdx] = tmp_mesh->get_face_predict_label[fdx];
				}
			}
			delete tmp_mesh;
		}
	}

	void read_and_write_oversegmentation_ground_truth()
	{
		std::ostringstream temp_path;
		temp_path 
			<< root_path 
			<< folder_names_level_0[2] 
			<< folder_names_level_1[train_test_predict_val];

		std::string temp_data_path = temp_path.str().data();
		std::vector<std::string> temp_ply_files;
		std::vector<std::string> temp_folder_names;
		getAllFiles(temp_data_path, file_formats[0], temp_ply_files, temp_folder_names);//get .ply filenames

		int R = rand() % 256;
		int G = rand() % 256;
		int B = rand() % 256;
		std::srand(std::time(0));
		for (int i = 0; i < base_names.size(); ++i)
		{
			std::cout << "Start to extract from mesh " << base_names[i] << std::endl;
			SFMesh* smesh_in = new SFMesh;
			std::ostringstream mesh_str_ostemp;
			//read .ply mesh
			mesh_str_ostemp
				<< temp_ply_files[i];

			std::string mesh_str_temp = mesh_str_ostemp.str().data();
			char * meshPath_temp = (char *)mesh_str_temp.data();
			rply_input(smesh_in, meshPath_temp);

			smesh_in->get_face_truth_label = smesh_in->get_face_property<int>("f:label");
			smesh_in->add_face_property<bool>("f:visited_check", false);
			auto get_face_visited_check = smesh_in->get_face_property<bool>("f:visited_check");
			smesh_in->get_face_color = smesh_in->get_face_property<vec3>("f:color");

			int seg_ind = 0;
			std::vector<std::vector<SFMesh::Face>> final_regions;

			for (auto f : smesh_in->faces())
			{
				std::vector<SFMesh::Face> current_region;
				if (!get_face_visited_check[f])
					get_face_visited_check[f] = true;
				else
					continue;

				int seed_label = smesh_in->get_face_truth_label[f];
				current_region.emplace_back(f);
				for (int ci = 0; ci < current_region.size(); ++ci)
				{
					SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_in->halfedges(current_region[ci]);
					SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
					do
					{
						SFMesh::Halfedge ho = smesh_in->opposite_halfedge(*h_fit);
						if (smesh_in->is_boundary(ho) == false)
						{
							SFMesh::Face l = smesh_in->face(ho);
							if (smesh_in->get_face_truth_label[l] == seed_label
								&& !get_face_visited_check[l])
							{
								get_face_visited_check[l] = true;
								current_region.emplace_back(l);
							}
						}
						++h_fit;
					} while (h_fit != h_end);
				}

				R = rand() % 256;
				G = rand() % 256;
				B = rand() % 256;

				for (int fi = 0; fi < current_region.size(); ++fi)
				{
					smesh_in->get_face_segment_id[current_region[fi]] = seg_ind;
					smesh_in->get_face_color[current_region[fi]] = vec3(R / 255.0f, G / 255.0f, B / 255.0f);
				}
				final_regions.push_back(current_region);
				++seg_ind;
			}

			std::cout << "final region = " << final_regions.size() << std::endl;

			std::ostringstream mesh_out;
			mesh_out
				<< root_path
				<< folder_names_level_0[11]
				<< folder_names_pssnet[5]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[i]
				<< prefixs[4]
				<< prefixs[7]
				<< prefixs[8]
				<< ".ply";

			std::string temp_out = mesh_out.str().data();
			char * meshPath_temp_out = (char *)temp_out.data();
			smesh_in->remove_common_non_used_properties();
			smesh_in->remove_non_used_properties_for_semantic_mesh();
			rply_output(smesh_in, meshPath_temp_out);

			delete smesh_in;
		}
	}

	void read_oversegmentation_testdata
	(
		SFMesh* test_mesh,
		const int mi
	)
	{
		std::cout << "Start to get test mesh  " << base_names[mi] << std::endl;
		std::ostringstream mesh_str_ostemp;
		//read .ply mesh
		mesh_str_ostemp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[1]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[4]
			<< prefixs[7]
			<< ".ply";

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		rply_input(test_mesh, meshPath_temp);

		test_mesh->get_points_coord = test_mesh->get_vertex_property<vec3>("v:point");
		test_mesh->get_face_segment_id = test_mesh->get_face_property<int>("f:face_segment_id");
		for (auto fi : test_mesh->faces())
			test_mesh->get_face_area[fi] = test_mesh->FaceArea(fi);
	}

	void read_oversegmentation_truthdata
	(
		SFMesh* smesh_in,
		const int mi
	)
	{
		std::cout << "Start to get truth mesh " << base_names[mi] << std::endl;
		std::ostringstream mesh_str_ostemp;
		//read .ply mesh

		mesh_str_ostemp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[5]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[4]
			<< prefixs[7]
			<< prefixs[8]
			<< ".ply";

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		rply_input(smesh_in, meshPath_temp);

		smesh_in->get_points_coord = smesh_in->get_vertex_property<vec3>("v:point");
		smesh_in->get_face_truth_label = smesh_in->get_face_property<int>("f:label");
		smesh_in->get_face_segment_id = smesh_in->get_face_property<int>("f:face_segment_id");
		for (auto fi : smesh_in->faces())
			smesh_in->get_face_area[fi] = smesh_in->FaceArea(fi);
	}

	//read batch names in *.txt
	void read_txt_batches(std::vector<std::vector<std::pair<int, std::string>>> &all_batches_out)
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Reading batch names." << std::endl;
		std::ostringstream batch_out;
		batch_out
			<< root_path
			<< prefixs[9]
			<< "names_"
			<< data_types[train_test_predict_val]
			<< ".txt";

		std::string str_temp = batch_out.str().data();
		std::ifstream fin;
		fin.exceptions(std::ios::failbit | std::ios::badbit);
		fin.open(str_temp.c_str());
		int line_number = 0, tile_count = 0;
		std::string line;
		std::vector<std::string> columns;
		while (fin.is_open() && !fin.eof())
		{
			std::vector<std::pair<int, std::string>> per_batch;
			std::getline(fin, line);
			if (!line.empty())
			{
				columns = Split(line, "\t", false);
				for (auto col : columns)
				{
					per_batch.emplace_back(std::make_pair(file_ind_map[col], col));
				}
				++tile_count;
			}
			all_batches_out.push_back(per_batch);
			++line_number;
		}

		fin.close();
		std::cout << "	batch (squared area) number = " << all_batches_out.size() << "; tiles = " << tile_count << std::endl;
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//read config *.txt
	void read_txt_config(const std::string &path)
	{
		std::cout << std::endl << "Reading configuration file: ";

		std::ostringstream batch_out;
		batch_out
			<< path
			<< "config.txt";

		std::string str_temp = batch_out.str().data();
		std::ifstream fin;
		fin.exceptions(std::ios::failbit | std::ios::badbit);
		fin.open(str_temp.c_str());

		if (!fin.good())
		{
			std::cout << " reading failed, there is no config.txt, the program will use the default param_nameeters. " << std::endl;
		}
		else
		{
			const double t_total = omp_get_wtime();
			int line_number = 0;
			std::string line;
			std::vector<std::string> columns;
			while (fin.is_open() && !fin.eof())
			{
				std::vector<std::pair<int, std::string>> per_batch;
				std::getline(fin, line);
				if (!line.empty())
				{
					columns = Split(line, "=", false);
					if (columns.size() == 1)
						continue;

					std::string param_name = columns[0];
					std::string param_value = columns[1];
					param_name.erase(remove_if(param_name.begin(), param_name.end(), isspace), param_name.end());
					param_value.erase(remove_if(param_value.begin(), param_value.end(), isspace), param_value.end());
					if (param_name == "operating_mode")
					{
						if (param_value != "default")
						{
							if (param_value == "Pipeline")
							{
								current_mode = operating_mode::Pipeline;
							}
							else if (param_value == "Mesh_feature_extraction")
							{
								current_mode = operating_mode::Mesh_feature_extraction;
							}
							else if (param_value == "Train_config")
							{
								current_mode = operating_mode::Train_config;
							}
							else if (param_value == "Test_config")
							{
								current_mode = operating_mode::Test_config;
							}
							else if (param_value == "Train_and_Test_config")
							{
								current_mode = operating_mode::Train_and_Test_config;
							}
							else if (param_value == "Data_evaluation_for_all_tiles_config")
							{
								current_mode = operating_mode::Data_evaluation_for_all_tiles_config;
							}
							else if (param_value == "Save_mesh_features_for_visualization")
							{
								current_mode = operating_mode::Save_mesh_features_for_visualization;
							}
							else if (param_value == "Class_statistics")
							{
								current_mode = operating_mode::Class_statistics;
							}
							else if (param_value == "Generate_semantic_sampled_points")
							{
								current_mode = operating_mode::Generate_semantic_sampled_points;
							}
							else if (param_value == "Feature_diversity_meaure")
							{
								current_mode = operating_mode::Feature_diversity_meaure;
							}
							else if (param_value == "Compute_mesh_area")
							{
								current_mode = operating_mode::Compute_mesh_area;
							}
							else if (param_value == "Moha_Verdi_SOTA_pipeline")
							{
								current_mode = operating_mode::Moha_Verdi_SOTA_pipeline;
							}
							else if (param_value == "Evaluation_SOTA")
							{
								current_mode = operating_mode::Evaluation_SOTA;
							}
							else if (param_value == "Get_labels_for_planar_non_planar_from_semantic")
							{
								current_mode = operating_mode::Get_labels_for_planar_non_planar_from_semantic;
							}

							else if (param_value == "PSSNet_oversegmentation")
							{
								current_mode = operating_mode::PSSNet_oversegmentation;
							}
							else if (param_value == "PSSNet_graph_construction")
							{
								current_mode = operating_mode::PSSNet_graph_construction;
							}
							else if (param_value == "PSSNet_pipeline_for_GCN")
							{
								current_mode = operating_mode::PSSNet_pipeline_for_GCN;
							}
							else if (param_value == "PSSNet_oversegmentation_evaluation")
							{
								current_mode = operating_mode::PSSNet_oversegmentation_evaluation;
							}
							else if (param_value == "PSSNet_pcl_generation_for_GCN")
							{
								current_mode = operating_mode::PSSNet_pcl_generation_for_GCN;
							}
						}
					}
					else if (param_name == "root_path")
					{
						if (param_value != "default")
							root_path = param_value;
					}
					else if (param_name == "seg_aug_py_path")
					{
						if (param_value != "default")
							seg_aug_py_path = param_value;
					}
					else if (param_name == "processing_mode")
					{
						if (param_value != "default")
							processing_mode = std::stoi(param_value);
					}
					else if (param_name == "process_train")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								process_data_selection["train"] = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								process_data_selection["train"] = false;
						}
					}
					else if (param_name == "process_test")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								process_data_selection["test"] = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								process_data_selection["test"] = false;
						}
					}
					else if (param_name == "process_predict")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								process_data_selection["predict"] = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								process_data_selection["predict"] = false;
						}
					}
					else if (param_name == "process_validate")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								process_data_selection["validate"] = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								process_data_selection["validate"] = false;
						}
					}
					else if (param_name == "use_binary")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_binary = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_binary = false;
						}
					}
					else if (param_name == "label_definition")
					{
						if (param_value != "default")
							label_definition = param_value;
					}
					else if (param_name == "labels_name")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							if (words.size() > 0)
							{
								labels_name.resize(words.size());
								for (int li = 0; li < labels_name.size(); ++li)
								{
									labels_name[li] = words[li];
								}
							}
						}
					}
					else if (param_name == "ignored_labels_name")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							if (words.size() > 0)
							{
								ignored_labels_name.resize(words.size());
								for (int li = 0; li < ignored_labels_name.size(); ++li)
								{
									ignored_labels_name[li] = words[li];
								}
							}
						}
					}
					else if (param_name == "labels_color")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ";", false);
							int i = 0;
							if (words.size() > 0)
							{
								labels_color.resize(words.size());
								for (auto words2 : words)
								{
									std::vector<std::string> words3 = Split(words2, ",", false);
									int j = 0;
									for (auto w : words3)
									{
										float c = std::stof(w);
										if (c > 1)
											c /= 255.0f;
										labels_color[i][j] = c;
										++j;
									}
									++i;
								}
							}
						}
					}
					else if (param_name == "labels_name_pnp")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							if (words.size() > 0)
							{
								labels_name_pnp.resize(words.size());
								for (int li = 0; li < labels_name_pnp.size(); ++li)
								{
									labels_name_pnp[li] = words[li];
								}
							}
						}
					}
					else if (param_name == "L1_to_L0_label_map")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							L1_to_L0_label_map.clear();
							int i = 0;
							for (auto words2 : words)
							{
								L1_to_L0_label_map[labels_name[i]] = std::stoi(words2);
								++i;
							}
						}
						assert(L1_to_L0_label_map.size() == labels_name.size());
					}
					else if (param_name == "labels_color_pnp")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ";", false);
							int i = 0;
							if (words.size() > 0)
							{
								labels_color_pnp.resize(words.size());
								for (auto words2 : words)
								{
									std::vector<std::string> words3 = Split(words2, ",", false);
									int j = 0;
									for (auto w : words3)
									{
										float c = std::stof(w);
										if (c > 1)
											c /= 255.0f;
										labels_color_pnp[i][j] = c;
										++j;
									}
									++i;
								}
							}
						}
					}
					else if (param_name == "generate_groundtruth_segments")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								generate_groundtruth_segments = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								generate_groundtruth_segments = false;
						}
					}
					else if (param_name == "only_evaluation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								only_evaluation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								only_evaluation = false;
						}
					}
					else if (param_name == "use_GCN_features")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_GCN_features = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_GCN_features = false;
						}
					}
					else if (param_name == "only_write_GCN_features")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								only_write_GCN_features = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								only_write_GCN_features = false;
						}
					}
					else if (param_name == "radius_default")
					{
						if (param_value != "default")
							radius_default = std::stof(param_value);
					}
					else if (param_name == "mrf_lambda_d")
					{
						if (param_value != "default")
							mrf_lambda_d = std::stof(param_value);
					}
					else if (param_name == "mrf_lambda_m")
					{
						if (param_value != "default")
							mrf_lambda_m = std::stof(param_value);
					}
					else if (param_name == "mrf_lambda_g")
					{
						if (param_value != "default")
							mrf_lambda_g = std::stof(param_value);
					}
					else if (param_name == "mrf_lambda_p")
					{
						if (param_value != "default")
							mrf_lambda_p = std::stof(param_value);
					}
					else if (param_name == "with_node_graphs")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								with_node_graphs = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								with_node_graphs = false;
						}
					}
					else if (param_name == "use_edges_between_boder_points")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_edges_between_boder_points = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_edges_between_boder_points = false;
						}
					}
					else if (param_name == "parallelism_relations")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								parallelism_relations = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								parallelism_relations = false;
						}
					}
					else if (param_name == "tolerance_angle")
					{
						if (param_value != "default")
							tolerance_angle = std::stof(param_value);
					}
					else if (param_name == "local_ground_relations")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								local_ground_relations = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								local_ground_relations = false;
						}
					}
					else if (param_name == "exterior_mat_relations")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								exterior_mat_relations = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								exterior_mat_relations = false;
						}
					}
					else if (param_name == "delaunay_relations_on_sampled_points")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								delaunay_relations_on_sampled_points = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								delaunay_relations_on_sampled_points = false;
						}
					}
					else if (param_name == "remove_close_vertices_for_delaunay_dis")
					{
						if (param_value != "default")
							remove_close_vertices_for_delaunay_dis = std::stof(param_value);
					}
					else if (param_name == "with_texture")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								with_texture = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								with_texture = false;
						}
					}
					else if (param_name == "use_pointcloud_region_growing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_pointcloud_region_growing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_pointcloud_region_growing = false;
						}
					}
					else if (param_name == "is_pointclouds_exist")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								is_pointclouds_exist = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								is_pointclouds_exist = false;
						}
					}
					else if (param_name == "enable_augment")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								enable_augment = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								enable_augment = false;
						}
					}
					else if (param_name == "augmented_data_exist")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								augmented_data_exist = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								augmented_data_exist = false;
						}
					}
					else if (param_name == "save_sampled_pointclouds")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_sampled_pointclouds = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_sampled_pointclouds = false;
						}
					}
					else if (param_name == "save_oversegmentation_mesh")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_oversegmentation_mesh = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_oversegmentation_mesh = false;
						}
					}
					else if (param_name == "save_tex_cloud")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_tex_cloud = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_tex_cloud = false;
						}
					}
					else if (param_name == "save_textures_in_predict")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_textures_in_predict = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_textures_in_predict = false;
						}
					}
					else if (param_name == "save_error_map")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_error_map = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_error_map = false;
						}
					}
					else if (param_name == "save_feature_importance")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								save_feature_importance = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								save_feature_importance = false;
						}
					}
					else if (param_name == "multi_scale_ele_radius")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							multi_scale_ele_radius.resize(words.size());
							int i = 0;
							for (auto words2 : words)
							{
								multi_scale_ele_radius[i] = std::stof(words2);
								++i;
							}
						}
					}
					else if (param_name == "hsv_bins")
					{
						if (param_value != "default")
						{
							std::vector<std::string> words = Split(param_value, ",", false);
							hsv_bins.resize(words.size());
							int i = 0;
							for (auto words2 : words)
							{
								hsv_bins[i] = std::stof(words2);
								++i;
							}
						}
					}
					else if (param_name == "mat_delta_convergance")
					{
						if (param_value != "default")
							mat_delta_convergance = std::stof(param_value);
					}
					else if (param_name == "mat_initialized_radius")
					{
						if (param_value != "default")
							mat_initialized_radius = std::stof(param_value);
					}
					else if (param_name == "mat_denoising_seperation_angle")
					{
						if (param_value != "default")
							mat_denoising_seperation_angle = std::stof(param_value);
					}
					else if (param_name == "mat_iteration_limit_number")
					{
						if (param_value != "default")
							mat_iteration_limit_number = std::stoi(param_value);
					}
					else if (param_name == "long_range_radius_default")
					{
						if (param_value != "default")
							long_range_radius_default = std::stof(param_value);
					}
					else if (param_name == "local_ground_segs")
					{
						if (param_value != "default")
							local_ground_segs = std::stoi(param_value);
					}
					else if (param_name == "sampling_point_density")
					{
						if (param_value != "default")
							sampling_point_density = std::stof(param_value);
					}
					else if (param_name == "ele_sampling_point_density")
					{
						if (param_value != "default")
							ele_sampling_point_density = std::stof(param_value);
					}
					else if (param_name == "use_existing_mesh_segments_on_training")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_existing_mesh_segments_on_training = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_existing_mesh_segments_on_training = false;
						}
					}
					else if (param_name == "use_existing_mesh_segments_on_testing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_existing_mesh_segments_on_testing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_existing_mesh_segments_on_testing = false;
						}
					}
					else if (param_name == "use_existing_mesh_segments_on_predicting")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_existing_mesh_segments_on_predicting = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_existing_mesh_segments_on_predicting = false;
						}
					}
					else if (param_name == "use_existing_mesh_segments_on_validation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_existing_mesh_segments_on_validation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_existing_mesh_segments_on_validation = false;
						}
					}
					else if (param_name == "use_face_pixels_color_aggregation_on_training")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_face_pixels_color_aggregation_on_training = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_face_pixels_color_aggregation_on_training = false;
						}
					}
					else if (param_name == "use_face_pixels_color_aggregation_on_testing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_face_pixels_color_aggregation_on_testing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_face_pixels_color_aggregation_on_testing = false;
						}
					}
					else if (param_name == "use_face_pixels_color_aggregation_on_predicting")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_face_pixels_color_aggregation_on_predicting = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_face_pixels_color_aggregation_on_predicting = false;
						}
					}
					else if (param_name == "use_face_pixels_color_aggregation_on_validation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_face_pixels_color_aggregation_on_validation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_face_pixels_color_aggregation_on_validation = false;
						}
					}
					else if (param_name == "use_merged_segments_on_training")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_merged_segments_on_training = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_merged_segments_on_training = false;
						}
					}
					else if (param_name == "use_merged_segments_on_testing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_merged_segments_on_testing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_merged_segments_on_testing = false;
						}
					}
					else if (param_name == "use_merged_segments_on_predicting")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_merged_segments_on_predicting = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_merged_segments_on_predicting = false;
						}
					}
					else if (param_name == "use_merged_segments_on_validation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_merged_segments_on_validation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_merged_segments_on_validation = false;
						}
					}
					else if (param_name == "adjacent_radius")
					{
						if (param_value != "default")
							adjacent_radius = std::stof(param_value);
					}
					else if (param_name == "adjacent_pt2plane_distance")
					{
						if (param_value != "default")
							adjacent_pt2plane_distance = std::stof(param_value);
					}
					else if (param_name == "adjacent_seg_angle")
					{
						if (param_value != "default")
							adjacent_seg_angle = std::stof(param_value);
					}
					else if (param_name == "mesh_distance_to_plane")
					{
						if (param_value != "default")
							mesh_distance_to_plane = std::stof(param_value);
					}
					else if (param_name == "mesh_accepted_angle")
					{
						if (param_value != "default")
							mesh_accepted_angle = std::stof(param_value);
					}
					else if (param_name == "mesh_minimum_region_size")
					{
						if (param_value != "default")
							mesh_minimum_region_size = std::stof(param_value);
					}
					else if (param_name == "used_k_neighbors")
					{
						if (param_value != "default")
							used_k_neighbors = std::stoi(param_value);
					}
					else if (param_name == "rf_tree_numbers")
					{
						if (param_value != "default")
							rf_tree_numbers = std::stoi(param_value);
					}
					else if (param_name == "rf_tree_depth")
					{
						if (param_value != "default")
							rf_tree_depth = std::stoi(param_value);
					}
					else if (param_name == "use_feas")
					{
						if (param_value != "default")
						{
							for (int i = 0; i < use_feas.size(); ++i)
								use_feas[i] = false;

							std::vector<std::string> words = Split(param_value, ",", false);
							for (auto words2 : words)
							{
								use_feas[std::stoi(words2)] = true;
							}
						}
					}
					else if (param_name == "use_mulsc_eles")
					{
						if (param_value != "default")
						{
							for (int i = 0; i < use_mulsc_eles.size(); ++i)
								use_mulsc_eles[i] = false;

							std::vector<std::string> words = Split(param_value, ",", false);
							for (auto words2 : words)
							{
								use_mulsc_eles[std::stoi(words2)] = true;
							}
						}
					}
					else if (param_name == "use_basic_features")
					{
						if (param_value != "default")
						{
							for (int i = 0; i < use_basic_features.size(); ++i)
								use_basic_features[i] = false;

							std::vector<std::string> words = Split(param_value, ",", false);
							for (auto words2 : words)
							{
								use_basic_features[std::stoi(words2)] = true;
							}
						}
					}
					else if (param_name == "use_eigen_features")
					{
						if (param_value != "default")
						{
							for (int i = 0; i < use_eigen_features.size(); ++i)
								use_eigen_features[i] = false;

							std::vector<std::string> words = Split(param_value, ",", false);
							for (auto words2 : words)
							{
								use_eigen_features[std::stoi(words2)] = true;
							}
						}
					}
					else if (param_name == "use_color_features")
					{
						if (param_value != "default")
						{
							for (int i = 0; i < use_color_features.size(); ++i)
								use_color_features[i] = false;

							std::vector<std::string> words = Split(param_value, ",", false);
							for (auto words2 : words)
							{
								use_color_features[std::stoi(words2)] = true;
							}
						}
					}
					else if (param_name == "input_type_for_statistics")
					{
						if (param_value != "default")
							input_type_for_statistics = std::stoi(param_value);
					}
					else if (param_name == "mr_facet_neg_sphericial_radius")
					{
						if (param_value != "default")
							mr_facet_neg_sphericial_radius = std::stof(param_value);
					}
					else if (param_name == "mr_limit_distance")
					{
						if (param_value != "default")
							mr_limit_distance = std::stof(param_value);
					}
					else if (param_name == "mr_angle_thres")
					{
						if (param_value != "default")
							mr_angle_thres = std::stof(param_value);
					}
					else if (param_name == "mr_L1_color_dis")
					{
						if (param_value != "default")
							mr_L1_color_dis = std::stof(param_value);
					}
					else if (param_name == "mr_max_sp_area")
					{
						if (param_value != "default")
							mr_max_sp_area = std::stof(param_value);
					}
					else if (param_name == "short_range_radius_default")
					{
						if (param_value != "default")
							short_range_radius_default = std::stof(param_value);
					}
					else if (param_name == "mrf_lambda_mh")
					{
						if (param_value != "default")
							mrf_lambda_mh = std::stof(param_value);
					}
					else if (param_name == "mrf_energy_amplify")
					{
						if (param_value != "default")
							mrf_energy_amplify = std::stof(param_value);
					}
					else if (param_name == "add_point_color_for_dp_input")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								add_point_color_for_dp_input = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								add_point_color_for_dp_input = false;
						}
					}
					else if (param_name == "sota_folder_path")
					{
						if (param_value != "default")
							sota_folder_path = param_value;
					}
					else if (param_name == "sota_prefixs")
					{
						if (sota_prefixs != "default")
							sota_prefixs = param_value;
					}
					else if (param_name == "partition_folder_path")
					{
						if (param_value != "default")
							partition_folder_path = param_value;
					}
					else if (param_name == "partition_prefixs")
					{
						if (param_value != "default")
							partition_prefixs = param_value;
					}
					else if (param_name == "label_string")
					{
						if (param_value != "default")
							label_string = param_value;
					}
					else if (param_name == "label_minus")
					{
						if (param_value != "default")
							label_minus = std::stoi(param_value);
					}
					else if (param_name == "equal_cloud")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								equal_cloud = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								equal_cloud = false;
						}
					}
					else if (param_name == "use_existing_splited_batch")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_existing_splited_batch = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_existing_splited_batch = false;
						}
					}
					else if (param_name == "use_batch_processing_on_training")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_batch_processing_on_training = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_batch_processing_on_training = false;
						}
					}
					else if (param_name == "use_batch_processing_on_testing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_batch_processing_on_testing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_batch_processing_on_testing = false;
						}
					}
					else if (param_name == "use_batch_processing_on_predicting")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_batch_processing_on_predicting = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_batch_processing_on_predicting = false;
						}
					}
					else if (param_name == "use_batch_processing_on_validation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_batch_processing_on_validation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_batch_processing_on_validation = false;
						}
					}
					else if (param_name == "batch_size")
					{
						if (param_value != "default")
							batch_size = std::stoi(param_value);
					}
					else if (param_name == "sub_batch_size")
					{
						if (param_value != "default")
							sub_batch_size = std::stoi(param_value);
					}
					else if (param_name == "sampling_strategy_training")
					{
						if (param_value != "default")
							sampling_strategy_training = std::stoi(param_value);
					}
					else if (param_name == "sampling_strategy_testing")
					{
						if (param_value != "default")
							sampling_strategy_testing = std::stoi(param_value);
					}
					else if (param_name == "sampling_strategy_predicting")
					{
						if (param_value != "default")
							sampling_strategy_predicting = std::stoi(param_value);
					}
					else if (param_name == "sampling_strategy_validation")
					{
						if (param_value != "default")
							sampling_strategy_validation = std::stoi(param_value);
					}
					else if (param_name == "use_pointcloud_region_growing_on_training")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_pointcloud_region_growing_on_training = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_pointcloud_region_growing_on_training = false;
						}
					}
					else if (param_name == "use_pointcloud_region_growing_on_testing")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_pointcloud_region_growing_on_testing = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_pointcloud_region_growing_on_testing = false;
						}
					}
					else if (param_name == "use_pointcloud_region_growing_on_predicting")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_pointcloud_region_growing_on_predicting = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_pointcloud_region_growing_on_predicting = false;
						}
					}
					else if (param_name == "use_pointcloud_region_growing_on_validation")
					{
						if (param_value != "default")
						{
							if (param_value == "true" || param_value == "True" || param_value == "TRUE")
								use_pointcloud_region_growing_on_validation = true;
							else if (param_value == "false" || param_value == "False" || param_value == "FALSE")
								use_pointcloud_region_growing_on_validation = false;
						}
					}
					else if (param_name == "pcl_distance_to_plane")
					{
						if (param_value != "default")
							pcl_distance_to_plane = std::stof(param_value);
					}
					else if (param_name == "pcl_accepted_angle")
					{
						if (param_value != "default")
							pcl_accepted_angle = std::stof(param_value);
					}
					else if (param_name == "pcl_minimum_region_size")
					{
						if (param_value != "default")
							pcl_minimum_region_size = std::stof(param_value);
					}
					else if (param_name == "pcl_k_nn")
					{
						if (param_value != "default")
							pcl_k_nn = std::stoi(param_value);
					}

				}

				++line_number;
			}

			std::cout << "Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
		}
		fin.close();
	}
	//--------------------------------------------Write data -------------------------------------------------//
	void rply_output
	(
		SFMesh* smesh_out,
		const char* file,
		const std::vector<std::string> &comment
	)
	{
		bool success = easy3d::MeshIO::save(file, smesh_out, comment);
		if (success)
			std::cout << "mesh saved" << std::endl;
		else
			std::cerr << "failed create the new file" << std::endl;
	}

	void rply_output
	(
		SFMesh* smesh_out,
		char* file
	)
	{
		bool success = easy3d::MeshIO::save(file, smesh_out, use_binary);
		if (success)
			std::cout << "mesh saved" << std::endl;
		else
			std::cerr << "failed create the new file" << std::endl;
	}

	void rply_output
	(
		PTCloud* pcl_out,
		char* file
	)
	{
		bool success = easy3d::PointCloudIO::save(file, pcl_out, use_binary);
		if (success)
			std::cout << "pointcloud saved" << std::endl;
		else
			std::cerr << "failed create the new file" << std::endl;
	}

	void write_pointcloud_data
	(
		PTCloud* pcl_out,
		const int sampled_or_ele,
		const int mi
	)
	{
		const double t_total = omp_get_wtime();
		//pcl_out->remove_non_used_properties();

		std::string temp_str;
		if (sampled_or_ele == 0)
		{
			std::cout << "	Start to writing sampled point cloud ..." << std::endl;
			temp_str = prefixs[2];
			pcl_out->remove_vertex_property(pcl_out->get_points_face_ele_belong_ids);
			if (sampling_strategy != 2 && sampling_strategy != 4)
				pcl_out->remove_vertex_property(pcl_out->get_points_face_belong_ids);
		}
		else
		{
			std::cout << "	Start to writing elevation sampled point cloud ..." << std::endl;
			temp_str = prefixs[10];
			pcl_out->remove_vertex_property(pcl_out->get_points_face_belong_ids);
		}

		std::ostringstream str_ostemp;
		if (processing_mode == 0 || processing_mode == 2)
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[5]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[mi]
				<< prefixs[0]
				<< temp_str
				<< ".ply";
		}
		else
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[8]
				<< sota_folder_path
				<< folder_names_level_0[9]
				<< folder_names_level_1[train_test_predict_val]
				<< base_names[mi]
				<< prefixs[0]
				<< temp_str
				<< ".ply";
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		rply_output(pcl_out, Path_temp);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void write_tex_pointcloud_data
	(
		PTCloud* pcl_out,
		const int mi
	)
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Start to writing data ..." << std::endl << std::endl;

		pcl_out->remove_all_properties();

		std::string temp_str = prefixs[11];
		pcl_out->remove_vertex_property(pcl_out->get_points_face_ele_belong_ids);

		std::ostringstream str_ostemp;
		str_ostemp
			<< root_path
			<< folder_names_level_0[5]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[0]
			<< temp_str
			<< ".ply";
		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		rply_output(pcl_out, Path_temp);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void write_feature_pointcloud_data
	(
		PTCloud* pcl_out,
		const std::string s1_test
	)
	{
		std::cout << "	- Start to writing feature point cloud: ";
		const double t_total = omp_get_wtime();
		pcl_out->remove_all_properties();
		pcl_out->remove_vertex_property(pcl_out->get_points_face_belong_id);
		pcl_out->remove_vertex_property(pcl_out->get_points_face_belong_ids);
		pcl_out->remove_vertex_property(pcl_out->get_points_face_ele_belong_ids);
		std::ostringstream str_ostemp;

		if (processing_mode == 0 && current_mode != operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[1]
				<< folder_names_level_1[train_test_predict_val]
				<< s1_test
				<< prefixs[0]
				<< prefixs[3]
				<< ".ply";
		}
		else if (processing_mode == 1 && current_mode != operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[8]
				<< sota_folder_path
				<< folder_names_level_0[1]
				<< folder_names_level_1[train_test_predict_val]
				<< s1_test
				<< prefixs[0]
				<< prefixs[3]
				<< ".ply";
		}
		else if (processing_mode == 2 || current_mode == operating_mode::PSSNet_pcl_generation_for_GCN_backbone)
		{
			if (use_GCN_features)
				str_ostemp
					<< root_path
					<< folder_names_level_0[11]
					<< folder_names_pssnet[6]
					<< folder_names_level_1[train_test_predict_val]
					<< s1_test
					<< prefixs[0]
					<< prefixs[3]
					<< ".ply";
			else
				str_ostemp
					<< root_path
					<< folder_names_level_0[11]
					<< folder_names_level_0[1]
					<< folder_names_level_1[train_test_predict_val]
					<< s1_test
					<< prefixs[0]
					<< prefixs[3]
					<< ".ply";
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		rply_output(pcl_out, Path_temp);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
		//delete pcl_out;
	}

	void write_feature_mesh_data
	(
		SFMesh* smesh_out,
		const int mi
	)
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Start to writing feature " << base_names[mi] << " for visualization in Mapple (https://3d.bk.tudelft.nl/liangliang/software/Mapple.zip) " << std::endl << std::endl;
		std::ostringstream str_ostemp;

		if (use_batch_processing)
		{
			str_ostemp
				<< root_path
				<< folder_names_level_0[7]
				<< folder_names_level_1[train_test_predict_val]
				<< prefixs[9] + std::to_string(mi)
				<< prefixs[4]
				<< prefixs[3]
				<< ".ply";
		}
		else
		{
			if (processing_mode == 0)
			{
				str_ostemp
					<< root_path
					<< folder_names_level_0[7]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< prefixs[4]
					<< prefixs[3]
					<< ".ply";
			}
			else if (processing_mode == 1)
			{
				str_ostemp
					<< root_path
					<< folder_names_level_0[8]
					<< sota_folder_path
					<< folder_names_level_0[7]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< prefixs[4]
					<< prefixs[3]
					<< ".ply";
			}
			else if (processing_mode == 2)
			{
				if (use_GCN_features)
					str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_pssnet[7]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[mi]
						<< prefixs[4]
						<< prefixs[3]
						<< ".ply";
				else
					str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_level_0[7]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[mi]
						<< prefixs[4]
						<< prefixs[3]
						<< ".ply";
			}
		}

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		rply_output(smesh_out, Path_temp);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void write_mesh_segments
	(
		SFMesh* smesh_out,
		const int mi,
		bool write_pnp
	)
	{
		const double t_total = omp_get_wtime();
		std::ostringstream mesh_str_ostemp;
		if (use_batch_processing)
		{
			if (processing_mode == 0)
			{
				mesh_str_ostemp
					<< root_path
					<< folder_names_level_0[6]
					<< folder_names_level_1[train_test_predict_val]
					<< prefixs[9] + std::to_string(mi)
					<< prefixs[4]
					<< prefixs[7]
					<< ".ply";
			}
			else if (processing_mode == 2)
			{
				if (write_pnp)
				{
					mesh_str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_pssnet[1]
						<< folder_names_level_1[train_test_predict_val]
						<< prefixs[9] + std::to_string(mi)
						<< prefixs[4]
						<< prefixs[7]
						<< ".ply";
				}
				else
				{
					mesh_str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_level_0[6]
						<< folder_names_level_1[train_test_predict_val]
						<< prefixs[9] + std::to_string(mi)
						<< prefixs[4]
						<< prefixs[7]
						<< ".ply";
				}
			}
			std::cout << "	Saving segment mesh: " << prefixs[9] + std::to_string(mi) << std::endl;
		}
		else
		{
			if (processing_mode == 0)
			{
				mesh_str_ostemp
					<< root_path
					<< folder_names_level_0[6]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< prefixs[4]
					<< prefixs[7]
					<< ".ply";
			}
			else if (processing_mode == 1)
			{
				mesh_str_ostemp
					<< root_path
					<< folder_names_level_0[8]
					<< sota_folder_path
					<< folder_names_level_0[6]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< prefixs[4]
					<< prefixs[7]
					<< ".ply";
			}
			else if (processing_mode == 2)
			{
				if (write_pnp)
				{
					mesh_str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_pssnet[1]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[mi]
						<< prefixs[4]
						<< prefixs[7]
						<< ".ply";
				}
				else
				{
					mesh_str_ostemp
						<< root_path
						<< folder_names_level_0[11]
						<< folder_names_level_0[6]
						<< folder_names_level_1[train_test_predict_val]
						<< base_names[mi]
						<< prefixs[4]
						<< prefixs[7]
						<< ".ply";
				}
			}

			std::cout << "	Saving segment mesh: " << base_names[mi] << std::endl;
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		smesh_out->remove_common_non_used_properties();
		smesh_out->remove_non_used_properties_for_segment_mesh();
		rply_output(smesh_out, meshPath_temp);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void write_semantic_mesh_data
	(
		SFMesh* smesh_out,
		const int mi
	)
	{
		const double t_total = omp_get_wtime();
		std::ostringstream mesh_str_ostemp;
		std::string basic_write_path, pref_tmp;
		if (processing_mode == 0) //RF
		{
			std::cout << "	Writing RF test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 1 && sota_folder_path != "PSSNet/") //SOTA
		{
			std::cout << "	Writing SOTA test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 2 || sota_folder_path == "PSSNet/")
		{
			if (current_mode != operating_mode::Process_semantic_pcl)
			{
				std::cout << "	Writing PSSNet step-1 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
			else
			{
				std::cout << "	Writing PSSNet step-2 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_pssnet[8] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
		}

		if (save_textures_in_predict)
		{
			basic_write_path += prefixs[9] + std::to_string(mi);
			if (0 != access(basic_write_path.c_str(), 0))
			{
				mkdir(basic_write_path.c_str());
			}
			basic_write_path += "/";
		}

		if (use_batch_processing)
		{
			mesh_str_ostemp
				<< basic_write_path
				<< prefixs[9] + std::to_string(mi)
				<< pref_tmp
				<< ".ply";
			std::cout << "	Saving semantic mesh: " << prefixs[9] + std::to_string(mi) << std::endl;
		}
		else
		{
			mesh_str_ostemp
				<< basic_write_path
				<< base_names[mi]
				<< pref_tmp
				<< ".ply";

			std::cout << "	Saving semantic mesh: " << base_names[mi] << std::endl;
		}

		std::vector<std::string> comment;
		comment = std::vector<std::string>(labels_name.size() + smesh_out->texture_names.size() + 1, std::string());
		for (int ti = 0; ti < smesh_out->texture_names.size(); ++ti)
		{
			comment[ti] = ply_comment_element[0] + " " + smesh_out->texture_names[ti];
		}
		comment[smesh_out->texture_names.size()] = ply_comment_element[1] + " -1 " + ply_comment_element[3];
		comment[smesh_out->texture_names.size()] = ply_comment_element[1] + " 0 " + ply_comment_element[2];

		for (int cmi = smesh_out->texture_names.size() + 1; cmi < comment.size(); ++cmi)
		{
			comment[cmi] = ply_comment_element[1] + " " +
				std::to_string(cmi - smesh_out->texture_names.size()) + " " +
				labels_name[cmi - smesh_out->texture_names.size() - 1];
		}

		if (train_test_predict_val == 2)
		{
			if (!smesh_out->get_face_property<int>("f:" + label_definition))
				smesh_out->add_face_property<int>("f:" + label_definition, -1);
			smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:" + label_definition);

			for (int fi = 0; fi < smesh_out->faces_size(); ++fi)
			{
				SFMesh::Face fdx(fi);
				if (!ignored_labels_name.empty() && smesh_out->get_face_truth_label[fdx] != -1)
				{
					bool is_ignore = false;
					for (int ig_i = 0; ig_i < ignored_labels_name.size(); ++ig_i)
					{
						if (ignored_labels_name[ig_i] == labels_name[smesh_out->get_face_truth_label[fdx] - 1])
						{
							is_ignore = true;
							break;
						}
					}

					if (!is_ignore)
						smesh_out->get_face_truth_label[fdx] = smesh_out->get_face_predict_label[fdx];
				}
				else if (!ignored_labels_name.empty() && smesh_out->get_face_truth_label[fdx] < 0)
				{
					smesh_out->get_face_truth_label[fdx] = 0;
				}
				else
				{
					smesh_out->get_face_truth_label[fdx] = smesh_out->get_face_predict_label[fdx];
				}
			}
			smesh_out->remove_face_property(smesh_out->get_face_predict_label);
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		smesh_out->remove_common_non_used_properties();
		smesh_out->remove_non_used_properties_for_semantic_mesh();
		rply_output(smesh_out, meshPath_temp, comment);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}


	void write_error_mesh_data
	(
		SFMesh* smesh_out,
		const int mi
	)
	{
		const double t_total = omp_get_wtime();
		std::ostringstream mesh_str_ostemp;
		std::string basic_write_path, pref_tmp;
		if (processing_mode == 0) //RF
		{
			std::cout << "	Writing RF test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 1 && sota_folder_path != "PSSNet/") //SOTA
		{
			std::cout << "	Writing SOTA test result : " << base_names[mi] << std::endl;
			basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
			pref_tmp = prefixs[4] + prefixs[5];
		}
		else if (processing_mode == 2 || sota_folder_path == "PSSNet/") //RF
		{
			if (previous_mode != operating_mode::Process_semantic_pcl)
			{
				std::cout << "	Writing PSSNet step-1 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
			else
			{
				std::cout << "	Writing PSSNet step-2 test result : " << base_names[mi] << std::endl;
				basic_write_path = root_path + folder_names_level_0[11] + folder_names_pssnet[8] + folder_names_level_1[train_test_predict_val];
				pref_tmp = prefixs[4] + prefixs[5];
			}
		}

		pref_tmp += prefixs[1];

		if (use_batch_processing)
		{
			mesh_str_ostemp
				<< basic_write_path
				<< prefixs[9] + std::to_string(mi)
				<< pref_tmp
				<< ".ply";
			std::cout << "Saving error-map: " << prefixs[9] + std::to_string(mi) << std::endl;
		}
		else
		{
			mesh_str_ostemp
				<< basic_write_path
				<< base_names[mi]
				<< pref_tmp
				<< ".ply";
			std::cout << "Saving error-map: " << base_names[mi] << std::endl;
		}

		std::vector<std::string> comment;
		comment = std::vector<std::string>(smesh_out->texture_names.size(), std::string());
		for (int ti = 0; ti < smesh_out->texture_names.size(); ++ti)
		{
			comment[ti] = ply_comment_element[0] + " " + smesh_out->texture_names[ti];
		}

		if (train_test_predict_val == 2)
		{
			if (!smesh_out->get_face_property<int>("f:" + label_definition))
				smesh_out->add_face_property<int>("f:" + label_definition, -1);
			smesh_out->get_face_truth_label = smesh_out->get_face_property<int>("f:" + label_definition);

#pragma omp parallel for schedule(dynamic)
			for (int fi = 0; fi < smesh_out->faces_size(); ++fi)
			{
				SFMesh::Face fdx(fi);
				smesh_out->get_face_truth_label[fdx] = smesh_out->get_face_predict_label[fdx];
			}
			smesh_out->remove_face_property(smesh_out->get_face_predict_label);
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		smesh_out->remove_common_non_used_properties();
		smesh_out->remove_non_used_properties_for_error_mesh();
		rply_output(smesh_out, meshPath_temp, comment);
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}


	void write_pnp_mesh_data //For L1 to L0 
	(
		SFMesh* smesh_out,
		const int mi
	)
	{
		std::ostringstream mesh_str_ostemp;
		if (current_mode == operating_mode::Get_labels_for_planar_non_planar_from_semantic)
		{
			if (use_batch_processing)
			{
				mesh_str_ostemp
					<< root_path
					<< folder_names_level_0[11]
					<< folder_names_level_0[2]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi] + "/"
					<< base_names[mi]
					<< prefixs[15]
					<< ".ply";
			}
			else
			{
				mesh_str_ostemp
					<< root_path
					<< folder_names_level_0[11]
					<< folder_names_level_0[2]
					<< folder_names_level_1[train_test_predict_val]
					<< base_names[mi]
					<< prefixs[15]
					<< ".ply";
			}
		}

		std::cout << "Saving testing mesh: " << base_names[mi] << std::endl;

		std::vector<std::string> comment(labels_name_pnp.size() + smesh_out->textures.size() + 1, std::string());
		for (int ti = 0; ti < smesh_out->textures.size(); ++ti)
		{
			comment[ti] = ply_comment_element[0] + " " + smesh_out->textures[ti];
		}
		comment[smesh_out->textures.size()] = ply_comment_element[1] + " -1 " + ply_comment_element[2];

		for (int cmi = smesh_out->textures.size() + 1; cmi < comment.size(); ++cmi)
		{
			comment[cmi] = ply_comment_element[1] + " " +
				std::to_string(cmi - smesh_out->textures.size()) + " " +
				labels_name_pnp[cmi - smesh_out->textures.size() - 1];
		}

		std::string mesh_str_temp = mesh_str_ostemp.str().data();
		char * meshPath_temp = (char *)mesh_str_temp.data();
		smesh_out->remove_common_non_used_properties();
		smesh_out->remove_non_used_properties_for_pnp_mesh();
		std::cout << "start to save " << std::endl;
		rply_output(smesh_out, meshPath_temp, comment);
	}

	//save batch names in *.txt
	void save_txt_batches(std::vector<std::vector<std::pair<int, std::string>>> &all_batches_out)
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Saving splited batches in *.txt file." << std::endl;
		std::ostringstream batch_out;
		batch_out
			<< root_path
			<< prefixs[9]
			<< "names_"
			<< data_types[train_test_predict_val]
			<< ".txt";

		std::string str_temp = batch_out.str().data();
		std::ofstream fout;
		fout.open(str_temp.c_str());
		int li = 0;
		for (auto batch_i : all_batches_out)
		{
			for (auto tile_i : batch_i)
			{
				fout << get<1>(tile_i) << "\t";
			}

			if (li < all_batches_out.size() - 1)
				fout << "\n";
			++li;
		}

		fout.close();
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//save prior in txt
	void save_txt_statistics
	(
		std::vector<float> &label_statistics,
		std::vector<float> &label_seg_statistics
	)
	{
		const double t_total = omp_get_wtime();
		std::cout << "	Saving class statistics." << std::endl;
		std::ostringstream stat_out;
		stat_out
			<< root_path
			<< folder_names_level_0[0]
			<< data_types[train_test_predict_val]
			<< prefixs[12]
			<< ".txt";

		float sum_area = std::accumulate(label_statistics.begin(), label_statistics.end(), 0);
		int sum_segs = std::accumulate(label_seg_statistics.begin(), label_seg_statistics.end(), 0);

		std::string str_temp = stat_out.str().data();
		std::ofstream fout;
		fout.open(str_temp.c_str());
		fout << "class" << "\t";
		std::cout << "class" << "\t";
		fout << "unclassified" << "\t";
		std::cout << "unclassified" << "\t";
		for (int li = 0; li < labels_name.size(); ++li)
		{
			fout << labels_name[li];
			std::cout << labels_name[li];
			if (li != labels_name.size() - 1)
			{
				fout << "\t";
				std::cout << "\t";
			}
			else
			{
				fout << "\n";
				std::cout << "\n";
			}
		}

		fout << "area" << "\t";
		std::cout << "area" << "\t";
		for (int li = 0; li < label_statistics.size(); ++li)
		{
			fout << std::fixed << std::showpoint << std::setprecision(2) << label_statistics[li];
			std::cout << std::fixed << std::showpoint << std::setprecision(2) << label_statistics[li];
			if (li != labels_name.size())
			{
				fout << "\t";
				std::cout << "\t";
			}
			else
			{
				fout << "\n";
				std::cout << "\n";
			}

		}

		fout << "area_percentage" << "\t";
		std::cout << "area_percentage" << "\t";
		for (int li = 0; li < label_statistics.size(); ++li)
		{
			fout << std::fixed << std::showpoint << std::setprecision(4) << float(label_statistics[li] / sum_area);
			std::cout << std::fixed << std::showpoint << std::setprecision(4) << float(label_statistics[li] / sum_area);
			if (li != labels_name.size())
			{
				fout << "\t";
				std::cout << "\t";
			}
			else
			{
				fout << "\n";
				std::cout << "\n";
			}

		}

		fout << "segment_num" << "\t";
		std::cout << "segment_num" << "\t";
		for (int li = 0; li < label_statistics.size(); ++li)
		{
			fout << label_seg_statistics[li];
			std::cout << label_seg_statistics[li];
			if (li != labels_name.size())
			{
				fout << "\t";
				std::cout << "\t";
			}
			else
			{
				fout << "\n";
				std::cout << "\n";
			}
		}

		fout << "num_percentage" << "\t";
		std::cout << "num_percentage" << "\t";
		for (int li = 0; li < label_statistics.size(); ++li)
		{
			fout << std::fixed << std::showpoint << std::setprecision(4) << float(label_seg_statistics[li]) / float(sum_segs);
			std::cout << std::fixed << std::showpoint << std::setprecision(4) << float(label_seg_statistics[li]) / float(sum_segs);
			if (li != labels_name.size())
			{
				fout << "\t";
				std::cout << "\t";
			}
			else
			{
				fout << "\n";
				std::cout << "\n";
			}
		}

		fout.close();
		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void save_txt_feature_importance
	(
		std::vector<std::string> &feature_bank,
		std::vector<std::pair<int, int>> &feaid_count,
		const int total_nodes
	)
	{
		std::cout << "Saving feature bank importance " << std::endl;
		std::ostringstream feature_importance_out;

		feature_importance_out
			<< root_path
			<< folder_names_level_0[0]
			<< prefixs[13]
			<< ".txt";

		std::string str_temp = feature_importance_out.str().data();
		std::ofstream fout;
		fout.open(str_temp.c_str());
		for (std::size_t i = 0; i < feaid_count.size(); ++i)
		{
			fout << feature_bank[feaid_count[i].first] << "\t";
			fout << feaid_count[i].second << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << float(feaid_count[i].second) / float(total_nodes) << "\n";
		}
		fout.close();
	}

	void save_txt_mesh_areas
	(
		std::vector<std::pair<std::string, float>> &mesh_area,
		std::vector<std::pair<std::string, std::vector<float>>> &mesh_class_area
	)
	{
		std::cout << "Saving mesh area in txt file " << std::endl;

		std::ostringstream area_out;
		area_out
			<< root_path
			<< folder_names_level_0[0]
			<< "mesh_area_statistics.txt";

		std::string str_temp = area_out.str().data();

		std::ofstream fout;
		fout.open(str_temp.c_str());
		fout << "File" << "\t" << "Sum_area" << "\t"; //<< "recall" << "\t" << "F1_scores" << "\t" << "IoU" << "\n";
		for (std::size_t li = 0; li < labels_name.size(); ++li)
		{
			if (li != labels_name.size() - 1)
				fout << labels_name[li] << "\t";
			else
				fout << labels_name[li] << "\n";
		}

		for (std::size_t i = 0; i < mesh_area.size(); ++i)
		{
			fout << mesh_area[i].first << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << mesh_area[i].second << "\t";
			for (std::size_t li = 0; li < labels_name.size(); ++li)
			{
				if (li != labels_name.size() - 1)
					fout << std::fixed << std::showpoint << std::setprecision(6) << mesh_class_area[i].second[li] << "\t";
				else
					fout << std::fixed << std::showpoint << std::setprecision(6) << mesh_class_area[i].second[li] << "\n";
			}
		}
		fout.close();
	}

	void save_txt_feature_divergence
	(
		std::vector<std::string> &all_names,
		std::vector<float> &feas_var
	)
	{
		std::cout << "Saving mesh feature divergence in txt file " << std::endl;

		std::ostringstream fead_out;
		fead_out
			<< root_path
			<< folder_names_level_0[0]
			<< "feature_divergence_statistics"
			<< ".txt";
		std::string str_temp = fead_out.str().data();

		std::ofstream fout;
		fout.open(str_temp.c_str());
		fout << "File" << "\t" << "Fea_var_avg" << "\n";
		for (std::size_t i = 0; i < feas_var.size(); ++i)
		{
			fout << all_names[i] << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << feas_var[i] << "\n";
		}
		fout.close();
	}

	void save_txt_evaluation
	(
		CGAL::Classification::Label_set &labels,
		CGAL::Classification::Evaluation &evaluation,
		std::ostringstream &evaluation_out,
		const int m
	)
	{
		if (m > 0)
		{
			if (use_batch_processing)
			{
				std::cout << "Saving evaluation: " << prefixs[10] + std::to_string(m) << std::endl;
			}
			else
			{
				std::cout << "Saving evaluation: " << base_names[m] << std::endl;
			}
		}
		else
		{
			std::cout << "Saving evaluation " << std::endl;
		}

		std::string str_temp = evaluation_out.str().data();

		std::ofstream fout;
		fout.open(str_temp.c_str());

		fout << "Class" << "\t" << "Precision" << "\t" << "recall" << "\t" << "F1_scores" << "\t" << "IoU" << "\n";
		for (std::size_t i = 0; i < labels.size(); ++i)
		{
			fout << labels[i]->name() << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.precision(labels[i]) << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.recall(labels[i]) << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.f1_score(labels[i]) << "\t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.intersection_over_union(labels[i]) << "\n";
		}

		fout << "Mean_Accuracy" << "\t";
		fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.mean_accuracy() << "\n";
		fout << "Overall_Accuracy" << "\t";
		fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.accuracy() << "\n";
		fout << "Mean_F1_score" << "\t";
		fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.mean_f1_score() << "\n";
		fout << "Mean_IoU" << "\t";
		fout << std::fixed << std::showpoint << std::setprecision(6) << evaluation.mean_intersection_over_union() << "\n";
		fout.close();
	}


	//For nodes attributes that are constructed
	void save_graph_nodes_ply
	(
		std::vector<superfacets> &spf_current,
		PTCloud *cloud_sparse,
		const int mi
	)
	{
		std::cout << " Saving graph: nodes = " << spf_current.size() << std::endl;
		std::ostringstream temp;
		temp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[3]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[16] << "_nodes"
			<< ".ply";

		std::string str_temp = temp.str().data();
		char * Path_temp = (char *)str_temp.data();
		PTCloud* spf_pcl = new PTCloud;

		spf_pcl->add_vertex_property<int>("v:segment_id");
		auto get_segment_id = spf_pcl->get_vertex_property<int>("v:segment_id");
		spf_pcl->add_vertex_property<std::vector<float>>("v:segment_plane_parameters");
		auto get_segment_plane_params = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_plane_parameters");
		spf_pcl->add_vertex_property<std::vector<int>>("v:segment_neighbor_ids");
		auto get_segment_neighbor_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_neighbor_ids");
		spf_pcl->add_vertex_property<int>("v:segment_non_planar_label");
		auto get_segment_non_planar = spf_pcl->get_vertex_property<int>("v:segment_non_planar_label");
		spf_pcl->add_vertex_property<int>("v:segment_local_ground_id");
		auto get_segment_local_ground_id = spf_pcl->get_vertex_property<int>("v:segment_local_ground_id");
		spf_pcl->add_vertex_property<std::vector<int>>("v:segment_exterior_ids");//spf-id of exterior mat toching
		auto get_segment_exterior_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_exterior_ids");
		//boundary point 
		spf_pcl->add_vertex_property<std::vector<int>>("v:segment_sampled_pts_ids");
		auto get_segment_sampled_pts_ids = spf_pcl->get_vertex_property<std::vector<int>>("v:segment_sampled_pts_ids");
		spf_pcl->add_vertex_property<std::vector<float>>("v:segment_sampled_pts_x");
		auto get_segment_sampled_pts_x = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_x");
		spf_pcl->add_vertex_property<std::vector<float>>("v:segment_sampled_pts_y");
		auto get_segment_sampled_pts_y = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_y");
		spf_pcl->add_vertex_property<std::vector<float>>("v:segment_sampled_pts_z");
		auto get_segment_sampled_pts_z = spf_pcl->get_vertex_property<std::vector<float>>("v:segment_sampled_pts_z");

		for (auto &spf_i : spf_current)
		{
			spf_pcl->add_vertex(spf_i.avg_center);
			get_segment_id[*(--spf_pcl->vertices_end())] = spf_i.id;
			get_segment_plane_params[*(--spf_pcl->vertices_end())] = { spf_i.plane_parameter.x, spf_i.plane_parameter.y, spf_i.plane_parameter.z, spf_i.plane_parameter.w };
			get_segment_neighbor_ids[*(--spf_pcl->vertices_end())] = spf_i.neighbor_ids;
			get_segment_non_planar[*(--spf_pcl->vertices_end())] = spf_i.label;
			get_segment_local_ground_id[*(--spf_pcl->vertices_end())] = spf_i.local_ground_id;
			for (int mat_i = 0; mat_i < spf_i.exterior_mat_ids.size(); ++mat_i)
			{
				if (spf_i.exterior_mat_ids[mat_i] != -1)
				{
					get_segment_exterior_ids[*(--spf_pcl->vertices_end())].push_back(spf_i.exterior_mat_ids[mat_i]);
				}
			}
			for (int spts_i = 0; spts_i < spf_i.sampled_points_ids.size(); ++spts_i)
			{
				int pi = spf_i.sampled_points_ids[spts_i];
				if (pi != -1)
				{
					PTCloud::Vertex ptx(pi);
					get_segment_sampled_pts_ids[*(--spf_pcl->vertices_end())].push_back(pi);
					get_segment_sampled_pts_x[*(--spf_pcl->vertices_end())].push_back(cloud_sparse->get_points_coord[ptx].x);
					get_segment_sampled_pts_y[*(--spf_pcl->vertices_end())].push_back(cloud_sparse->get_points_coord[ptx].y);
					get_segment_sampled_pts_z[*(--spf_pcl->vertices_end())].push_back(cloud_sparse->get_points_coord[ptx].z);
					spf_i.sampled_pts_coords.push_back(cloud_sparse->get_points_coord[ptx]);
				}
			}
		}
		spf_pcl->remove_vertex_property(spf_pcl->get_points_ground_truth);
		spf_pcl->remove_vertex_property(spf_pcl->get_points_face_belong_id);
		rply_output(spf_pcl, Path_temp);
		delete spf_pcl;
	}

	//For edges that are constructed
	void save_graph_edges_ply
	(
		PTCloud* pcl_temp,
		std::vector<std::pair<int, int>> &spf_edges,
		const int mi,
		std::vector<superfacets> spf_current
	)
	{
		if (spf_current.empty())
			std::cout << " Saving graph: nodes = " << pcl_temp->vertices_size() << "; edges = " << spf_edges.size() << std::endl;
		else
			std::cout << " Saving graph: nodes = " << spf_current.size() << "; edges = " << spf_edges.size() << std::endl;//<<"; expanded edges = ";
		std::ostringstream temp;
		temp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[0] << folder_names_pssnet[2]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[16]
			<< ".ply";

		std::string str_temp = temp.str().data();
		std::ofstream fout;
		fout.open(str_temp.c_str());
		//triangle stored as segment
		fout << "ply" << "\n";
		fout << "format ascii 1.0" << "\n";
		if (spf_current.empty())
			fout << "element vertex " << pcl_temp->vertices_size() << "\n";
		else
			fout << "element vertex " << spf_current.size() << "\n";
		fout << "property float x" << "\n";
		fout << "property float y" << "\n";
		fout << "property float z" << "\n";
		fout << "element edge " << spf_edges.size() << "\n";
		fout << "property int vertex1" << "\n";
		fout << "property int vertex2" << "\n";
		fout << "end_header" << "\n";

		if (spf_current.empty())
		{
			for (auto ptx : pcl_temp->vertices())
				fout << std::fixed << std::showpoint << std::setprecision(6) << pcl_temp->get_vertex_property<vec3>("v:point")[ptx].x << "\t" << pcl_temp->get_vertex_property<vec3>("v:point")[ptx].y << "\t" << pcl_temp->get_vertex_property<vec3>("v:point")[ptx].z << "\n";
		}
		else
		{
			sort(spf_current.begin(), spf_current.end(), smallest_segment_id);
			int total_num_edges_expand = 0;
			for (int seg_i = 0; seg_i < spf_current.size(); ++seg_i)
			{
				vec3 vtemp = spf_current[seg_i].avg_center;
				fout << std::fixed << std::showpoint << std::setprecision(6) << vtemp.x << "\t" << vtemp.y << "\t" << vtemp.z << "\n";
			}
		}

		for (auto ei : spf_edges)
		{
			fout << ei.first << "\t" << ei.second << "\n";
		}
		fout.close();
	}

	//write GCN point cloud data: sampling point cloud with 1D features
	void write_feature_pointcloud_data_for_GCN
	(
		PTCloud* pcl_out,
		const int mi
	)
	{
		std::cout << "Start to writing feature point cloud for GCN ..." << std::endl << std::endl;

		pcl_out->remove_vertex_property(pcl_out->get_points_face_belong_ids);
		pcl_out->remove_vertex_property(pcl_out->get_points_face_ele_belong_ids);
		pcl_out->remove_vertex_property(pcl_out->get_points_use_seg_truth);
		pcl_out->remove_vertex_property(pcl_out->get_points_rgb_x);
		pcl_out->remove_vertex_property(pcl_out->get_points_rgb_y);
		pcl_out->remove_vertex_property(pcl_out->get_points_rgb_z);
		pcl_out->remove_vertex_property(pcl_out->get_points_hsv_x);
		pcl_out->remove_vertex_property(pcl_out->get_points_hsv_y);
		pcl_out->remove_vertex_property(pcl_out->get_points_hsv_z);
		std::ostringstream str_ostemp;
		str_ostemp
			<< root_path
			<< folder_names_level_0[11]
			<< folder_names_pssnet[0] << folder_names_pssnet[4]
			<< folder_names_level_1[train_test_predict_val]
			<< base_names[mi]
			<< prefixs[0]
			<< prefixs[17]
			<< ".ply";

		std::string str_temp = str_ostemp.str().data();
		char * Path_temp = (char *)str_temp.data();

		rply_output(pcl_out, Path_temp);
	}

	void get_mesh_labels
	(
		SFMesh *tmp_mesh,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth_train,
		std::vector< std::vector<int>> &seg_face_vec
	)
	{
		for (int si = 0; si < seg_face_vec.size(); ++si)
		{
			std::vector<int> majority_labels(labels_name.size(), 0);
			for (int fi = 0; fi < seg_face_vec[si].size(); ++fi)
			{
				SFMesh::Face fdx(seg_face_vec[si][fi]);
				int fdx_label_truth = tmp_mesh->get_face_truth_label[fdx];

				if (fdx_label_truth != 0 && fdx_label_truth != -1)
					++majority_labels[fdx_label_truth - 1];

				tmp_mesh->get_face_segment_id[fdx] = seg_ids[si];
			}
			int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			seg_truth_train.push_back(maxElementIndex);
		}
	}

	void get_mesh_labels
	(
		SFMesh *tmp_mesh,
		std::vector<int> &seg_truth_train,
		std::vector< std::vector<int>> &seg_face_vec
	)
	{
		superfacet_id_index_map.clear();
		for (int si = 0; si < seg_face_vec.size(); ++si)
		{
			std::vector<int> majority_labels(labels_name.size(), 0);
			for (int fi = 0; fi < seg_face_vec[si].size(); ++fi)
			{
				SFMesh::Face fdx(seg_face_vec[si][fi]);
				if (train_test_predict_val != 2)
				{
					int fdx_label_truth = tmp_mesh->get_face_truth_label[fdx];
					//labels should be within [1 to INF]
					if (fdx_label_truth != 0 && fdx_label_truth != -1)
						++majority_labels[fdx_label_truth - 1];
				}

				tmp_mesh->get_face_segment_id[fdx] = si;
				if (fi == 0)
					superfacet_id_index_map[tmp_mesh->get_face_segment_id[fdx]] = si;
			}

			if (train_test_predict_val != 2)
			{
				int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
				seg_truth_train.push_back(maxElementIndex);
			}
		}
	}

	void save_txt_evaluation
	(
		std::vector<float> &op_out,
		std::vector<float> &gc_tc_out,
		std::vector<float> &br_out,
		std::ostringstream &evaluation_out,
		const int m
	)
	{
		if (m != -1)
			std::cout << "Saving evaluation: " << base_names[m] << std::endl;
		else
			std::cout << "Saving all evaluation !!!" << std::endl;
		std::string str_temp = evaluation_out.str().data();

		std::ofstream fout;
		fout.open(str_temp.c_str());
		//OP
		fout << "OP\n";
		for (std::size_t i = 0; i < op_out.size() - 1; ++i)
		{
			fout << labels_name_pnp[i] << ": \t";
			fout << std::fixed << std::showpoint << std::setprecision(6) << op_out[i] << "\n";
		}
		fout << "mOP: \t";
		fout << std::fixed << std::showpoint << std::setprecision(6) << op_out[labels_name_pnp.size()] << "\n\n";

		//num ratio
		fout << "Number Ratio\n";
		fout << "Num test : \t" << int(gc_tc_out[0]) << "\n";
		fout << "Num train : \t" << int(gc_tc_out[1]) << "\n";
		fout << "Num_ratio (g_c / t_c)  : \t" << std::fixed << std::showpoint << std::setprecision(6) << gc_tc_out[2] << "\n\n";

		//Border
		fout << "Border\n";
		fout << "Num edges truth : \t" << int(br_out[0]) << "\n";
		fout << "Num edges pred : \t" << int(br_out[1]) << "\n";
		fout << "Num edges Intersec : \t" << int(br_out[2]) << "\n";
		fout << "BR : \t" << std::fixed << std::showpoint << std::setprecision(6) << br_out[3] << "\n";
		fout << "BP : \t" << std::fixed << std::showpoint << std::setprecision(6) << br_out[4] << "\n";

		fout.close();
	}
}
