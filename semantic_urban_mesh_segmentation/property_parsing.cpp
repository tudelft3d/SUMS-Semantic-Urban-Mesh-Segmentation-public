/*
*   Name        : property_parsing.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh property parsing and point cloud sampling
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

#include "property_parsing.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	void sampling_point_cloud_on_mesh
	(
		SFMesh* smesh_in,
		std::vector<cv::Mat> &texture_maps_in,
		PTCloud *cloud_3d_sampled,
		PTCloud *cloud_ele,
		PTCloud *face_center_cloud,
		std::map<int, int> &ptidx_faceid_map,
		const int mi
	)
	{
		std::cout << "	- Pre-processing of the input mesh: ";
		switch (sampling_strategy)
		{
		case 0:
			std::cout << "Sample the point clouds from meshes, ";
			break;
		case 1:
			std::cout << "Using face centers as sampled point clouds, ";
			break;
		case 2:
			std::cout << "Using face centers and vertices as sampled point clouds, ";
			break;
		case 3:
			std::cout << "Sample the point clouds from meshes + Using face centers as sampled point clouds, ";
			break;
		case 4:
			std::cout << "Sample the point clouds from meshes + Using face centers and vertices as sampled point clouds, ";
			break;
		}
		const double t_total = omp_get_wtime();

		//add temporary vertex property
		smesh_in->add_vertex_property<bool>("v:vertex_visited", false);
		auto get_vert_visited = smesh_in->get_vertex_property<bool>("v:vertex_visited");

		//reset global properties
		mesh_bounding_box = easy3d::Box3();
		mesh_all_area = 0.0f;

		//temporary point cloud for assisting sampling process
		PTCloud* tex_cloud_temp = new PTCloud;
		tex_cloud_temp->get_points_color = tex_cloud_temp->add_vertex_property<vec3>("v:color");
		face_center_cloud->add_vertex_property<vec3>("v:normal");
		face_center_cloud->get_points_normals = face_center_cloud->get_vertex_property<vec3>("v:normal");
		PTCloud *face_center_vertices_cloud = new PTCloud;
		easy3d::PointCloud *initial_sampled_sampled_point_cloud = new easy3d::PointCloud;
		//Pre-processing color information with different color spaces
		std::vector<cv::Mat> hsv_maps;
		for (int i = 0; i < texture_maps_in.size(); i++)
		{
			cv::Mat hsv_temp = cv::Mat::zeros(texture_maps_in[i].size(), texture_maps_in[i].type());
			cv::cvtColor(texture_maps_in[i], hsv_temp, cv::COLOR_BGR2HSV);
			hsv_maps.emplace_back(hsv_temp);
		}
		//parsing face properties: face center, texture color
		//parsing global properties: mesh_bounding_box, mesh_all_area 
		for (auto fi : smesh_in->faces())
		{
			for (auto vi : smesh_in->vertices(fi))
			{
				if (!get_vert_visited[vi])
				{
					get_vert_visited[vi] = true;
					mesh_bounding_box.add_point(smesh_in->get_points_coord[vi]);
					face_center_vertices_cloud->add_vertex(smesh_in->get_points_coord[vi]);
					if (sampling_strategy == 2 || sampling_strategy == 4)
						initial_sampled_sampled_point_cloud->add_vertex(smesh_in->get_points_coord[vi]);

					for (auto f_neg : smesh_in->faces(vi))
					{
						face_center_vertices_cloud->get_points_face_belong_ids[*(--face_center_vertices_cloud->vertices_end())].push_back(f_neg.idx());
					}
				}
				smesh_in->get_face_center[fi] += smesh_in->get_points_coord[vi];
			}

			smesh_in->get_face_center[fi] /= 3.0f;
			smesh_in->get_face_area[fi] = FaceArea(smesh_in, fi);
			smesh_in->get_face_normals[fi] = smesh_in->compute_face_normal(fi);
			mesh_all_area += smesh_in->get_face_area[fi];

			//get color for sampled point cloud
			if (with_texture && add_point_color)
				texture_pointcloud_generation(smesh_in, fi, texture_maps_in, hsv_maps, tex_cloud_temp);

			//generate face center point cloud
			PTCloud::Vertex ptx(fi.idx());
			face_center_cloud->add_vertex(smesh_in->get_face_center[fi]);
			face_center_cloud->get_points_normals[ptx] = smesh_in->get_face_normals[fi];
			face_center_cloud->get_points_face_belong_id[ptx] = fi.idx();
			ptidx_faceid_map[(*(--face_center_cloud->vertices_end())).idx()] = fi.idx();

			face_center_vertices_cloud->add_vertex(smesh_in->get_face_center[fi]);
			face_center_vertices_cloud->get_points_face_belong_ids[*(--face_center_vertices_cloud->vertices_end())].push_back(fi.idx());

			if (sampling_strategy == 1 || sampling_strategy == 2 || sampling_strategy == 3 || sampling_strategy == 4)
				initial_sampled_sampled_point_cloud->add_vertex(smesh_in->get_face_center[fi]);
		}

		//point cloud sampling initialization
		if (sampling_strategy == 0 || sampling_strategy == 3 || sampling_strategy == 4)
			sampling_pointcloud_on_mesh(initial_sampled_sampled_point_cloud, smesh_in, sampling_point_density);//Require mesh area

		easy3d::PointCloud *initial_ele_sampled_point_cloud = new easy3d::PointCloud;
		sampling_pointcloud_on_mesh(initial_ele_sampled_point_cloud, smesh_in, ele_sampling_point_density);//Require mesh area

		//Adding color and sampled points
		finalization_sampling_point_cloud(cloud_3d_sampled, cloud_ele, initial_sampled_sampled_point_cloud, initial_ele_sampled_point_cloud);

		//build kdtrees for face center and texture cloud
		easy3d::KdTree *tex_tree = new easy3d::KdTree;
		if (with_texture && add_point_color)
			Build_kdtree(tex_cloud_temp, tex_tree);
		easy3d::KdTree *center_cloud_tree = new easy3d::KdTree;
		Build_kdtree(face_center_cloud, center_cloud_tree);

		if (sampling_strategy == 2 || sampling_strategy == 4)
		{
			for (int pi = 0; pi < face_center_vertices_cloud->vertices_size(); ++pi)
			{
				PTCloud::Vertex ptx(pi);
				cloud_3d_sampled->get_points_face_belong_ids[ptx].insert(cloud_3d_sampled->get_points_face_belong_ids[ptx].end(),
					face_center_vertices_cloud->get_points_face_belong_ids[ptx].begin(),
					face_center_vertices_cloud->get_points_face_belong_ids[ptx].end());
			}
		}

		//get sampled point cloud normals and face belong from mesh
		get_sampling_cloud_normals_from_mesh_faces(smesh_in, cloud_3d_sampled, face_center_cloud, tex_cloud_temp, center_cloud_tree, tex_tree);

		//get sampled point cloud normals and face belong from mesh
		match_ele_sampling_pointcloud_with_mesh_faces(smesh_in, cloud_ele, face_center_cloud, center_cloud_tree);

		//delete temporary vertex property
		smesh_in->remove_vertex_property(smesh_in->get_vertex_property<bool>("v:vertex_visited"));

		//output texture point cloud
		if (save_tex_cloud)
		{
			write_tex_pointcloud_data(tex_cloud_temp, mi);
		}

		delete tex_cloud_temp;
		delete face_center_vertices_cloud;
		delete initial_sampled_sampled_point_cloud;
		delete initial_ele_sampled_point_cloud;
		delete tex_tree;
		delete center_cloud_tree;

		std::cout << "done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;

		//save sampled point cloud
		if (save_sampled_pointclouds)
		{
			write_pointcloud_data(cloud_3d_sampled, 0, mi);//use as face center point cloud if not use sampling
			write_pointcloud_data(cloud_ele, 1, mi);
		}
	}

	void sampling_point_cloud_on_mesh
	(
		SFMesh* smesh_in,
		std::vector<cv::Mat> &texture_maps_in,
		PTCloud *cloud_3d,
		const int mi
	)
	{
		std::cout << "	- Pre-processing of input mesh: ";
		switch (sampling_strategy)
		{
		case 0:
			std::cout << "Sample the point clouds from meshes, ";
			break;
		case 1:
			std::cout << "Using face centers as sampled point clouds, ";
			break;
		case 2:
			std::cout << "Using face centers and vertices as sampled point clouds, ";
			break;
		case 3:
			std::cout << "Sample the point clouds from meshes + Using face centers as sampled point clouds, ";
			break;
		case 4:
			std::cout << "Sample the point clouds from meshes + Using face centers and vertices as sampled point clouds, ";
			break;
		}
		const double t_total = omp_get_wtime();

		//add temporary vertex property
		smesh_in->add_vertex_property<bool>("v:vertex_visited", false);
		auto get_vert_visited = smesh_in->get_vertex_property<bool>("v:vertex_visited");

		//reset global properties
		mesh_bounding_box = easy3d::Box3();
		mesh_all_area = 0.0f;

		//temporary point cloud for assisting sampling process
		PTCloud* tex_cloud_temp = new PTCloud;
		tex_cloud_temp->get_points_color = tex_cloud_temp->add_vertex_property<vec3>("v:color");
		PTCloud *face_center_cloud = new PTCloud;
		face_center_cloud->add_vertex_property<vec3>("v:normal");
		face_center_cloud->get_points_normals = face_center_cloud->get_vertex_property<vec3>("v:normal");
		PTCloud *face_center_vertices_cloud = new PTCloud;
		easy3d::PointCloud *initial_sampled_point_cloud = new easy3d::PointCloud;

		//Pre-processing color information with different color spaces
		std::vector<cv::Mat> hsv_maps, lab_maps;
		for (int i = 0; i < texture_maps_in.size(); i++)
		{
			cv::Mat hsv_temp = cv::Mat::zeros(texture_maps_in[i].size(), texture_maps_in[i].type());
			cv::cvtColor(texture_maps_in[i], hsv_temp, cv::COLOR_BGR2HSV);
			hsv_maps.emplace_back(hsv_temp);
			cv::Mat lab_temp = cv::Mat::zeros(texture_maps_in[i].size(), texture_maps_in[i].type());
			cv::cvtColor(texture_maps_in[i], lab_temp, cv::COLOR_BGR2Lab);
			lab_maps.emplace_back(lab_temp);
		}
		//parsing face properties: face center, texture color
		//parsing global properties: mesh_bounding_box, mesh_all_area 
		for (auto fi : smesh_in->faces())
		{
			for (auto vi : smesh_in->vertices(fi))
			{
				if (!get_vert_visited[vi])
				{
					get_vert_visited[vi] = true;
					mesh_bounding_box.add_point(smesh_in->get_points_coord[vi]);
					face_center_vertices_cloud->add_vertex(smesh_in->get_points_coord[vi]);
					if (sampling_strategy == 2 || sampling_strategy == 4)
						initial_sampled_point_cloud->add_vertex(smesh_in->get_points_coord[vi]);

					for (auto f_neg : smesh_in->faces(vi))
					{
						face_center_vertices_cloud->get_points_face_belong_ids[*(--face_center_vertices_cloud->vertices_end())].push_back(f_neg.idx());
					}
				}
				smesh_in->get_face_center[fi] += smesh_in->get_points_coord[vi];
			}

			smesh_in->get_face_center[fi] /= 3.0f;
			smesh_in->get_face_area[fi] = FaceArea(smesh_in, fi);
			smesh_in->get_face_normals[fi] = smesh_in->compute_face_normal(fi);
			mesh_all_area += smesh_in->get_face_area[fi];

			//get color for sampled point cloud
			if (with_texture && (add_point_color_for_dp_input || save_tex_cloud))
				texture_pointcloud_generation(smesh_in, fi, texture_maps_in, hsv_maps, tex_cloud_temp);

			//generate face center point cloud
			PTCloud::Vertex ptx(fi.idx());
			face_center_cloud->add_vertex(smesh_in->get_face_center[fi]);
			face_center_cloud->get_points_normals[ptx] = smesh_in->get_face_normals[fi];
			face_center_cloud->get_points_face_belong_id[ptx] = fi.idx();

			face_center_vertices_cloud->add_vertex(smesh_in->get_face_center[fi]);
			face_center_vertices_cloud->get_points_face_belong_ids[*(--face_center_vertices_cloud->vertices_end())].push_back(fi.idx());

			if (sampling_strategy == 1 || sampling_strategy == 2 || sampling_strategy == 3 || sampling_strategy == 4)
				initial_sampled_point_cloud->add_vertex(smesh_in->get_face_center[fi]);
		}

		//point cloud sampling initialization, require mesh_bounding_box and mesh_all_area
		if (sampling_strategy == 0 || sampling_strategy == 3 || sampling_strategy == 4)
			sampling_pointcloud_on_mesh(initial_sampled_point_cloud, smesh_in, sampling_point_density);//Require mesh area

		//Adding color and sampled points
		finalization_sampling_point_cloud(cloud_3d, initial_sampled_point_cloud);

		//build kdtrees for face center and texture cloud
		easy3d::KdTree *tex_tree = new easy3d::KdTree;
		if (with_texture && add_point_color_for_dp_input)
			Build_kdtree(tex_cloud_temp, tex_tree);
		easy3d::KdTree *center_cloud_tree = new easy3d::KdTree;
		Build_kdtree(face_center_cloud, center_cloud_tree);

		if (sampling_strategy == 2 || sampling_strategy == 4)
		{
			for (int pi = 0; pi < face_center_vertices_cloud->vertices_size(); ++pi)
			{
				PTCloud::Vertex ptx(pi);
				cloud_3d->get_points_face_belong_ids[ptx].insert(cloud_3d->get_points_face_belong_ids[ptx].end(),
					face_center_vertices_cloud->get_points_face_belong_ids[ptx].begin(),
					face_center_vertices_cloud->get_points_face_belong_ids[ptx].end());
			}
		}

		//get sampled point cloud normals and face belong from mesh
		get_sampling_cloud_normals_from_mesh_faces(smesh_in, cloud_3d, face_center_cloud, tex_cloud_temp, center_cloud_tree, tex_tree);

		//delete temporary vertex property
		smesh_in->remove_vertex_property(smesh_in->get_vertex_property<bool>("v:vertex_visited"));

		//output texture point cloud
		if (save_tex_cloud)
		{
			write_tex_pointcloud_data(tex_cloud_temp, mi);
		}

		delete tex_cloud_temp;	delete face_center_cloud;
		delete face_center_vertices_cloud;
		delete initial_sampled_point_cloud;
		delete tex_tree; delete center_cloud_tree;

		std::cout << "done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//construct superfacet from raw mesh
	void construct_superfacets
	(
		SFMesh *smesh_in,
		PTCloud *cloud_sampled,
		std::vector<superfacets>& spf_final,
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_ids
	)
	{
		std::cout << "	- Construct superfacets from raw mesh: ";
		const double t_total = omp_get_wtime();

		std::map<int, int> segment_id_vecind;//segment_id and index in the vector
		for (auto f : smesh_in->faces())
		{
			//parsing faces properties for further computation
			for (auto vi : smesh_in->vertices(f))
			{
				if (is_pointclouds_exist)
					smesh_in->get_face_center[f] += smesh_in->get_points_coord[vi];
				smesh_in->get_points_normals[vi] = smesh_in->compute_vertex_normal(vi);
				vec3_validation_check(smesh_in->get_points_normals[vi]);
			}
			if (is_pointclouds_exist)
				smesh_in->get_face_center[f] /= 3.0f;
			smesh_in->get_face_area[f] = FaceArea(smesh_in, f);
			smesh_in->get_face_normals[f] = smesh_in->compute_face_normal(f);
			vec3_validation_check(smesh_in->get_face_normals[f]);

			//initialize vector size of mat based features
			smesh_in->get_face_interior_medialball_radius[f] = std::vector<float>(smesh_in->get_face_sampled_points[f].size(), 0.0f);

			//parsing sampled point cloud to a temporary vector
			std::vector<Point_3> ExactVec_CGAL;
			std::vector<int> sampled_points_ids;
			std::pair<float, float> local_elevation_minmax_ini(FLT_MAX, -FLT_MAX);
			float avg_ele = 0.0f;
			for (auto pi : smesh_in->get_face_sampled_points[f])
			{
				PTCloud::Vertex ptx(pi);
				vec3 p_temp = cloud_sampled->get_points_coord[ptx];
				avg_ele += p_temp.z;
				ExactVec_CGAL.emplace_back(p_temp.x, p_temp.y, p_temp.z);
				sampled_points_ids.push_back(pi);
				local_elevation_minmax_ini.first = local_elevation_minmax_ini.first < p_temp.z ? local_elevation_minmax_ini.first : p_temp.z;
				local_elevation_minmax_ini.second = local_elevation_minmax_ini.second > p_temp.z ? local_elevation_minmax_ini.second : p_temp.z;
			}

			//construct or parsing to segment
			int seg_i = -1;
			if (segment_id_vecind.find(smesh_in->get_face_segment_id[f]) == segment_id_vecind.end())
			{
				seg_i = spf_final.size();
				segment_id_vecind[smesh_in->get_face_segment_id[f]] = spf_final.size();
				superfacets spf_temp;
				spf_temp.id = smesh_in->get_face_segment_id[f];
				spf_temp.face_vec.emplace_back(f);
				spf_temp.face_id_vec.push_back(f.idx());
				spf_temp.ExactVec_CGAL.insert(spf_temp.ExactVec_CGAL.end(), ExactVec_CGAL.begin(), ExactVec_CGAL.end());
				spf_temp.avg_ele += avg_ele;
				spf_temp.sum_area += smesh_in->get_face_area[f];
				spf_temp.sampled_points = sampled_points_ids;

				spf_temp.local_elevation_minmax = local_elevation_minmax_ini;
				if (smesh_in->is_boundary(f))
					spf_temp.contain_mesh_border_facets = true;
				spf_final.emplace_back(spf_temp);
				std::vector<int> face_vec{ f.idx() };
			}
			else
			{
				seg_i = segment_id_vecind[smesh_in->get_face_segment_id[f]];
				spf_final[seg_i].face_vec.emplace_back(f);
				spf_final[seg_i].face_id_vec.push_back(f.idx());
				spf_final[seg_i].ExactVec_CGAL.insert(spf_final[seg_i].ExactVec_CGAL.end(), ExactVec_CGAL.begin(), ExactVec_CGAL.end());
				spf_final[seg_i].avg_ele += avg_ele;
				spf_final[seg_i].sum_area += smesh_in->get_face_area[f];

				spf_final[seg_i].local_elevation_minmax.first = spf_final[seg_i].local_elevation_minmax.first < local_elevation_minmax_ini.first ?
					spf_final[seg_i].local_elevation_minmax.first : local_elevation_minmax_ini.first;
				spf_final[seg_i].local_elevation_minmax.second = spf_final[seg_i].local_elevation_minmax.second > local_elevation_minmax_ini.second ?
					spf_final[seg_i].local_elevation_minmax.second : local_elevation_minmax_ini.second;

				spf_final[seg_i].sampled_points.insert(spf_final[seg_i].sampled_points.end(),
					sampled_points_ids.begin(), sampled_points_ids.end());

				if (smesh_in->is_boundary(f))
					spf_final[seg_i].contain_mesh_border_facets = true;
			}
		}

		//more faster for OpenMP dynamic if use descending order according to the size of each segment
		//since the larger segment thread need more time to run
		sort(spf_final.begin(), spf_final.end(), larger_segment_size);

		//build the global superfacet id-index map
		seg_face_vec = std::vector<std::vector<int>>(spf_final.size(), std::vector<int>());
		for (int spf_i = 0; spf_i < spf_final.size(); ++spf_i)
		{
			seg_ids.push_back(spf_final[spf_i].id);
			superfacet_id_index_map[spf_final[spf_i].id] = spf_i;
			seg_face_vec[spf_i].insert(seg_face_vec[spf_i].end(), spf_final[spf_i].face_id_vec.begin(), spf_final[spf_i].face_id_vec.end());
			spf_final[spf_i].avg_ele /= float(spf_final[spf_i].ExactVec_CGAL.size());
		}

		std::cout << "	Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//--- construct segment based point clouds, each segment represent by a point with it's features ---
	void construct_feature_pointclouds
	(
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		PTCloud* fea_cloud
	)
	{
		std::cout << "	- Construct feature point clouds, use ";
		const double t_total = omp_get_wtime();
		if (train_test_predict_val == 0)//ignore here for smote augmentation
		{
			//by default, unclassified labels in training data will be ignored
			//ignore selected labels
			if (!ignored_labels_name.empty())
				ignore_truth_labels(seg_truth);

			for (int sfi = 0; sfi < basic_feas.size(); ++sfi)
			{
				//if (seg_truth[sfi] != -1)
				fea_cloud->add_vertex(vec3(basic_feas[sfi][0], basic_feas[sfi][1], basic_feas[sfi][2]));
			}
		}
		else
		{
			for (int sfi = 0; sfi < basic_feas.size(); ++sfi)
				fea_cloud->add_vertex(vec3(basic_feas[sfi][0], basic_feas[sfi][1], basic_feas[sfi][2]));
		}

		fea_cloud->add_all_feature_properties
		(
			seg_face_vec,
			seg_ids,
			seg_truth,
			basic_feas,
			eigen_feas, color_feas,
			mulsc_ele_feas
		);
		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//--- configuration merge mesh and point cloud ---
	void input_mesh_configuration
	(
		SFMesh *smesh
	)
	{
		//mesh vertex properties
		smesh->get_points_coord = smesh->get_vertex_property<vec3>("v:point");

		//mesh face properties
		if (smesh->get_face_property<int>("f:" + label_definition))
			smesh->get_face_truth_label = smesh->get_face_property<int>("f:" + label_definition);
		else
		{
			smesh->add_face_property<int>("f:" + label_definition, -1);
			smesh->get_face_truth_label = smesh->get_face_property<int>("f:" + label_definition);
		}

		if (smesh->get_face_property<int>("f:face_predict"))
			smesh->get_face_predict_label = smesh->get_face_property<int>("f:face_predict");
		else
		{
			smesh->add_face_property<int>("f:face_predict", -1);
			smesh->get_face_predict_label = smesh->get_face_property<int>("f:face_predict");
		}

		if (smesh->get_face_property<int>("f:face_segment_id"))
			smesh->get_face_segment_id = smesh->get_face_property<int>("f:face_segment_id");
		else
		{
			smesh->add_face_property<int>("f:face_segment_id", -1);
			smesh->get_face_segment_id = smesh->get_face_property<int>("f:face_segment_id");
		}

		if (smesh->get_face_property<vec3>("f:color"))
			smesh->get_face_color = smesh->get_face_property<vec3>("f:color");
		else
		{
			smesh->add_face_property<vec3>("f:color");
			smesh->get_face_color = smesh->get_face_property<vec3>("f:color");
		}

		if (!smesh->get_face_property<vec3>("f:normal"))
			smesh->add_face_property<vec3>("f:normal");
		smesh->get_face_normals = smesh->get_face_property<vec3>("f:normal");
		if (!smesh->get_vertex_property<vec3>("v:normal"))
			smesh->add_vertex_property<vec3>("v:normal");
		smesh->get_points_normals = smesh->get_vertex_property<vec3>("v:normal");
	}

	//--- add mesh properties from input mesh ---
	void add_mesh_properties_from_input
	(
		SFMesh *smesh,
		const int mi,
		SFMesh *smesh_seg,
		std::vector<cv::Mat> &texture_maps,
		const int batch_index
	)
	{
		input_mesh_configuration(smesh);

		//get texture information
		if (with_texture)
		{
			if (!smesh->get_face_property<int>("f:texnumber"))
				smesh->add_face_property<int>("f:texnumber", 0);
			smesh->get_face_texnumber = smesh->get_face_property<int>("f:texnumber");

			if (!smesh->get_face_property<std::vector<float>>("f:texcoord"))
				smesh->add_face_property<std::vector<float>>("f:texcoord", std::vector<float>({ 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f }));
			smesh->get_face_texcoord = smesh->get_face_property<std::vector<float>>("f:texcoord");

			if (smesh->textures.empty())
			{
				cv::Mat dummy(1, 1, CV_8UC3, cv::Scalar(0, 255, 0));
				texture_maps.emplace_back(dummy);
				with_texture = false;
			}
			else
			{
				//create batch predict folder 
				std::string basic_write_path;
				if (save_textures_in_predict)
				{
					if (processing_mode == 0) //RF
						basic_write_path = root_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];
					else if (processing_mode == 1) //SOTA
						basic_write_path = root_path + folder_names_level_0[8] + sota_folder_path + folder_names_level_0[4] + folder_names_level_1[train_test_predict_val];

					basic_write_path += prefixs[9] + std::to_string(batch_index);
					if (0 != access(basic_write_path.c_str(), 0))
						mkdir(basic_write_path.c_str());
				}

				for (auto tex_i : smesh->textures)
				{
					if (!tex_i.empty() && tex_i[tex_i.size() - 1] == '\r')
						tex_i.erase(tex_i.size() - 1);
					smesh->texture_names.push_back(tex_i);

					std::ostringstream texture_str_ostemp;

					std::string path_tmp;
					std::vector<std::string> temp_file_folders;
					if (processing_mode == 2)
					{
						std::ostringstream temp_path;
						temp_path << root_path << folder_names_level_0[2] << folder_names_level_1[train_test_predict_val];
						std::string temp_data_path = temp_path.str().data();
						std::vector<std::string> temp_ply_files;
						getAllFiles(temp_data_path, file_formats[0], temp_ply_files, temp_file_folders);//get .ply filenames
					}

					if (file_folders.size() > 1)
					{
						if (processing_mode == 2)
							path_tmp = temp_file_folders[mi] + tex_i;
						else
							path_tmp = file_folders[mi] + tex_i;
					}
					else
					{
						if (processing_mode == 2)
							path_tmp = temp_file_folders[0] + tex_i;
						else
							path_tmp = file_folders[0] + tex_i;
					}

					texture_str_ostemp << path_tmp;
					std::string texture_str_temp = texture_str_ostemp.str().data();
					char * texturePath_temp = (char *)texture_str_temp.data();
					cv::Mat texture_map = cv::imread(texturePath_temp);
					if (texture_map.empty())
					{
						std::cout << "read texture failure or no texture provide!" << std::endl;
						cv::Mat dummy(128, 128, CV_8UC3, cv::Scalar(0, 255, 0));
						texture_map = dummy;
					}
					texture_maps.emplace_back(texture_map);

					if (save_textures_in_predict)
					{
						std::string current_write_path = basic_write_path + "/";
						current_write_path += tex_i;
						std::ostringstream write_texture_str_ostemp;
						write_texture_str_ostemp << current_write_path;
						std::string write_texture_str_temp = write_texture_str_ostemp.str().data();
						char * write_texturePath_temp = (char *)write_texture_str_temp.data();
						cv::imwrite(write_texturePath_temp, texture_map);
					}
				}
			}
		}

		//get over-segmented mesh information
		if (use_existing_mesh_segments)
		{
			if (smesh_seg->get_face_property<int>("f:face_segment_id"))
				smesh_seg->get_face_segment_id = smesh_seg->get_face_property<int>("f:face_segment_id");
			else
			{
				smesh_seg->add_face_property<int>("f:face_segment_id", -1);
				smesh_seg->get_face_segment_id = smesh_seg->get_face_property<int>("f:face_segment_id");
			}
		}

		//pre-compute mesh properties
		smesh->class_area.resize(labels_name.size());
		for (auto fi : smesh->faces())
		{
			smesh->get_face_normals[fi] = smesh->compute_face_normal(fi);
			smesh->get_face_area[fi] = FaceArea(smesh, fi);
			smesh->get_face_tile_index[fi] = mi;

			//unclassified = 0, unlabeled = -1
			if (smesh->get_face_truth_label[fi] != 0 && smesh->get_face_truth_label[fi] != -1)
			{
				smesh->mesh_area += smesh->get_face_area[fi];
				smesh->class_area[smesh->get_face_truth_label[fi] - 1] += smesh->get_face_area[fi];
			}

			if (use_existing_mesh_segments)
				smesh->get_face_segment_id[fi] = smesh_seg->get_face_segment_id[fi];
		}
	}

	//--- add feature information on mesh faces for visualization ---
	void visualization_feature_on_mesh
	(
		SFMesh *smesh_in,
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		SFMesh *smesh_out
	)
	{
		smesh_out->assign(*smesh_in);
		std::map<int, int> segment_id_vecind;//segment_id and index in the vector
		smesh_out->add_selected_feature_properties
		(
			seg_face_vec,
			seg_truth,
			basic_feas,
			eigen_feas,
			color_feas,
			mulsc_ele_feas
		);
	}

	//--- parsing PTCloud to cgal point cloud ---
	void parsing_to_pcl
	(
		PTCloud *pcl_out,
		Point_range& point_set
	)
	{
		auto pt_it = point_set.index_back_inserter();

		pcl_out->get_points_coord = pcl_out->get_vertex_property<vec3>("v:point");
		pcl_out->get_points_normals = pcl_out->get_vertex_property<vec3>("v:normal");
		for (auto vi : pcl_out->vertices())
		{
			Point_3 p_tmp(pcl_out->get_points_coord[vi].x, pcl_out->get_points_coord[vi].y, pcl_out->get_points_coord[vi].z);
			Vector_3 n_tmp(pcl_out->get_points_normals[vi].x, pcl_out->get_points_normals[vi].y, pcl_out->get_points_normals[vi].z);
			point_set.insert(p_tmp, n_tmp);
		}
	}

	//--- parsing SFMesh to Mesh ---
	void parsing_to_mesh
	(
		SFMesh *smesh_out,
		Mesh &mesh_out
	)
	{
		smesh_out->get_points_coord = smesh_out->get_vertex_property<vec3>("v:point");
		for (auto vi : smesh_out->vertices())
		{
			mesh_out.add_vertex(Point_3(smesh_out->get_points_coord[vi].x, smesh_out->get_points_coord[vi].y, smesh_out->get_points_coord[vi].z));
		}

		for (auto fi : smesh_out->faces())
		{
			std::vector<unsigned int> indices;
			for (auto v : smesh_out->vertices(fi))
			{
				indices.push_back(v.idx());
			}

			Mesh::Vertex_index u(indices[0]);
			Mesh::Vertex_index v(indices[1]);
			Mesh::Vertex_index w(indices[2]);
			mesh_out.add_face(u, v, w);
		}
	}

	//--- parsing from point cloud to cgal cluster ---
	std::vector<Cluster_point> convert_to_CGAL_Cluster(std::vector< std::vector<float>> &basic_feas_train)
	{
		std::vector<Cluster_point> cluster_points;
		Point_set pcl_cgal;
		for (int i = 0; i < basic_feas_train.size(); ++i)
			pcl_cgal.insert(Point_3(basic_feas_train[i][0], basic_feas_train[i][1], basic_feas_train[i][2]));

		int idx = 0;
		for (Point_set::iterator pc_it = pcl_cgal.begin(); pc_it != pcl_cgal.end(); ++pc_it)
		{
			if (std::size_t(idx) >= cluster_points.size())
				cluster_points.resize(idx + 1, Cluster_point(pcl_cgal, pcl_cgal.point_map()));
			cluster_points[std::size_t(idx)].insert(idx);
			++idx;
		}

		return cluster_points;
	}

	//--- ignore labels ---
	void check_ignored_truth_labels()
	{
		//get shifted label index from ignored labels
		int new_li = 0;
		for (int li = 0; li < labels_name.size(); ++li)
		{
			label_ignore[li] = false;
			for (int ig_i = 0; ig_i < ignored_labels_name.size(); ++ig_i)
			{
				if (ignored_labels_name[ig_i] == labels_name[li])
				{
					label_ignore[li] = true;
					break;
				}
			}

			if (!label_ignore[li])
			{
				int count_pre_ignored = 0;
				for (int li2 = 0; li2 < li; ++li2)
				{
					for (int ig_i2 = 0; ig_i2 < ignored_labels_name.size(); ++ig_i2)
					{
						if (ignored_labels_name[ig_i2] == labels_name[li2])
						{
							++count_pre_ignored;
							break;
						}
					}
				}
				label_shiftdis[li] = count_pre_ignored;
				new_label_shiftback[new_li] = label_shiftdis[li];
				++new_li;
			}
		}
	}

	void ignore_truth_labels
	(
		std::vector<int> &seg_truth
	)
	{
		check_ignored_truth_labels();

		//Ignore labels by shifting all labels of input data
		//2->0, 6->1, 7->2, 8->3, 9->4
		for (int li = 0; li < seg_truth.size(); ++li)
		{
			if (label_ignore[seg_truth[li]])
			{
				seg_truth[li] = -1;
			}
			else
			{
				seg_truth[li] -= label_shiftdis[seg_truth[li]];
			}
		}
	}
}