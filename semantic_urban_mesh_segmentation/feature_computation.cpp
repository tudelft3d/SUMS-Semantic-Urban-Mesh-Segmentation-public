/*
*   Name        : feature_computation.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : handcrafted features extraction and texture processing
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


#include "feature_computation.hpp"
#include "over_segmentation.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
	//------------------------------------------------ Surface mesh sampling functions ----------------------------------------------//
	//------------------------------------------------ ------------------------------- ----------------------------------------------//
		//Calculate single facet feature property<float>
	void texture_pointcloud_generation
	(
		SFMesh* smesh_in,
		SFMesh::Face &fd,
		std::vector<cv::Mat> &texture_maps,
		std::vector<cv::Mat> &hsv_maps,
		PTCloud* tex_cloud
	)
	{
		int texture_id = smesh_in->get_face_texnumber[fd];
		double a_g = 0.0;

		vec3 p1, p2, p3;
		int vind = 0;
		for (auto v : smesh_in->vertices(fd))
		{
			switch (vind)
			{
			case 0:
				p1 = smesh_in->get_points_coord[v];
				++vind;
				break;
			case 1:
				p2 = smesh_in->get_points_coord[v];
				++vind;
				break;
			case 2:
				p3 = smesh_in->get_points_coord[v];
				++vind;
				break;
			default:
				break;
			}
		}

		std::vector<vec2> uv_triangle;
		std::vector<float> U_vec, V_vec;

		U_vec.push_back(smesh_in->get_face_texcoord[fd][0]);
		U_vec.push_back(smesh_in->get_face_texcoord[fd][2]);
		U_vec.push_back(smesh_in->get_face_texcoord[fd][4]);
		V_vec.push_back(smesh_in->get_face_texcoord[fd][1]);
		V_vec.push_back(smesh_in->get_face_texcoord[fd][3]);
		V_vec.push_back(smesh_in->get_face_texcoord[fd][5]);
		uv_triangle.emplace_back(U_vec[0], V_vec[0]);
		uv_triangle.emplace_back(U_vec[1], V_vec[1]);
		uv_triangle.emplace_back(U_vec[2], V_vec[2]);

		std::vector<vec3> coord3d_triangle;
		coord3d_triangle.emplace_back(p1);
		coord3d_triangle.emplace_back(p2);
		coord3d_triangle.emplace_back(p3);

		std::vector<float> x_vec; std::vector<float> y_vec;
		std::copy(U_vec.begin(), U_vec.end(), std::back_inserter(x_vec));
		std::copy(V_vec.begin(), V_vec.end(), std::back_inserter(y_vec));
		std::sort(x_vec.begin(), x_vec.end(), smaller_coord);
		std::sort(y_vec.begin(), y_vec.end(), smaller_coord);

		int x_dis = (x_vec[2] - x_vec[0]) * texture_pointcloud_density;
		int y_dis = (y_vec[2] - y_vec[0]) * texture_pointcloud_density;

		int count_pix = 0;
		float Rf_sum = 0.0f, Gf_sum = 0.0f, Bf_sum = 0.0f, Hf_sum = 0.0f, Sf_sum = 0.0f, Vf_sum = 0.0f;
		for (int xi = 0; xi < x_dis; ++xi)
		{
			for (int yi = 0; yi < y_dis; ++yi)
			{
				std::vector<float> P;
				P.push_back(x_vec[0] + xi / texture_pointcloud_density);
				P.push_back(y_vec[0] + yi / texture_pointcloud_density);

				if (P[0] > 1.0f || P[0] < 0.0f || P[1] > 1.0f || P[1] < 0.0f)
				{
					P.clear();
					continue;
				}

				if (PointinTriangle(U_vec, V_vec, P))
				{
					//get 3d coordinates
					vec2 newcoord(x_vec[0] + xi / texture_pointcloud_density, y_vec[0] + yi / texture_pointcloud_density);
					vec3 current_3d;
					uv_to_3D_coordinates(uv_triangle, coord3d_triangle, newcoord, current_3d);

					//get matched pixel
					float Rf_temp = (float)texture_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*texture_maps[texture_id].rows - 1, newcoord[0] * texture_maps[texture_id].cols)[2];
					float Gf_temp = (float)texture_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*texture_maps[texture_id].rows - 1, newcoord[0] * texture_maps[texture_id].cols)[1];
					float Bf_temp = (float)texture_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*texture_maps[texture_id].rows - 1, newcoord[0] * texture_maps[texture_id].cols)[0];
					float Hf_temp = (float)hsv_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*hsv_maps[texture_id].rows - 1, newcoord[0] * hsv_maps[texture_id].cols)[0];
					float Sf_temp = (float)hsv_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*hsv_maps[texture_id].rows - 1, newcoord[0] * hsv_maps[texture_id].cols)[1];
					float Vf_temp = (float)hsv_maps[texture_id].at<cv::Vec3b>((1 - newcoord[1])*hsv_maps[texture_id].rows - 1, newcoord[0] * hsv_maps[texture_id].cols)[2];

					Rf_sum += Rf_temp;
					Gf_sum += Gf_temp;
					Bf_sum += Bf_temp;
					Hf_sum += Hf_temp * 2.0f;
					Sf_sum += Sf_temp / 255.0f * 100.0f;
					Vf_sum += Vf_temp / 255.0f * 100.0f;

					//create texture cloud
					if (tex_cloud != nullptr)
					{
						tex_cloud->add_vertex(current_3d);
						tex_cloud->get_points_color[*(--tex_cloud->vertices_end())] = vec3(Rf_temp / 255.0f, Gf_temp / 255.0f, Bf_temp / 255.0f);

						tex_cloud->get_points_rgb_x[*(--tex_cloud->vertices_end())] = Rf_temp;
						tex_cloud->get_points_rgb_y[*(--tex_cloud->vertices_end())] = Gf_temp;
						tex_cloud->get_points_rgb_z[*(--tex_cloud->vertices_end())] = Bf_temp;
						tex_cloud->get_points_hsv_x[*(--tex_cloud->vertices_end())] = Hf_temp * 2.0f;
						tex_cloud->get_points_hsv_y[*(--tex_cloud->vertices_end())] = Sf_temp / 255.0f * 100.0f;
						tex_cloud->get_points_hsv_z[*(--tex_cloud->vertices_end())] = Vf_temp / 255.0f * 100.0f;

						tex_cloud->get_points_face_belong_id[*(--tex_cloud->vertices_end())] = fd.idx();
						tex_cloud->get_points_ground_truth[*(--tex_cloud->vertices_end())] = smesh_in->get_face_truth_label[fd];
						smesh_in->get_face_texture_points[fd].push_back(tex_cloud->vertices_size());
					}

					smesh_in->get_face_color[fd] += vec3(Rf_temp / 255.0f, Gf_temp / 255.0f, Bf_temp / 255.0f);
					if (use_face_pixels_color_aggregation)
					{
						smesh_in->get_face_rgb_x[fd].push_back(Rf_temp);
						smesh_in->get_face_rgb_y[fd].push_back(Gf_temp);
						smesh_in->get_face_rgb_z[fd].push_back(Bf_temp);
						smesh_in->get_face_hsv_x[fd].push_back(Hf_temp * 2.0f);
						smesh_in->get_face_hsv_y[fd].push_back(Sf_temp / 255.0f * 100.0f);
						smesh_in->get_face_hsv_z[fd].push_back(Vf_temp / 255.0f * 100.0f);
					}

					++count_pix;
				}
				P.clear();
			}
		}

		if (count_pix == 0)
		{
			smesh_in->get_face_color[fd] = vec3();
			if (!use_face_pixels_color_aggregation)
			{
				smesh_in->get_face_rgb_x[fd].push_back(0.0f);
				smesh_in->get_face_rgb_y[fd].push_back(0.0f);
				smesh_in->get_face_rgb_z[fd].push_back(0.0f);
				smesh_in->get_face_hsv_x[fd].push_back(0.0f);
				smesh_in->get_face_hsv_y[fd].push_back(0.0f);
				smesh_in->get_face_hsv_z[fd].push_back(0.0f);
			}
		}
		else
		{
			smesh_in->get_face_color[fd] /= float(count_pix);
			if (!use_face_pixels_color_aggregation)
			{
				smesh_in->get_face_rgb_x[fd].push_back(Rf_sum / float(count_pix));
				smesh_in->get_face_rgb_y[fd].push_back(Gf_sum / float(count_pix));
				smesh_in->get_face_rgb_z[fd].push_back(Bf_sum / float(count_pix));
				smesh_in->get_face_hsv_x[fd].push_back(Hf_sum / float(count_pix));
				smesh_in->get_face_hsv_y[fd].push_back(Sf_sum / float(count_pix));
				smesh_in->get_face_hsv_z[fd].push_back(Vf_sum / float(count_pix));
			}
		}
	}

	//--- get face texture colors ---
	void face_texture_processor
	(
		SFMesh *smesh_out,
		std::vector<cv::Mat> &texture_maps,
		const int mi,
		PTCloud* tex_cloud
	)
	{
		std::cout << "	- Preprocessing of mesh textures: ";
		const double t_total = omp_get_wtime();

		std::vector<cv::Mat> hsv_maps, lab_maps;
		for (int i = 0; i < texture_maps.size(); i++)
		{
			cv::Mat hsv_temp = cv::Mat::zeros(texture_maps[i].size(), texture_maps[i].type());
			cv::cvtColor(texture_maps[i], hsv_temp, cv::COLOR_BGR2HSV);
			hsv_maps.emplace_back(hsv_temp);
		}

		int prev_percent = -1;
		for (auto fd : smesh_out->faces())
		{
			int progress = int(100.0f * (fd.idx() + 1) / float(smesh_out->n_faces()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			if (add_point_color)
				texture_pointcloud_generation(smesh_out, fd, texture_maps, hsv_maps, tex_cloud);
			else
				texture_pointcloud_generation(smesh_out, fd, texture_maps, hsv_maps);
		}
		std::cout << "Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	void parsing_texture_color_to_sampled_pointcloud
	(
		PTCloud *cloud_3d,
		PTCloud *tex_cloud,
		const int ni,
		PTCloud::Vertex &ptx
	)
	{
		PTCloud::Vertex ntx(ni);

		cloud_3d->get_points_color[ptx] = tex_cloud->get_points_color[ntx];
		cloud_3d->get_points_rgb_x[ptx] = tex_cloud->get_points_rgb_x[ntx];
		cloud_3d->get_points_rgb_y[ptx] = tex_cloud->get_points_rgb_y[ntx];
		cloud_3d->get_points_rgb_z[ptx] = tex_cloud->get_points_rgb_z[ntx];
		cloud_3d->get_points_hsv_x[ptx] = tex_cloud->get_points_hsv_x[ntx];
		cloud_3d->get_points_hsv_y[ptx] = tex_cloud->get_points_hsv_y[ntx];
		cloud_3d->get_points_hsv_z[ptx] = tex_cloud->get_points_hsv_z[ntx];
	}

	//get sampling cloud from mesh faces, assign elevation face sampled points as well
	void match_ele_sampling_pointcloud_with_mesh_faces
	(
		SFMesh *smesh_out,
		PTCloud *cloud_3d_pt,
		PTCloud *face_center_cloud,
		easy3d::KdTree *center_cloud_tree
	)
	{
		//one point can be shared by multiple faces
		easy3d::KdTree *cloud_tree = new easy3d::KdTree;
		Build_kdtree(cloud_3d_pt, cloud_tree);
		//each face should has one corresponding elevation point
		int prev_percent = -1;
		for (auto f_cur : smesh_out->faces())
		{
			int progress = int(100.0f * (f_cur.idx() + 1) / float(smesh_out->faces_size()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			int cur_id = cloud_tree->find_closest_point(smesh_out->get_face_center[f_cur]);
			PTCloud::Vertex p_cur(cur_id);
			cloud_3d_pt->get_points_face_ele_belong_ids[p_cur].push_back(f_cur.idx());
			smesh_out->get_face_ele_sampled_points[f_cur].push_back(p_cur.idx());
		}
		delete cloud_tree;

		//each elevation point should has one corresponding face
		cloud_3d_pt->get_points_coord = cloud_3d_pt->get_vertex_property<vec3>("v:point");
		prev_percent = -1;
		for (auto ptx : cloud_3d_pt->vertices())
		{
			int progress = int(100.0f * (ptx.idx() + 1) / float(cloud_3d_pt->vertices_size()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			int cur_id = center_cloud_tree->find_closest_point(cloud_3d_pt->get_points_coord[ptx]);
			SFMesh::Vertex v_cen(cur_id);
			cloud_3d_pt->get_points_face_belong_id[ptx] = v_cen.idx();
			SFMesh::Face f_cen(v_cen.idx());
			smesh_out->get_face_ele_sampled_points[f_cen].push_back(ptx.idx());
		}
	}

	//get sampling cloud normals from mesh faces, assign face sampled points as well
	void get_sampling_cloud_normals_from_mesh_faces
	(
		SFMesh *smesh_out,
		PTCloud *cloud_3d_pt,
		PTCloud *face_center_cloud,
		PTCloud *tex_cloud,
		easy3d::KdTree *center_cloud_tree,
		easy3d::KdTree *tex_tree
	)
	{
		smesh_out->get_points_coord = smesh_out->get_vertex_property<vec3>("v:point");
		smesh_out->get_face_normals = smesh_out->get_face_property<vec3>("f:normal");

		cloud_3d_pt->add_vertex_property<vec3>("v:color");
		cloud_3d_pt->get_points_color = cloud_3d_pt->get_vertex_property<vec3>("v:color");
		cloud_3d_pt->add_vertex_property<vec3>("v:normal");
		cloud_3d_pt->get_points_normals = cloud_3d_pt->get_vertex_property<vec3>("v:normal");
		cloud_3d_pt->get_points_coord = cloud_3d_pt->get_vertex_property<vec3>("v:point");

		int prev_percent = -1;
		for (auto p_cur : cloud_3d_pt->vertices())
		{
			int progress = int(100.0f * (p_cur.idx() + 1) / float(cloud_3d_pt->n_vertices()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			int neighbor_id = center_cloud_tree->find_closest_point(cloud_3d_pt->get_points_coord[p_cur]);
			PTCloud::Vertex p_cen_n(neighbor_id);
			cloud_3d_pt->get_points_normals[p_cur] = face_center_cloud->get_points_normals[p_cen_n];
			cloud_3d_pt->get_points_face_belong_id[p_cur] = face_center_cloud->get_points_face_belong_id[p_cen_n];
			int f_cur_ind = face_center_cloud->get_points_face_belong_id[p_cen_n];
			SFMesh::Face f_cur(f_cur_ind);
			smesh_out->get_face_sampled_points[f_cur].push_back(p_cur.idx());

			if (train_test_predict_val != 2)
				cloud_3d_pt->get_points_ground_truth[p_cur] = smesh_out->get_face_truth_label[f_cur];
			if (with_texture && add_point_color)
			{
				int closet_ind = tex_tree->find_closest_point(cloud_3d_pt->get_points_coord[p_cur]);
				parsing_texture_color_to_sampled_pointcloud(cloud_3d_pt, tex_cloud, closet_ind, p_cur);
			}
		}

		//each face should have one point
		//for face do not have sampled points, use face center instead
		easy3d::KdTree *sampled_cloud_tree = new easy3d::KdTree;
		Build_kdtree(cloud_3d_pt, sampled_cloud_tree);
		face_center_cloud->get_points_coord = face_center_cloud->get_vertex_property<vec3>("v:point");

		for (auto pt : face_center_cloud->vertices())
		{
			int neighbor_id = sampled_cloud_tree->find_closest_point(face_center_cloud->get_points_coord[pt]);
			PTCloud::Vertex p_cen_n(neighbor_id);
			SFMesh::Face f_cur(pt.idx());

			if (smesh_out->get_face_sampled_points[f_cur].empty())
				smesh_out->get_face_sampled_points[f_cur].push_back(p_cen_n.idx());
		}

		delete sampled_cloud_tree;
	}

	void finalization_sampling_point_cloud
	(
		PTCloud *cloud_3d_sampled,
		PTCloud *cloud_3d_ele,
		easy3d::PointCloud* initial_sampled_sampled_point_cloud,
		easy3d::PointCloud* initial_ele_sampled_point_cloud
	)
	{
		auto ini_sampled_point_coord = initial_sampled_sampled_point_cloud->get_vertex_property<vec3>("v:point");
		auto ini_ele_point_coord = initial_ele_sampled_point_cloud->get_vertex_property<vec3>("v:point");

		//for sampled point cloud sampling
		for (auto gp : initial_sampled_sampled_point_cloud->vertices())
			cloud_3d_sampled->add_vertex(ini_sampled_point_coord[gp]);

		//for elevation point cloud sampling
		for (auto gp : initial_ele_sampled_point_cloud->vertices())
			cloud_3d_ele->add_vertex(ini_ele_point_coord[gp]);
	}

	void finalization_sampling_point_cloud
	(
		PTCloud *cloud_3d,
		easy3d::PointCloud* initial_sampled_point_cloud
	)
	{
		auto ini_point_coord = initial_sampled_point_cloud->get_vertex_property<vec3>("v:point");

		for (auto gp : initial_sampled_point_cloud->vertices())
			cloud_3d->add_vertex(ini_point_coord[gp]);
	}

	//--- merge point cloud ---
	void merge_pointcloud
	(
		PTCloud *cloud_tmp,
		PTCloud *cloud_all,
		SFMesh *smesh_all,
		int &pre_face_size,
		const int sampled_or_ele_or_others, //0: sampled; 1: ele; 2: others
		std::map<int, int> &ptidx_faceid_map_all
	)
	{
		for (auto ptx : cloud_tmp->vertices())
		{
			cloud_all->add_vertex(cloud_tmp->get_points_coord[ptx]);
			cloud_all->get_points_coord[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_coord[ptx];
			if (sampled_or_ele_or_others == 0)
			{
				cloud_all->get_points_color[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_color[ptx];
				cloud_all->get_points_normals[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_normals[ptx];

				cloud_all->get_points_rgb_x[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_rgb_x[ptx];
				cloud_all->get_points_rgb_y[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_rgb_y[ptx];
				cloud_all->get_points_rgb_z[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_rgb_z[ptx];
				cloud_all->get_points_hsv_x[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_hsv_x[ptx];
				cloud_all->get_points_hsv_y[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_hsv_y[ptx];
				cloud_all->get_points_hsv_z[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_hsv_z[ptx];

				cloud_all->get_points_ground_truth[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_ground_truth[ptx];
				cloud_all->get_points_tile_index[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_tile_index[ptx];
				cloud_all->get_points_face_belong_id[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_face_belong_id[ptx] + pre_face_size;

				cloud_all->get_points_on_mesh_border[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_on_mesh_border[ptx];
			}
			else if (sampled_or_ele_or_others == 1)
			{
				cloud_all->get_points_face_ele_belong_ids[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_face_ele_belong_ids[ptx];
				cloud_all->get_points_tile_index[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_tile_index[ptx];
				for (int i = 0; i < cloud_tmp->get_points_face_ele_belong_ids[ptx].size(); ++i)
				{
					int fi = cloud_tmp->get_points_face_ele_belong_ids[ptx][i] + pre_face_size;
					SFMesh::Face fdx(fi);
					//smesh_all->get_face_ele_sampled_points_id_index_map[fdx][(*(--cloud_all->vertices_end())).idx()] = smesh_all->get_face_ele_sampled_points[fdx].size();
					smesh_all->get_face_ele_sampled_points[fdx].push_back((*(--cloud_all->vertices_end())).idx());
				}
			}
			else
			{
				cloud_all->get_points_face_belong_id[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_face_belong_id[ptx] + pre_face_size;
				cloud_all->get_points_tile_index[*(--cloud_all->vertices_end())] = cloud_tmp->get_points_tile_index[ptx];
				ptidx_faceid_map_all[(*(--cloud_all->vertices_end())).idx()] = cloud_all->get_points_face_belong_id[*(--cloud_all->vertices_end())];
			}
		}
	}

	//--- merge mesh ---
	void merge_mesh
	(
		SFMesh *smesh_tmp,
		SFMesh *smesh_all,
		int &vert_ind,
		int &pre_sampled_cloud_size,
		int &tex_size,
		SFMesh *smesh_overseg
	)
	{
		//merge mesh
		smesh_tmp->add_vertex_property<bool>("v:visited_vertex", false);
		auto get_visited_vertex = smesh_tmp->get_vertex_property<bool>("v:visited_vertex");

		smesh_all->textures.insert(smesh_all->textures.end(), smesh_tmp->textures.begin(), smesh_tmp->textures.end());
		smesh_all->texture_names.insert(smesh_all->texture_names.end(), smesh_tmp->texture_names.begin(), smesh_tmp->texture_names.end());
		std::map<SFMesh::Vertex, SFMesh::Vertex> vert_in_out;
		for (auto f : smesh_tmp->faces())
		{
			std::vector<int> vertex_indices;
			std::vector<vec3> temp_verts;
			for (auto v : smesh_tmp->vertices(f))
			{
				temp_verts.push_back(smesh_tmp->get_points_coord[v]);
				if (get_visited_vertex[v] == false)
				{
					SFMesh::Vertex vtx(vert_ind);
					smesh_all->add_vertex(smesh_tmp->get_points_coord[v]);
					get_visited_vertex[v] = true;
					vert_in_out[v] = vtx;
					vertex_indices.push_back(vert_ind++);
				}
				else
				{
					auto it = vert_in_out.find(v);
					vertex_indices.push_back(it->second.idx());
				}
			}
			smesh_all->add_triangle(SFMesh::Vertex(vertex_indices[0]), SFMesh::Vertex(vertex_indices[1]), SFMesh::Vertex(vertex_indices[2]));
			smesh_all->get_face_texnumber[*(--smesh_all->faces_end())] = tex_size + smesh_tmp->get_face_texnumber[f];
			smesh_all->get_face_texcoord[*(--smesh_all->faces_end())] = smesh_tmp->get_face_texcoord[f];
			smesh_all->get_face_truth_label[*(--smesh_all->faces_end())] = smesh_tmp->get_face_truth_label[f];
			smesh_all->get_face_predict_label[*(--smesh_all->faces_end())] = smesh_tmp->get_face_predict_label[f];
			smesh_all->get_face_normals[*(--smesh_all->faces_end())] = smesh_tmp->get_face_normals[f];
			smesh_all->get_face_tile_index[*(--smesh_all->faces_end())] = smesh_tmp->get_face_tile_index[f];
			smesh_all->get_face_area[*(--smesh_all->faces_end())] = smesh_tmp->get_face_area[f];
			smesh_all->get_face_center[*(--smesh_all->faces_end())] = smesh_tmp->get_face_center[f];

			//pre_sampled_cloud_size
			for (int si = 0; si < smesh_tmp->get_face_sampled_points[f].size(); ++si)
			{
				int pi_ind = smesh_tmp->get_face_sampled_points[f][si] + pre_sampled_cloud_size;
				smesh_all->get_face_sampled_points_id_index_map[*(--smesh_all->faces_end())][pi_ind] = smesh_all->get_face_sampled_points[*(--smesh_all->faces_end())].size();
				smesh_all->get_face_sampled_points[*(--smesh_all->faces_end())].push_back(pi_ind);
			}

			if (use_existing_mesh_segments)
				smesh_all->get_face_segment_id[*(--smesh_all->faces_end())] = smesh_overseg->get_face_segment_id[*(--smesh_all->faces_end())];

			smesh_all->get_face_rgb_x[*(--smesh_all->faces_end())] = smesh_tmp->get_face_rgb_x[f];
			smesh_all->get_face_rgb_y[*(--smesh_all->faces_end())] = smesh_tmp->get_face_rgb_y[f];
			smesh_all->get_face_rgb_z[*(--smesh_all->faces_end())] = smesh_tmp->get_face_rgb_z[f];
			smesh_all->get_face_hsv_x[*(--smesh_all->faces_end())] = smesh_tmp->get_face_hsv_x[f];
			smesh_all->get_face_hsv_y[*(--smesh_all->faces_end())] = smesh_tmp->get_face_hsv_y[f];
			smesh_all->get_face_hsv_z[*(--smesh_all->faces_end())] = smesh_tmp->get_face_hsv_z[f];

		}
		smesh_tmp->remove_vertex_property(get_visited_vertex);
	}

	void feature_batch_normalization(std::vector< std::vector< float > > &geof)
	{
		//Data distribution normalization(Batch normalization), get minmax in order to normalize to the range [0,1]
		std::vector<std::pair<float, float>> mean_sd_vecs(geof[0].size(), std::pair<float, float>(0.0f, 0.0f));

		//sum
		for (int i = 0; i < geof.size(); ++i)
			for (int j = 0; j < geof[i].size(); ++j)
				mean_sd_vecs[j].first += geof[i][j];

		//mean
		for (int j = 0; j < mean_sd_vecs.size(); ++j)
			mean_sd_vecs[j].first /= float(geof.size());

		//var
		for (int i = 0; i < geof.size(); ++i)
			for (int j = 0; j < geof[i].size(); ++j)
				mean_sd_vecs[j].second += std::pow((geof[i][j] - mean_sd_vecs[j].first), 2.0f);

		//std
		for (int j = 0; j < mean_sd_vecs.size(); ++j)
			mean_sd_vecs[j].second = std::sqrt(mean_sd_vecs[j].second / float(geof.size()));

		//batch normalization
		std::vector< std::vector< float > > geof_norm(geof.size(), std::vector< float >(geof[0].size(), 0.0f));
		std::vector<std::pair<float, float>> minmax_vecs(geof[0].size(), std::pair<float, float>(FLT_MAX, -FLT_MAX));

		for (int i = 0; i < geof.size(); ++i)
		{
			for (int j = 0; j < geof[i].size(); ++j)
			{
				geof_norm[i][j] = (geof[i][j] - mean_sd_vecs[j].first) / mean_sd_vecs[j].second;
				minmax_vecs[j].first = minmax_vecs[j].first < geof_norm[i][j] ? minmax_vecs[j].first : geof_norm[i][j];
				minmax_vecs[j].second = minmax_vecs[j].second > geof_norm[i][j] ? minmax_vecs[j].second : geof_norm[i][j];
			}
		}

		//minmax normalize batch normalization
		for (int i = 0; i < geof.size(); ++i)
			for (int j = 0; j < geof[i].size(); ++j)
			{
				geof[i][j] = (geof_norm[i][j] - minmax_vecs[j].first) / (minmax_vecs[j].second - minmax_vecs[j].first);
				value_validation_check(geof[i][j]);
			}
	}

	//find local neighbor elevation
	void local_elevation_for_pointcloud
	(
		SFMesh *smesh_in,
		PTCloud* cloud_pt_3d,
		std::vector<superfacets>& spf_final
	)
	{
		//update copy point cloud with segment average elevation in majority facet favor. 
		PointCloud *cloud_copy = new PointCloud, *cloud2Dproj = new PointCloud;
		cloud_copy->assign(*cloud_pt_3d);
		cloud2Dproj->assign(*cloud_pt_3d);
		auto cloud_point_coord = cloud_pt_3d->get_vertex_property<vec3>("v:point");
		cloud_pt_3d->add_vertex_property<int>("v:segment_majority_id", -1);
		auto get_segment_majority_id = cloud_pt_3d->get_vertex_property<int>("v:segment_majority_id");
#pragma omp parallel for schedule(dynamic)
		for (int vi = 0; vi < cloud_pt_3d->vertices_size(); ++vi)
		{
			PTCloud::Vertex ptx(vi);
			//get segment index
			std::map<int, int> seg_count_majority;//id, count
			std::pair<int, int> seg_maxcount(-1, 0);//id, max count
			for (int i = 0; i < cloud_pt_3d->get_points_face_ele_belong_ids[ptx].size(); ++i)
			{
				int fi = cloud_pt_3d->get_points_face_ele_belong_ids[ptx][i];
				SFMesh::Face fdx(fi);
				int seg_id = smesh_in->get_face_segment_id[fdx];
				auto it_s = seg_count_majority.find(seg_id);
				if (it_s == seg_count_majority.end())
					seg_count_majority[seg_id] = 1;
				else
					++seg_count_majority[seg_id];

				if (seg_count_majority[seg_id] > seg_maxcount.second)
				{
					seg_maxcount.first = seg_id;
					seg_maxcount.second = seg_count_majority[seg_id];
				}
			}

			int seg_ind = superfacet_id_index_map[seg_maxcount.first];
			get_segment_majority_id[ptx] = seg_maxcount.first;
			cloud_copy->get_vertex_property<vec3>("v:point")[ptx].z = spf_final[seg_ind].avg_ele;
			cloud2Dproj->get_vertex_property<vec3>("v:point")[ptx].z = 0.0f;
		}

		//for compute cloud local elevation
		easy3d::KdTree *cloud_2D_proj_tree = new easy3d::KdTree;
		Build_kdtree(cloud2Dproj, cloud_2D_proj_tree);
		//long range
		float sqr_long_range = long_range_radius_default * long_range_radius_default;
		cloud_pt_3d->add_vertex_property<std::vector<std::pair<int, float>>>("v:pid_local_longrange_zmin", std::vector<std::pair<int, float>>(local_ground_segs, std::pair<int, float>(-1, 999999.0f)));
		auto get_points_pid_local_longrange_zmin = cloud_pt_3d->get_vertex_property<std::vector<std::pair<int, float>>>("v:pid_local_longrange_zmin");
		//find local lowest point cloud each point
#pragma omp parallel for schedule(dynamic)
		for (int vi = 0; vi < cloud_pt_3d->vertices_size(); ++vi)
		{
			//for compute local elevation
			PTCloud::Vertex ptx(vi);
			std::vector<int> neighbor_indices;
			vec2 p2d(cloud_point_coord[ptx].x, cloud_point_coord[ptx].y);
			//long range 
			cloud_2D_proj_tree->find_points_in_radius(p2d, sqr_long_range, neighbor_indices, cloud_copy, get_points_pid_local_longrange_zmin[ptx]);

			//multi-scales 2 elevation computation
			for (int sc_i = 0; sc_i < multi_scale_ele_radius.size(); ++sc_i)
			{
				float sqr_scale_range = std::pow(multi_scale_ele_radius[sc_i], 2.0f);
				neighbor_indices.clear();
				cloud_2D_proj_tree->find_points_in_radius_minmax(p2d, sqr_scale_range, neighbor_indices, cloud_copy, cloud_pt_3d->get_point_mulscale_2_elevation[ptx][sc_i]);
			}
		}

		//get first [local_ground_segs] local lowest point with its ids for each face
#pragma omp parallel for schedule(dynamic)
		for (int fi = 0; fi < smesh_in->faces_size(); ++fi)
		{
			SFMesh::Face fdx(fi);
			std::vector<std::pair<int, float>> face_segid_local_longrange_elevation_vec(smesh_in->get_face_ele_sampled_points[fdx].size() * local_ground_segs, std::pair<int, float>(-1, FLT_MAX));
			std::vector<std::pair<int, float>> face_segid_local_shortrange_elevation_vec(smesh_in->get_face_ele_sampled_points[fdx].size() * local_ground_segs, std::pair<int, float>(-1, FLT_MAX));
			std::map<int, bool> seg_longrange_check, seg_shortrange_check;
			int vec_longrange_ind = 0, vec_shortrange_ind = 0;
			for (int vi = 0; vi < smesh_in->get_face_ele_sampled_points[fdx].size(); ++vi)
			{
				PTCloud::Vertex ptx(smesh_in->get_face_ele_sampled_points[fdx][vi]);
				//get local ground points
				//long range
				for (int psi = 0; psi < get_points_pid_local_longrange_zmin[ptx].size(); ++psi)
				{
					if (get_points_pid_local_longrange_zmin[ptx][psi].first != -1)
					{
						PTCloud::Vertex lcoal_ptx(get_points_pid_local_longrange_zmin[ptx][psi].first);
						int local_segi = get_segment_majority_id[lcoal_ptx];

						auto it_s = seg_longrange_check.find(local_segi);
						if (it_s == seg_longrange_check.end())
						{
							face_segid_local_longrange_elevation_vec[vec_longrange_ind].first = local_segi;
							face_segid_local_longrange_elevation_vec[vec_longrange_ind++].second = get_points_pid_local_longrange_zmin[ptx][psi].second;
							seg_longrange_check[local_segi] = true;
						}
					}
					else
						break;
				}

				//get multi scale 2 relative elevation
				for (int sc_i = 0; sc_i < multi_scale_ele_radius.size(); ++sc_i)
				{
					if (cloud_pt_3d->get_point_mulscale_2_elevation[ptx][sc_i].first < smesh_in->get_face_mulsc_2_minmax_elevation[fdx][sc_i].first)
						smesh_in->get_face_mulsc_2_minmax_elevation[fdx][sc_i].first = cloud_pt_3d->get_point_mulscale_2_elevation[ptx][sc_i].first;
					if (cloud_pt_3d->get_point_mulscale_2_elevation[ptx][sc_i].second > smesh_in->get_face_mulsc_2_minmax_elevation[fdx][sc_i].second)
						smesh_in->get_face_mulsc_2_minmax_elevation[fdx][sc_i].second = cloud_pt_3d->get_point_mulscale_2_elevation[ptx][sc_i].second;
				}
			}

			//assign the local long range ground segments
			sort(face_segid_local_longrange_elevation_vec.begin(), face_segid_local_longrange_elevation_vec.end(), lower_local_elevation);
			int temp_size = local_ground_segs > face_segid_local_longrange_elevation_vec.size() ? face_segid_local_longrange_elevation_vec.size() : local_ground_segs;
			for (int psi = 0; psi < temp_size; ++psi)
			{
				if (face_segid_local_longrange_elevation_vec[psi].first != -1)
					smesh_in->get_face_segid_local_longrange_elevation_vec[fdx][psi] = face_segid_local_longrange_elevation_vec[psi];
			}
		}

		cloud_pt_3d->remove_vertex_property(get_points_pid_local_longrange_zmin);

		delete cloud_copy;
		delete cloud2Dproj;
		delete cloud_2D_proj_tree;
	}

	void medial_ball_features
	(
		SFMesh *smesh_out,
		PTCloud *cloud_3d_pt
	)
	{
		easy3d::KdTree *tree3d = new easy3d::KdTree;
		Build_kdtree(cloud_3d_pt, tree3d);
		cloud_3d_pt->get_points_coord = cloud_3d_pt->get_vertex_property<vec3>("v:point");
		cloud_3d_pt->get_points_normals = cloud_3d_pt->get_vertex_property<vec3>("v:normal");

		cloud_3d_pt->add_vertex_property<bool>("v:medialball_check", false);
		auto get_vert_medialball_check = cloud_3d_pt->get_vertex_property<bool>("v:medialball_check");

#pragma omp parallel for schedule(runtime)
		for (int vi = 0; vi < cloud_3d_pt->vertices_size(); ++vi)
		{
			PTCloud::Vertex v1(vi);

			if (get_vert_medialball_check[v1] == true)
				continue;
			get_vert_medialball_check[v1] = true;

			unsigned int count_iteration = 0;
			int qidx = -1, qidx_next;
			vec3 ball_center, ball_center_next;
			float radius = mat_initialized_radius, radius_next;
			vec3 p_current = cloud_3d_pt->get_points_coord[v1];
			vec3 n_current = cloud_3d_pt->get_points_normals[v1];
			ball_center = p_current - n_current * radius;//ball center initialize

			float dist = -2 * radius, separation_angle = 0.0f;

			while (true)
			{
				qidx_next = tree3d->find_closest_point(ball_center, dist);//squared distance
				PTCloud::Vertex vtx(qidx_next);
				const easy3d::vec3& q_nearest = cloud_3d_pt->get_points_coord[vtx];

				// This should handle all (special) cases where we want to break the loop
				// - normal case when ball no longer shrinks
				// - the case where q==p
				// - any duplicate point cases

				if ((dist >= (radius - mat_delta_convergance)*(radius - mat_delta_convergance))
					|| (p_current == q_nearest))
					break;

				// Compute next ball center
				radius_next = compute_radius(p_current, n_current, q_nearest);
				ball_center_next = p_current - n_current * radius_next;

				if (!std::isnormal(ball_center_next.x) && !std::isnormal(ball_center_next.y) && !std::isnormal(ball_center_next.z))
					break;

				// Denoising
				float angle_temp = cos_angle(p_current - ball_center_next, q_nearest - ball_center_next);
				separation_angle = std::acos(angle_temp);

				if (count_iteration == 0 && separation_angle < 32 * (M_PI / 180)) {//mat_denoising_seperation_angle
					break;
				}

				if (count_iteration > 0 && (separation_angle <  45 * (M_PI / 180) && radius_next >(q_nearest - p_current).norm()))
				{
					break;
				}

				// Stop iteration if this looks like an infinite loop:
				if (count_iteration > mat_iteration_limit_number)
					break;

				ball_center = ball_center_next;
				qidx = qidx_next;
				radius = radius_next;
				++count_iteration;
			}

			if (qidx == -1)
			{
				qidx = qidx_next;
			}

			PTCloud::Vertex vtx_now(qidx);
			//parsing to mesh

			int pt_ind_1 = -1, pt_ind_2 = -1;
			int f_1 = cloud_3d_pt->get_points_face_belong_id[v1];
			int f_2 = cloud_3d_pt->get_points_face_belong_id[vtx_now];
			SFMesh::Face temp_face_1(f_1);
			SFMesh::Face temp_face_2(f_2);

			pt_ind_1 = smesh_out->get_face_sampled_points_id_index_map[temp_face_1][v1.idx()];
			pt_ind_2 = smesh_out->get_face_sampled_points_id_index_map[temp_face_2][vtx_now.idx()];

			smesh_out->get_face_interior_medialball_radius[temp_face_1][pt_ind_1] = radius;
			smesh_out->get_face_interior_medialball_radius[temp_face_2][pt_ind_2] = radius;

			get_vert_medialball_check[vtx_now] = true;
		}

		cloud_3d_pt->remove_vertex_property(cloud_3d_pt->get_vertex_property<bool>("v:medialball_check"));
		delete tree3d;
	}

	//compute multiscale geometric features only on mesh vertices or sampled points
	void compute_geometric_features_only_per_scale
	(
		SFMesh* smesh_out,
		PTCloud* point_cloud,
		superfacets &spf_current,
		const int seg_i,
		std::vector<float> &geo_feas,
		std::vector< std::vector<float>> &basic_feas
	)
	{
		//--- geometric features define ---
		float eigen_1 = 0.0f, eigen_2 = 0.0f, eigen_3 = 0.0f, verticality = 0.0f, linearity = 0.0f, planarity = 0.0f, anisotropy = 0.0f,
			sphericity = 0.0f, eigenentropy = 0.0f, omnivariance = 0.0f, sumeigenvals = 0.0f, curvature = 0.0f,
			verticality_eig1 = 0.0f, verticality_eig3 = 0.0f, surface = 0.0f, volume = 0.0f,
			absolute_eigvec_1_moment_1st = 0.0f, absolute_eigvec_2_moment_1st = 0.0f, absolute_eigvec_3_moment_1st = 0.0f,
			absolute_eigvec_1_moment_2nd = 0.0f, absolute_eigvec_2_moment_2nd = 0.0f, absolute_eigvec_3_moment_2nd = 0.0f,
			vertical_moment_1st = 0.0f, vertical_moment_2nd = 0.0f, uniformity = 0.0f, points_to_plane_dist_mean = 0.0f;

		//--- get PCA eigen value and vectors, refer to PCA_utils.h from CGAL ---
		DiagonalizeTraits::Vector eigen_values;
		DiagonalizeTraits::Matrix eigen_vectors;
		Plane plane_c;
		//--- last column of eigen vector correspond to the largest eigen value in the last row ---
		PCA_EigenSolver(spf_current.sampled_points, point_cloud, eigen_values, eigen_vectors, plane_c);
		easy3d::vec3 c_valf(eigen_values[2], eigen_values[1], eigen_values[0]);
		easy3d::vec3 c_vec1(-eigen_vectors[6], eigen_vectors[7], -eigen_vectors[8]);
		easy3d::vec3 c_vec2(-eigen_vectors[3], -eigen_vectors[4], -eigen_vectors[5]);
		easy3d::vec3 c_vec3(-eigen_vectors[0], -eigen_vectors[1], -eigen_vectors[2]);
		easy3d::vec4 plane_param(plane_c.a(), plane_c.b(), plane_c.c(), -plane_c.d());
		spf_current.plane_parameter = plane_param;

		//--- compute eigen-based features -----
		//eig1>eig2>eig3
		eigen_1 = c_valf[0];//eig1
		eigen_2 = c_valf[1];//eig2
		eigen_3 = c_valf[2];//eig3
		verticality = compute_verticality_from_normal(c_vec3);
		linearity = 1.0f - (c_valf[0] - c_valf[1]) / c_valf[0];
		anisotropy = 1.0f - (c_valf[0] - c_valf[2]) / c_valf[0];
		planarity = 1.0f - (c_valf[1] - c_valf[2]) / c_valf[0];
		sphericity = (1.0f - c_valf[2] / c_valf[0]) < eigen_feature_cut_off ? 0.0f : (1.0f - c_valf[2] / c_valf[0]) / eigen_feature_cut_off;
		curvature = 1.0f - 3.0f * c_valf[2] / (c_valf[0] + c_valf[1] + c_valf[2]);// < eigen_feature_cut_off ? 0.0f : (1.0f - 3.0f * c_valf[2] / (c_valf[0] + c_valf[1] + c_valf[2])) / eigen_feature_cut_off;
		eigenentropy = -(c_valf[0] * log(c_valf[0]) + c_valf[1] * log(c_valf[1]) + c_valf[2] * log(c_valf[2]));
		omnivariance = pow(c_valf[0] * c_valf[1] * c_valf[2], 1.0 / 3.0);
		sumeigenvals = c_valf[0] + c_valf[1] + c_valf[2];
		verticality_eig1 = std::abs(M_PI / 2.0f - std::acos(cos_angle(c_vec1, easy3d::vec3(0, 0, 1))));
		verticality_eig3 = std::abs(M_PI / 2.0f - std::acos(cos_angle(c_vec3, easy3d::vec3(0, 0, 1))));
		surface = eigen_1 * eigen_2;
		volume = eigen_1 * eigen_2 * eigen_3;
		uniformity = c_valf[1] / c_valf[0];

		int pts_size = spf_current.sampled_points.size();
		auto point_3d_coord = point_cloud->get_vertex_property<vec3>("v:point");

		//compute geometric features
		for (auto nt_i : spf_current.sampled_points)
		{
			PTCloud::Vertex ntx(nt_i);
			//--- accumulate eigen moment features ---
			absolute_eigvec_1_moment_1st += easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec1);
			absolute_eigvec_2_moment_1st += easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec2);
			absolute_eigvec_3_moment_1st += easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec3);
			absolute_eigvec_1_moment_2nd += std::pow(easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec1), 2);
			absolute_eigvec_2_moment_2nd += std::pow(easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec2), 2);
			absolute_eigvec_3_moment_2nd += std::pow(easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), c_vec3), 2);
			vertical_moment_1st += easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), easy3d::vec3(0, 0, 1));
			vertical_moment_2nd += std::pow(easy3d::dot((point_3d_coord[ntx] - spf_current.avg_center), easy3d::vec3(0, 0, 1)), 2);

			//--- accumulate point to plane distance ---
			points_to_plane_dist_mean += dist2plane(plane_param, point_3d_coord[ntx]);
		}

		//--- average eigen moment features ---
		absolute_eigvec_1_moment_1st /= float(pts_size);
		absolute_eigvec_2_moment_1st /= float(pts_size);
		absolute_eigvec_3_moment_1st /= float(pts_size);
		absolute_eigvec_1_moment_2nd /= float(pts_size);
		absolute_eigvec_2_moment_2nd /= float(pts_size);
		absolute_eigvec_3_moment_2nd /= float(pts_size);
		vertical_moment_1st /= float(pts_size);
		vertical_moment_2nd /= float(pts_size);

		points_to_plane_dist_mean /= float(pts_size);
		//--- fill the supplementary basic features vector ---
		basic_feas[seg_i][6] = 0.0f;//points_to_plane_dist_mean

		//---  fill the eigen feature vector ---
		geo_feas[0] = eigen_1;
		geo_feas[1] = eigen_2;
		geo_feas[2] = eigen_3;
		geo_feas[3] = verticality;
		geo_feas[4] = linearity;
		geo_feas[5] = planarity;
		geo_feas[6] = sphericity;
		geo_feas[7] = anisotropy;
		geo_feas[8] = eigenentropy;
		geo_feas[9] = omnivariance;
		geo_feas[10] = sumeigenvals;
		geo_feas[11] = curvature;
		geo_feas[12] = verticality_eig1;
		geo_feas[13] = verticality_eig3;
		geo_feas[14] = surface;
		geo_feas[15] = volume;
		geo_feas[16] = absolute_eigvec_1_moment_1st;
		geo_feas[17] = absolute_eigvec_2_moment_1st;
		geo_feas[18] = absolute_eigvec_3_moment_1st;
		geo_feas[19] = absolute_eigvec_1_moment_2nd;
		geo_feas[20] = absolute_eigvec_2_moment_2nd;
		geo_feas[21] = absolute_eigvec_3_moment_2nd;
		geo_feas[22] = vertical_moment_1st;
		geo_feas[23] = vertical_moment_2nd;
		geo_feas[24] = uniformity;

		//--- value validate check ---
		for (int geof_i = 0; geof_i < geo_feas.size(); ++geof_i)
			value_validation_check(geo_feas[geof_i]);
	}

	void compute_radiometric_features_on_face_textures
	(
		SFMesh* smesh_out,
		superfacets &spf_current,
		const int seg_i,
		std::vector<float> &tex_feas
	)
	{
		//--- radiometric features define ---
		float red = 0.0f, green = 0.0f, blue = 0.0f, hue = 0.0f, sat = 0.0f, val = 0.0f,
			greenness = 0.0f, red_var = 0.0f, green_var = 0.0f, blue_var = 0.0f, hue_var = 0.0f, sat_var = 0.0f, val_var = 0.0f, greenness_var = 0.0f;

		std::vector<std::vector<int>> hsv_bins_count_vec = { std::vector<int>(hsv_bins[0], 0), std::vector<int>(hsv_bins[1], 0), std::vector<int>(hsv_bins[2], 0) };
		std::vector<int> interval_vec = { 360 / hsv_bins[0], 100 / hsv_bins[1], 100 / hsv_bins[2] };

		int count_pixels = 0;
		for (auto fd : spf_current.face_vec)
		{
			//parsing back to face pixels
			for (int pix_i = 0; pix_i < smesh_out->get_face_rgb_x[fd].size(); ++pix_i)
			{
				//--- accumulate color space features ---
				red += smesh_out->get_face_rgb_x[fd][pix_i];
				green += smesh_out->get_face_rgb_y[fd][pix_i];
				blue += smesh_out->get_face_rgb_z[fd][pix_i];
				hue += smesh_out->get_face_hsv_x[fd][pix_i];
				sat += smesh_out->get_face_hsv_y[fd][pix_i];
				val += smesh_out->get_face_hsv_z[fd][pix_i];

				//--- accumulate color index features ---
				std::vector<float> temp = { smesh_out->get_face_rgb_z[fd][pix_i], smesh_out->get_face_rgb_y[fd][pix_i], smesh_out->get_face_rgb_x[fd][pix_i] };//BGR
				greenness += temp[1] - 0.39*temp[2] - 0.61*temp[0];

				//---HSV color texture statics histogram build ---
				std::vector<float> hsv = { smesh_out->get_face_hsv_x[fd][pix_i], smesh_out->get_face_hsv_y[fd][pix_i], smesh_out->get_face_hsv_z[fd][pix_i] };
				for (int hsv_i = 0; hsv_i < hsv_bins.size(); ++hsv_i)
				{
					int bin_index = 0;
					for (int invertal_i = 0; invertal_i < hsv_bins[hsv_i] * interval_vec[hsv_i]; invertal_i += interval_vec[hsv_i])
					{
						if ((hsv[hsv_i] > invertal_i && hsv[hsv_i] <= invertal_i + interval_vec[hsv_i]) //normal case
							|| (hsv[hsv_i] == 0 && bin_index == 0))//when bin is zero include it 
						{
							++hsv_bins_count_vec[hsv_i][bin_index];
							break;
						}
						++bin_index;
					}
				}
				++count_pixels;
			}
		}

		//--- average color space features ---
		red /= float(count_pixels);
		green /= float(count_pixels);
		blue /= float(count_pixels);
		hue /= float(count_pixels);
		sat /= float(count_pixels);
		val /= float(count_pixels);

		//--- average color index features ---
		greenness /= float(count_pixels);

		//--- compute variance ---
		for (auto fd : spf_current.face_vec)
		{
			//parsing back to face pixels
			for (int pix_i = 0; pix_i < smesh_out->get_face_rgb_x[fd].size(); ++pix_i)
			{
				//--- accumulate color space variance ---
				red_var += std::pow(smesh_out->get_face_rgb_x[fd][pix_i] - red, 2);
				green_var += std::pow(smesh_out->get_face_rgb_y[fd][pix_i] - green, 2);
				blue_var += std::pow(smesh_out->get_face_rgb_z[fd][pix_i] - blue, 2);
				hue_var += std::pow(smesh_out->get_face_hsv_x[fd][pix_i] - hue, 2);
				sat_var += std::pow(smesh_out->get_face_hsv_y[fd][pix_i] - sat, 2);
				val_var += std::pow(smesh_out->get_face_hsv_z[fd][pix_i] - val, 2);

				//--- accumulate color index variance ---
				std::vector<float> temp = { smesh_out->get_face_rgb_z[fd][pix_i], smesh_out->get_face_rgb_y[fd][pix_i], smesh_out->get_face_rgb_x[fd][pix_i] };//BGR
				greenness_var += std::pow(temp[1] - 0.39*temp[2] - 0.61*temp[0] - greenness, 2);
			}
		}

		//--- average color space variance ---
		red_var /= float(count_pixels);
		green_var /= float(count_pixels);
		blue_var /= float(count_pixels);
		hue_var /= float(count_pixels);
		sat_var /= float(count_pixels);
		val_var /= float(count_pixels);

		//--- average color index variance ---
		greenness_var /= float(count_pixels);

		//---  fill the radiometric feature vector ---
		tex_feas[0] = red;
		tex_feas[1] = green;
		tex_feas[2] = blue;
		tex_feas[3] = hue;
		tex_feas[4] = sat;
		tex_feas[5] = val;
		tex_feas[6] = greenness;
		tex_feas[7] = red_var;
		tex_feas[8] = green_var;
		tex_feas[9] = blue_var;
		tex_feas[10] = hue_var;
		tex_feas[11] = sat_var;
		tex_feas[12] = val_var;
		tex_feas[13] = greenness_var;

		int tex_feas_i = 14;// up to 38
		for (int hsv_i = 0; hsv_i < hsv_bins.size(); ++hsv_i)
		{
			for (int bin_i = 0; bin_i < hsv_bins[hsv_i]; ++bin_i)
			{
				tex_feas[tex_feas_i++] = hsv_bins_count_vec[hsv_i][bin_i];
			}
		}

		for (int texf_i = 0; texf_i < tex_feas.size(); ++texf_i)
			value_validation_check(tex_feas[texf_i]);
	}

	//only allow one scale 
	void compute_features_on_vertices_face_centers_pcl
	(
		SFMesh* smesh_out,
		PTCloud* point_cloud,
		superfacets &spf_current,
		const int seg_i,
		std::vector<float> &geo_feas,
		std::vector<float> &tex_feas,
		std::vector< std::vector<float>> &basic_feas
	)
	{
		//compute geometric features on vertices and face centers within a segment
		compute_geometric_features_only_per_scale(smesh_out, point_cloud, spf_current, seg_i, geo_feas, basic_feas);

		//compute radiometric features on texture pixels of each facet within a segment
		compute_radiometric_features_on_face_textures(smesh_out, spf_current, seg_i, tex_feas);
	}

	//Segment based features(verticality, shape descriptor, area, mat radius)
	void compute_segment_basic_features
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &segment_out,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float> > &basic_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas,
		PTCloud *cloud_sampled,  //for MAT 
		PTCloud *cloud_ele      //for local elevation
	)
	{
		std::cout << "	- Compute basic mesh features: ";
		const double t_total = omp_get_wtime();

		//--- compute medial axis transform features ---
		medial_ball_features(smesh_out, cloud_sampled);

		//--- compute relative elevation of point cloud ---
		local_elevation_for_pointcloud(smesh_out, cloud_ele, segment_out);

		//--- compute features ---
		for (int i = 0; i < segment_out.size(); ++i)
		{
			std::vector<int> majority_labels(labels_name.size(), 0);
			std::map<int, bool> vert_visit, seg_longrange_check, seg_shortrange_check;
			//face features extract
			float temp_div_interior = 0.0f, temp_div_exterior = 0.0f;
			float interior_mat_radius = 0.0f, relative_elevation = 0.0f;

			vec3 avg_center;
			std::vector<std::pair<float, float>> multi_scales__seg_minmax_ele(multi_scale_ele_radius.size(),
				std::pair<float, float>(FLT_MAX, -FLT_MAX));
			std::pair<int, float> short_range_local_segid(-1, FLT_MAX);
			for (int j = 0; j < segment_out[i].face_vec.size(); ++j)
			{
				avg_center += smesh_out->get_face_center[segment_out[i].face_vec[j]] * smesh_out->get_face_area[segment_out[i].face_vec[j]];

				if (train_test_predict_val != 2)
				{
					if (smesh_out->get_face_truth_label[segment_out[i].face_vec[j]] != 0 && smesh_out->get_face_truth_label[segment_out[i].face_vec[j]] != -1)
						++majority_labels[smesh_out->get_face_truth_label[segment_out[i].face_vec[j]] - 1];
				}
				else
				{
					if (smesh_out->get_face_predict_label[segment_out[i].face_vec[j]] != 0 && smesh_out->get_face_predict_label[segment_out[i].face_vec[j]] != -1)
						++majority_labels[smesh_out->get_face_predict_label[segment_out[i].face_vec[j]] - 1];
				}

				//MAT features
				for (int mati = 0; mati < smesh_out->get_face_interior_medialball_radius[segment_out[i].face_vec[j]].size(); ++mati)
				{
					interior_mat_radius += smesh_out->get_face_interior_medialball_radius[segment_out[i].face_vec[j]][mati];
					++temp_div_interior;
				}

				//get segment vertices
				for (auto vtx : smesh_out->vertices(segment_out[i].face_vec[j]))
				{
					auto it_v = vert_visit.find(vtx.idx());
					if (it_v == vert_visit.end())
					{
						segment_out[i].ExactVec.push_back(vtx.idx());
						vert_visit[vtx.idx()] = true;
					}
				}

				//local long range lowest largest ground segment id
				for (int psi = 0; psi < local_ground_segs; ++psi)
				{
					auto psi_zmin_pair = smesh_out->get_face_segid_local_longrange_elevation_vec[segment_out[i].face_vec[j]][psi];
					auto it_s = seg_longrange_check.find(psi_zmin_pair.first);
					if (it_s == seg_longrange_check.end())
					{
						segment_out[i].segid_local_longrange_zmin_vec.emplace_back(psi_zmin_pair);
						seg_longrange_check[psi_zmin_pair.first] = true;
					}
				}

				//get multi scale local minimum elevations
				for (int sc_i = 0; sc_i < multi_scale_ele_radius.size(); ++sc_i)
				{
					if (smesh_out->get_face_mulsc_2_minmax_elevation[segment_out[i].face_vec[j]][sc_i].first < multi_scales__seg_minmax_ele[sc_i].first)
					{
						multi_scales__seg_minmax_ele[sc_i].first = smesh_out->get_face_mulsc_2_minmax_elevation[segment_out[i].face_vec[j]][sc_i].first;
					}
					if (smesh_out->get_face_mulsc_2_minmax_elevation[segment_out[i].face_vec[j]][sc_i].second > multi_scales__seg_minmax_ele[sc_i].second)
					{
						multi_scales__seg_minmax_ele[sc_i].second = smesh_out->get_face_mulsc_2_minmax_elevation[segment_out[i].face_vec[j]][sc_i].second;
					}
				}
			}
			//segment_out[i].short_range_local_ground_id = short_range_local_segid.first;

			//get local long range ground elevation
			std::tuple<int, float, float> segid_longrange_maxarea_ele(-1, -FLT_MAX, -1.0f);//id, area, elevation
			sort(segment_out[i].segid_local_longrange_zmin_vec.begin(), segment_out[i].segid_local_longrange_zmin_vec.end(), lower_local_elevation);
			int temp_size = local_ground_segs > segment_out[i].segid_local_longrange_zmin_vec.size() ?
				segment_out[i].segid_local_longrange_zmin_vec.size() : local_ground_segs;
			for (int psi = 0; psi < temp_size; ++psi)
			{
				int candidate_segid = segment_out[i].segid_local_longrange_zmin_vec[psi].first;
				int candidate_segind = superfacet_id_index_map[candidate_segid];
				if (segment_out[i].segid_local_longrange_zmin_vec[psi].first != -1
					&& get<1>(segid_longrange_maxarea_ele) < segment_out[candidate_segind].sum_area)
				{
					get<0>(segid_longrange_maxarea_ele) = candidate_segid;
					get<1>(segid_longrange_maxarea_ele) = segment_out[candidate_segind].sum_area;
					get<2>(segid_longrange_maxarea_ele) = segment_out[i].segid_local_longrange_zmin_vec[psi].second;
				}
			}

			int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			int new_label = maxElementIndex;
			if (majority_labels[maxElementIndex] == 0)
				new_label = -1;

			if (train_test_predict_val != 2)
				segment_out[i].ground_truth = new_label;
			else
				segment_out[i].predict = new_label;

			interior_mat_radius /= temp_div_interior;

			avg_center /= segment_out[i].sum_area;
			segment_out[i].avg_center = avg_center;

			segment_out[i].local_ground_longrange_segid = get<0>(segid_longrange_maxarea_ele);
			segment_out[i].local_longrange_ground_ele = get<2>(segid_longrange_maxarea_ele);

			relative_elevation = avg_center.z - segment_out[i].local_longrange_ground_ele;
			relative_elevation = relative_elevation > relative_elevation_cut_off_max ? relative_elevation_cut_off_max : relative_elevation;

			float triangle_density = float(segment_out[i].face_vec.size()) / segment_out[i].sum_area;

			//multi-scale relative elevation based features
			multi_scales_elevations(multi_scales__seg_minmax_ele, mulsc_ele_feas[i], avg_center.z);

			//shape features computation
			float shape_descriptor = 0.0f, compactness = 0.0f, shape_index = 0.0f;
			segment_shape_based_features(smesh_out, cloud_sampled, segment_out, i, shape_descriptor, compactness, shape_index);

			//parsing ground truth base on majority label
			seg_truth[i] = segment_out[i].ground_truth;

			//parsing features
			basic_feas[i][0] = avg_center.z;
			basic_feas[i][1] = interior_mat_radius;
			basic_feas[i][2] = segment_out[i].sum_area > cutoff_spfarea_max ? cutoff_spfarea_max : segment_out[i].sum_area;
			basic_feas[i][3] = relative_elevation;
			basic_feas[i][4] = triangle_density > cutoff_spffacetdensity_max ? cutoff_spffacetdensity_max : triangle_density;
			basic_feas[i][5] = segment_out[i].ExactVec.size() > cutoff_spf_vertex_count ? cutoff_spf_vertex_count : segment_out[i].ExactVec.size();//vertex_count
			basic_feas[i][6] =  0.0f;//points_to_plane_dist_mean
			basic_feas[i][7] = compactness;//compactness
			basic_feas[i][8] = shape_index;//shape_index
			basic_feas[i][9] = shape_descriptor;//shape_descriptor
		}
		std::cout << "Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//compute multiscale features on segment
	void compute_segment_features
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &segment_out,
		PTCloud *cloud_pt_3d_sampled,
		std::vector<std::vector<float>> &seg_plane_params,
		std::vector< std::vector<float> > &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas
	)
	{
		std::cout << "	- Compute mesh segment features: ";
		const double t_total = omp_get_wtime();
		//get 2d point cloud and build kdtree
		auto point_3d_coord_sampled = cloud_pt_3d_sampled->get_vertex_property<vec3>("v:point");

		easy3d::KdTree *tree_3d_sampled = new easy3d::KdTree;
		Build_kdtree(cloud_pt_3d_sampled, tree_3d_sampled);

#pragma omp parallel for schedule(dynamic)
		for (int seg_i = 0; seg_i < segment_out.size(); ++seg_i)
		{
			compute_features_on_vertices_face_centers_pcl
			(
				smesh_out,
				cloud_pt_3d_sampled,
				segment_out[seg_i],
				seg_i,
				eigen_feas[seg_i], color_feas[seg_i], basic_feas
			);

			seg_plane_params[seg_i][0] = segment_out[seg_i].plane_parameter.x;
			seg_plane_params[seg_i][1] = segment_out[seg_i].plane_parameter.y;
			seg_plane_params[seg_i][2] = segment_out[seg_i].plane_parameter.z;
			seg_plane_params[seg_i][3] = segment_out[seg_i].plane_parameter.w;
		}

		delete tree_3d_sampled;
		std::cout << "Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//normalize all features
	void normalization_all
	(
		std::vector< std::vector<float>> &basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas,
		std::vector< std::vector<float> > &mulsc_ele_feas
	)
	{
		std::cout << "	- Feature normalization: ";
		const double t_total = omp_get_wtime();
		feature_batch_normalization(basic_feas);
		feature_batch_normalization(eigen_feas);
		feature_batch_normalization(color_feas);
		feature_batch_normalization(mulsc_ele_feas);
		std::cout << "Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}


	//--- merge mesh and point cloud ---
	void tiles_merge_to_batch
	(
		SFMesh *smesh_tmp,
		PTCloud *cloud_sampled_tmp,
		PTCloud *cloud_ele_tmp,
		PTCloud *face_center_cloud_tmp,
		SFMesh *smesh_all,
		PTCloud *cloud_sampled_all,
		PTCloud *cloud_ele_all,
		PTCloud *face_center_cloud_all,
		std::map<int, int> &ptidx_faceid_map_all,
		int &vert_ind,
		int &pre_face_size,
		int &pre_sampled_cloud_size,
		int &tex_size,
		SFMesh *smesh_overseg
	)
	{
		merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size, smesh_overseg);
		merge_pointcloud(cloud_sampled_tmp, cloud_sampled_all, smesh_all, pre_face_size, 0);
		merge_pointcloud(cloud_ele_tmp, cloud_ele_all, smesh_all, pre_face_size, 1);
		merge_pointcloud(face_center_cloud_tmp, face_center_cloud_all, smesh_all, pre_face_size, 2, ptidx_faceid_map_all);

		pre_face_size += smesh_tmp->faces_size();
		pre_sampled_cloud_size += cloud_sampled_tmp->vertices_size();
		tex_size += smesh_tmp->texture_names.size();
	}

	//--- processing batch tiles ---
	void process_batch_tiles(std::vector<std::pair<int, std::string>> &batch_base_names, const int batch_index)
	{
		SFMesh* smesh_all = new SFMesh, *smesh_overseg = new SFMesh;
		PTCloud *cloud_sampled_all = new PTCloud,
			*cloud_ele_all = new PTCloud,
			*face_center_cloud_all = new PTCloud;
		std::map<int, int> ptidx_faceid_map_all;

		//--- configuration merge mesh and point cloud ---
		mesh_configuration(smesh_all);
		pointcloud_configuration(cloud_sampled_all);
		pointcloud_configuration(cloud_ele_all);
		pointcloud_configuration(face_center_cloud_all);

		if (use_existing_mesh_segments)
			read_mesh_data(smesh_overseg, batch_index);

		int ind = 0, vert_ind = 0, pre_face_size = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;
			PTCloud *cloud_sampled_tmp = new PTCloud,
				*cloud_ele_tmp = new PTCloud,
				*face_center_cloud_tmp = new PTCloud;
			std::vector<cv::Mat> texture_maps;
			std::map<int, int> ptidx_faceid_map_tmp;

			std::cout << "Finished " << ind << " tiles (total " << batch_base_names.size() << " tiles), current ";
			//--- read mesh *.ply data ---
			read_mesh_data(smesh_tmp, tile_i.first, texture_maps);

			//get facet texture if do not use sampling
			if (with_texture)
				face_texture_processor(smesh_tmp, texture_maps, tile_i.first);

			//--- sampling point cloud on mesh data ---
			if (is_pointclouds_exist)
			{
				read_pointcloud_data(smesh_tmp, cloud_sampled_tmp, 0, tile_i.first);
				read_pointcloud_data(smesh_tmp, cloud_ele_tmp, 1, tile_i.first);
			}
			else
			{
				sampling_point_cloud_on_mesh(smesh_tmp, texture_maps, cloud_sampled_tmp, cloud_ele_tmp, face_center_cloud_tmp, ptidx_faceid_map_tmp, tile_i.first);
			}

			//--- merge mesh and point cloud ---
			if (use_existing_mesh_segments)
				tiles_merge_to_batch(smesh_tmp, cloud_sampled_tmp, cloud_ele_tmp, face_center_cloud_tmp, smesh_all, cloud_sampled_all, cloud_ele_all, face_center_cloud_all, ptidx_faceid_map_all, vert_ind, pre_face_size, pre_sampled_cloud_size, tex_size, smesh_overseg);
			else
				tiles_merge_to_batch(smesh_tmp, cloud_sampled_tmp, cloud_ele_tmp, face_center_cloud_tmp, smesh_all, cloud_sampled_all, cloud_ele_all, face_center_cloud_all, ptidx_faceid_map_all, vert_ind, pre_face_size, pre_sampled_cloud_size, tex_size);

			delete smesh_tmp;
			delete cloud_sampled_tmp;
			delete cloud_ele_tmp;
			delete face_center_cloud_tmp;
			++ind;
		}

		//--- segment feature computation pipeline ---
		PTCloud* fea_cloud = new PTCloud;
		get_segment_features(smesh_all, cloud_sampled_all, cloud_ele_all, face_center_cloud_all, fea_cloud, ptidx_faceid_map_all, batch_index);

		//--- write feature point cloud ---
		std::string sfc_out = prefixs[9] + std::to_string(batch_index);
		write_feature_pointcloud_data(fea_cloud, sfc_out);

		//--- write mesh segments ---
		if (save_oversegmentation_mesh)
			write_mesh_segments(smesh_all, batch_index);

		delete smesh_all;
		delete smesh_overseg;
		delete cloud_sampled_all;
		delete cloud_ele_all;
		delete face_center_cloud_all;
		delete fea_cloud;
	}

	//--- processing single tile ---
	void process_single_tile(const int mi)
	{
		SFMesh *smesh_out = new SFMesh;
		PTCloud *cloud_sampled = new PTCloud, *cloud_ele = new PTCloud, *face_center_cloud = new PTCloud;
		std::vector<cv::Mat> texture_maps;
		std::map<int, int> ptidx_faceid_map;

		//--- read mesh *.ply data ---
		read_mesh_data(smesh_out, mi, texture_maps);

		//--- sampling point cloud on mesh data ---
		//--- read sampled point cloud *.ply data ---
		if (is_pointclouds_exist)
		{
			read_pointcloud_data(smesh_out, cloud_sampled, 0, mi);
			read_pointcloud_data(smesh_out, cloud_ele, 1, mi);
		}
		else
		{
			sampling_point_cloud_on_mesh(smesh_out, texture_maps, cloud_sampled, cloud_ele, face_center_cloud, ptidx_faceid_map, mi);
		}

		//--- get facet texture if do not use sampling
		if (with_texture)
			face_texture_processor(smesh_out, texture_maps, mi);

		//--- segment feature computation pipeline ---
		PTCloud* fea_cloud = new PTCloud;
		get_segment_features(smesh_out, cloud_sampled, cloud_ele, face_center_cloud, fea_cloud, ptidx_faceid_map, mi);

		//--- write feature point cloud ---
		std::string sfc_out = base_names[mi];
		write_feature_pointcloud_data(fea_cloud, sfc_out);

		//--- write mesh segments ---
		if (save_oversegmentation_mesh)
			write_mesh_segments(smesh_out, mi);

		delete smesh_out;
		delete cloud_sampled;
		delete cloud_ele;
		delete face_center_cloud;
		delete fea_cloud;
	}

	//--- segment feature computation pipeline ---
	void get_segment_features
	(
		SFMesh *smesh_out,
		PTCloud *cloud_sampled,
		PTCloud *cloud_ele,
		PTCloud *face_center_cloud,
		PTCloud *fea_cloud,
		std::map<int, int> &ptidx_faceid_map,
		const int mi
	)
	{
		//--- construct segment by region growing from data or use a simple rg ---
		std::vector<superfacets> segment_tmp;

		std::map<int, int> faceid_segid_map;
		std::map<int, std::vector<int>> seg_neighbors;
		if (!use_existing_mesh_segments)
		{
			std::cout << "	- Region growing to generate mesh segments: " << std::endl;
			const double t_total = omp_get_wtime();
			if (!enable_MohaVer_region_growing)
			{
				if (use_pointcloud_region_growing)//point cloud region growing
					cgal_regiongrowing(cloud_sampled, smesh_out, segment_tmp, faceid_segid_map, seg_neighbors);
				else//mesh region growing
					cgal_regiongrowing(smesh_out, segment_tmp, faceid_segid_map, seg_neighbors);
			}
			else
			{
				std::vector<vertex_planarity> current_mesh_planarity;
				initial_vertex_planarity(smesh_out, cloud_sampled, current_mesh_planarity);
				perform_mohaverdi_oversegmentation(smesh_out, segment_tmp, current_mesh_planarity, faceid_segid_map, seg_neighbors);
			}

			std::cout << "		- Found " << segment_tmp.size() << " superfacets took " << omp_get_wtime() - t_total << " (s). " << std::endl;
		}

		if (use_merged_segments)
		{
			const double t_seg = omp_get_wtime();
			std::vector<superfacets> segment_merged;
			std::cout << "	- Merging mesh segments: " << std::endl;
			merge_segments
			(
				smesh_out,
				face_center_cloud,
				segment_tmp,
				segment_merged,
				faceid_segid_map,
				ptidx_faceid_map,
				seg_neighbors
			);

			std::cout << "		- Found " << segment_merged.size() << ", and merged " << segment_tmp.size() - segment_merged.size() << " superfacets took " << omp_get_wtime() - t_seg << " (s). " << std::endl;
		}

		//write_feature_mesh_data(smesh_out, "region_growing_test.ply");exit(0);
		std::vector<superfacets> segment_out;
		std::vector<std::vector<int>> seg_face_vec;
		std::vector<int> seg_ids;
		construct_superfacets(smesh_out, cloud_sampled, segment_out, seg_face_vec, seg_ids);

		//--- assign segment color ---
		if (save_oversegmentation_mesh && !use_existing_mesh_segments)
			get_segments_color(smesh_out, segment_out);

		//--- compute segment based features (include point-based multi-scale relative elevation) ---
		std::vector<int> seg_truth(std::vector<int>(segment_out.size(), -1));
		std::vector< std::vector< float > > basic_feas(segment_out.size(), std::vector< float >(basic_feature_base_names.size(), default_feature_value_minmax.first));
		std::vector< std::vector<float> > mulsc_ele_feas(segment_out.size(), std::vector<float>(multi_scale_ele_radius.size(), default_feature_value_minmax.first));
		compute_segment_basic_features(smesh_out, segment_out, seg_truth, basic_feas, mulsc_ele_feas, cloud_sampled, cloud_ele);

		//--- compute segment buffer area based multi-scale features ---
		std::vector<std::vector<float>> seg_plane_params(segment_out.size(), std::vector<float>(4, 0.0f));
		//i,k = segments, features(gradient)
		std::vector< std::vector< float > > eigen_feas(segment_out.size(), std::vector< float >(eigen_feature_base_names.size(), default_feature_value_minmax.first)), color_feas(segment_out.size(), std::vector< float >(color_feature_base_names.size(), default_feature_value_minmax.first));
		compute_segment_features(smesh_out, segment_out, cloud_sampled, seg_plane_params, basic_feas, eigen_feas, color_feas);

		//--- normalize all features: no-scale, multi-scales, prior, gradients ---
		normalization_all(basic_feas, eigen_feas, color_feas, mulsc_ele_feas);

		//--- write features as a point cloud into binary .ply files ---
		construct_feature_pointclouds(seg_face_vec, seg_ids, seg_truth, basic_feas, eigen_feas, color_feas, mulsc_ele_feas, fea_cloud);
	}

	//--- processing visualization batch tiles ---
	void visualization_process_batch_tiles(std::vector<std::pair<int, std::string>> &batch_base_names, const int batch_index)
	{
		std::string sfc_in = prefixs[9] + std::to_string(batch_index);
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector<std::vector<int>> seg_face_vec;
		std::vector< std::vector<float>> basic_feas;
		std::vector< std::vector<float> > mulsc_ele_feas;
		std::vector< std::vector<float> > eigen_feas, color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, eigen_feas, color_feas, mulsc_ele_feas
		);

		SFMesh* smesh_all = new SFMesh, *smesh_overseg = new SFMesh;;
		mesh_configuration(smesh_all);

		if (use_existing_mesh_segments)
			read_mesh_data(smesh_overseg, batch_index);

		int ind = 0, vert_ind = 0, pre_sampled_cloud_size = 0, tex_size = 0;
		for (auto tile_i : batch_base_names)
		{
			SFMesh *smesh_tmp = new SFMesh;

			//--- read mesh *.ply data ---
			read_mesh_data(smesh_tmp, tile_i.first);

			//--- merge mesh ---
			if (use_existing_mesh_segments)
				merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size, smesh_overseg);
			else
				merge_mesh(smesh_tmp, smesh_all, vert_ind, pre_sampled_cloud_size, tex_size);
			tex_size += smesh_tmp->texture_names.size();
			delete smesh_tmp;
			++ind;
		}


		SFMesh *smesh_out = new SFMesh;
		visualization_feature_on_mesh
		(
			smesh_all, seg_face_vec, seg_truth,
			basic_feas, eigen_feas, color_feas,
			mulsc_ele_feas,
			smesh_out
		);

		write_feature_mesh_data(smesh_out, batch_index);
		delete smesh_out;

		delete smesh_overseg;
		delete smesh_all;
		delete pcl_out;
	}

	//--- processing visualization single tiles ---
	void visualization_process_single_tiles(const int pi)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_out = read_feature_pointcloud_data(sfc_in);
		if (pcl_out == nullptr)
		{
			std::cerr << "File loading failed" << std::endl;
			throw std::exception();
		}
		std::vector<int> seg_truth, seg_ids;
		std::vector<std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > eigen_feas, color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_out, seg_face_vec, seg_ids, seg_truth,
			basic_feas, eigen_feas, color_feas, mulsc_ele_feas
		);

		SFMesh *smesh_in = new SFMesh;
		mesh_configuration(smesh_in);

		read_mesh_data(smesh_in, pi);
		SFMesh *smesh_out = new SFMesh;
		visualization_feature_on_mesh
		(
			smesh_in, seg_face_vec, seg_truth,
			basic_feas, eigen_feas, color_feas, mulsc_ele_feas,
			smesh_out
		);

		//std::string sfc_out = get_feature_name_prifix(selection_vis_fn[si]).first + base_names[pi];
		write_feature_mesh_data(smesh_out, pi);
		delete smesh_out;

		delete pcl_out;
		delete smesh_in;
	}

	//--- PSSNet over-segmentation ---
	//--- processing pnp mrf single tiles ---
	void PNP_MRF_single_tiles
	(
		const int pi
	)
	{
		SFMesh *tmp_mesh = new SFMesh;
		mesh_configuration(tmp_mesh);
		read_labeled_mesh_data(tmp_mesh, pi);

		PTCloud *cloud_in = new PTCloud;
		pointcloud_configuration(cloud_in);
		read_pointcloud_data(tmp_mesh, cloud_in, 0, pi);

		std::vector<superfacets> spf_current;
		PTCloud* face_center_cloud = new PTCloud;

		std::vector<vertex_planarity> current_mesh_planarity;
		initial_vertex_planarity(tmp_mesh, cloud_in, current_mesh_planarity);
		perform_pnpmrf_oversegmentation_on_mesh(tmp_mesh, spf_current, current_mesh_planarity);

		get_segments_color(tmp_mesh, spf_current);

		write_mesh_segments(tmp_mesh, pi, true);

		delete tmp_mesh;
		delete cloud_in;
	}

	//--- PSSNet for GCN input ---
	//--- feature selection for output ---
	void feature_selection_for_GCN
	(
		SFMesh *smesh_in,
		PTCloud *cloud_out,
		std::vector<std::vector<int>> &seg_face_vec,
		std::vector<int> &seg_ids,
		std::vector<int> &seg_truth,
		std::vector< std::vector<float>> & basic_feas,
		std::vector< std::vector<float> > &eigen_feas,
		std::vector< std::vector<float> > &color_feas
	)
	{
		std::cout << "  Attaching segment features to points, use ";
		const double t_total = omp_get_wtime();
		std::vector<std::vector<int>> seg_pcl_vec(seg_face_vec.size(), std::vector<int>());
		std::map<int, bool> seg_visited;
		for (int seg_i = 0; seg_i < seg_face_vec.size(); ++seg_i)
		{
			seg_visited[seg_i] = false;
			for (int fi = 0; fi < seg_face_vec[seg_i].size(); ++fi)
			{
				SFMesh::Face fdx(seg_face_vec[seg_i][fi]);
				seg_pcl_vec[seg_i].insert(seg_pcl_vec[seg_i].end(),
						smesh_in->get_face_sampled_points[fdx].begin(), smesh_in->get_face_sampled_points[fdx].end());
			}
		}

		//cloud_out->remove_all_properties();
		cloud_out->remove_vertex_property(cloud_out->get_points_on_mesh_border);
		if (sampling_strategy == 1 || sampling_strategy == 2)
			get_segment_border_points(cloud_out, seg_truth, seg_pcl_vec);

		cloud_out->add_selected_feature_properties_for_GCN
		(
			seg_face_vec,
			seg_ids,
			seg_truth,
			basic_feas,
			eigen_feas, 
			color_feas
		);
		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//--- processing feature selection for GCN single tiles ---
	void feature_selection_for_GCN_single_tiles(const int pi)
	{
		std::string sfc_in = base_names[pi];
		easy3d::PointCloud* pcl_in = read_feature_pointcloud_data(sfc_in);

		SFMesh *smesh_out = new SFMesh;
		std::vector<cv::Mat> texture_maps;
		read_mesh_data(smesh_out, pi, texture_maps);

		PTCloud *sampled_cloud = new PTCloud;
		read_pointcloud_data(smesh_out, sampled_cloud, 0, pi);

		//texture color matching for new sampled point cloud
		PTCloud* face_center_cloud = new PTCloud;
		face_center_cloud->get_points_color = face_center_cloud->add_vertex_property<vec3>("v:color", vec3());

		if (!smesh_out->get_face_property<vec3>("f:color"))
			smesh_out->add_face_property<vec3>("f:color", vec3());
		smesh_out->get_face_color = smesh_out->get_face_property<vec3>("f:color");
		for (auto fi : smesh_out->faces())
		{
			smesh_out->get_face_color[fi] = vec3();
			face_center_cloud->add_vertex(smesh_out->get_face_center[fi]);
		}

		if (with_texture)
			face_texture_processor(smesh_out, texture_maps, pi);

		easy3d::KdTree *face_center_tree = new easy3d::KdTree;
		Build_kdtree(face_center_cloud, face_center_tree);
		sampled_cloud->get_points_color = sampled_cloud->get_vertex_property<vec3>("v:color");

		std::cout << "  Parsing texture color to augmented sampled point cloud, use ";
		const double t_total = omp_get_wtime();
		sampled_cloud->add_vertex_property<int>("v:point_segment_id", -1);
#pragma omp parallel for schedule(runtime)
		for (int pi = 0; pi < sampled_cloud->vertices_size(); ++pi)
		{
			PTCloud::Vertex ptx(pi);
			int closet_ind = face_center_tree->find_closest_point(sampled_cloud->get_vertex_property<vec3>("v:point")[ptx]);
			SFMesh::Face fd(closet_ind);
			sampled_cloud->get_points_color[ptx] = smesh_out->get_face_color[fd];
			sampled_cloud->get_vertex_property<int>("v:point_segment_id")[ptx] = smesh_out->get_face_segment_id[fd];
		}
		delete face_center_cloud;
		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;

		std::vector<int> seg_truth, seg_ids;
		std::vector<std::vector<int>> seg_face_vec;
		std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
		std::vector< std::vector<float> > eigen_feas, color_feas;
		get_all_feature_properties_from_feature_point_cloud
		(
			pcl_in, seg_face_vec, seg_ids, seg_truth,
			basic_feas, eigen_feas, color_feas, mulsc_ele_feas
		);

		seg_truth.clear();
		get_mesh_labels(smesh_out, seg_truth, seg_face_vec);

		feature_selection_for_GCN
		(
			smesh_out,
			sampled_cloud,
			seg_face_vec,
			seg_ids,
			seg_truth,
			basic_feas, eigen_feas, color_feas
		);

		//--- write GCN feature point cloud ---
		write_feature_pointcloud_data_for_GCN(sampled_cloud, pi);

		delete pcl_in;
		delete sampled_cloud;
		delete smesh_out;
	}
}