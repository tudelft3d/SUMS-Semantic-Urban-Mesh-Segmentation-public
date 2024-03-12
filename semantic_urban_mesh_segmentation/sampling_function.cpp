/*
*   Name        : sampling_function.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for point cloud sampling
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

#include "sampling_function.hpp"
#include <easy3d/point_cloud_io.h>
using namespace easy3d;
namespace semantic_mesh_segmentation
{
	void sampling_pointcloud_on_mesh
	(
		easy3d::PointCloud* possion_cloud,
		SFMesh *smesh_out,
		const float point_density,
		const float tolerance,
		const int bestSamplePoolSize,
		const int MontecarloRate
	)
	{
		sampling_points_number = int(smesh_out->mesh_area * point_density);
		sampling_points_number = sampling_points_number < 1 ? 1 : sampling_points_number;

		std::cout << "sampling_points_number = " << sampling_points_number << std::endl;
		float diskRadius = ComputePoissonDiskRadius(smesh_out->mesh_area, sampling_points_number);
		easy3d::PointCloud* montecarlo_cloud = new easy3d::PointCloud;
		montecarlo_sampling(smesh_out, montecarlo_cloud, sampling_points_number * MontecarloRate);
		PoissonDiskPruningByNumber(possion_cloud, montecarlo_cloud, smesh_out, sampling_points_number, diskRadius, 
			tolerance, bestSamplePoolSize, MontecarloRate);
		delete montecarlo_cloud;
	}

	void random_sampling_pointcloud_on_selected_faces
	(
		SFMesh* smesh_in,
		std::vector<SFMesh::Face>& label_component_faces_i,
		easy3d::PointCloud* sampled_cloud,
		const float component_area
	)
	{
		if (!smesh_in->get_face_property<std::vector<easy3d::vec3>>("f:sampled_points"))
			smesh_in->add_face_property<std::vector<easy3d::vec3>>("f:sampled_points", std::vector<easy3d::vec3>());
		auto fd_sampled_pts = smesh_in->get_face_property<std::vector<easy3d::vec3>>("f:sampled_points");
		sampled_cloud->add_vertex_property<int>("v:face_id", -1);
		auto vt_fid = sampled_cloud->get_vertex_property<int>("v:face_id");
		int num_needed = int(component_area * sampling_point_density);
		std::cout << "sampling_points_number = " << num_needed << std::endl;
		for (auto fd : label_component_faces_i)
		{
			// samples number considering the facet size (i.e., area)
			float samples_num = smesh_in->get_face_area[fd] * sampling_point_density;
			if (samples_num < 1)
				samples_num = 1;
			std::size_t quant_samples_num = (std::size_t)samples_num;

			// generate points
			for (unsigned int j = 0; j < quant_samples_num; j++) {
				// compute barycentric coords
				double s = sqrt((double)rand() / (double)RAND_MAX);
				double t = (double)rand() / (double)RAND_MAX;
				double c[3];

				c[0] = 1.0 - s;
				c[1] = s * (1.0 - t);
				c[2] = s * t;

				vec3 p;
				std::size_t i = 0;
				for (auto vd : smesh_in->vertices(fd))
				{
					p = p + c[i] * smesh_in->get_points_coord[vd];
					++i;
				}

				auto sampled_vd = sampled_cloud->add_vertex(p);//PointCloud::Vertex v = 
				vt_fid[sampled_vd] = fd.idx();
				fd_sampled_pts[fd].push_back(p);
			}

			//add face center
			auto sampled_vd = sampled_cloud->add_vertex(smesh_in->get_face_center[fd]);
			vt_fid[sampled_vd] = fd.idx();
			fd_sampled_pts[fd].push_back(smesh_in->get_face_center[fd]);
		}
	}

	void face_center_point_cloud
	(
		SFMesh* smesh,
		easy3d::PointCloud* fd_cen_pcl
	)
	{
		if (!fd_cen_pcl->get_vertex_property<easy3d::vec3>("v:normal"))
			fd_cen_pcl->add_vertex_property<easy3d::vec3>("v:normal", easy3d::vec3());
		auto get_pcl_normal = fd_cen_pcl->get_vertex_property<easy3d::vec3>("v:normal");
		if (!fd_cen_pcl->get_vertex_property<easy3d::vec3>("v:color"))
			fd_cen_pcl->add_vertex_property<easy3d::vec3>("v:color", easy3d::vec3());
		auto get_pcl_color = fd_cen_pcl->get_vertex_property<easy3d::vec3>("v:color");
		fd_cen_pcl->add_vertex_property<int>("v:label", -1);
		auto get_pcl_label = fd_cen_pcl->get_vertex_property<int>("v:label");

		for (auto fd : smesh->faces())
		{
			easy3d::vec3 fd_cen(0.0f, 0.0f, 0.0f);
			for (auto vd : smesh->vertices(fd))
				fd_cen += smesh->get_vertex_property<easy3d::vec3>("v:point")[vd];
			fd_cen /= 3.0f;
			auto cur_vd = fd_cen_pcl->add_vertex(fd_cen);
			get_pcl_normal[cur_vd] = smesh->get_face_normals[fd];
			get_pcl_color[cur_vd] = smesh->get_face_color[fd];
			get_pcl_label[cur_vd] = smesh->get_face_truth_label[fd];
		}
	}

	void assign_texpcl_properties
	(
		easy3d::PointCloud* sampled_cloud,
		easy3d::PointCloud* tex_cloud
	)
	{
		if (!sampled_cloud->get_vertex_property<easy3d::vec3>("v:normal"))
			sampled_cloud->add_vertex_property<easy3d::vec3>("v:normal", easy3d::vec3());
		if (!sampled_cloud->get_vertex_property<easy3d::vec3>("v:color"))
			sampled_cloud->add_vertex_property<easy3d::vec3>("v:color", easy3d::vec3());
		sampled_cloud->add_vertex_property<int>("v:label", -1);

		auto get_texpcl_normal = tex_cloud->get_vertex_property<easy3d::vec3>("v:normal");
		auto get_texpcl_color = tex_cloud->get_vertex_property<easy3d::vec3>("v:color");
		auto get_texpcl_label = tex_cloud->get_vertex_property<int>("v:label");

		auto get_pcl_coord = sampled_cloud->get_vertex_property<easy3d::vec3>("v:point");
		auto get_pcl_normal = sampled_cloud->get_vertex_property<easy3d::vec3>("v:normal");
		auto get_pcl_color = sampled_cloud->get_vertex_property<easy3d::vec3>("v:color");
		auto get_pcl_label = sampled_cloud->get_vertex_property<int>("v:label");

		easy3d::KdTree tex_tree;
		tex_tree.begin();
		tex_tree.add_point_cloud(tex_cloud);
		tex_tree.end();

		for (auto& vd : sampled_cloud->vertices())
		{
			int vnear_i = tex_tree.find_closest_point(get_pcl_coord[vd]);
			easy3d::PointCloud::Vertex vnear_d(vnear_i);
			get_pcl_color[vd] = get_texpcl_color[vnear_d];
			get_pcl_label[vd] = get_texpcl_label[vnear_d];
			get_pcl_normal[vd] = get_texpcl_normal[vnear_d];

			if (with_texture_mask && get_pcl_label[vd] > 0)
				get_pcl_label[vd] = get_pcl_label[vd] - 1;// remove terrian label
		}
	}

	void face_random_sampling
	(
		SFMesh* mesh,
		easy3d::SurfaceMesh::Face& fd,
		easy3d::PointCloud* sampled_cloud,
		const int samples_num
	)
	{
		auto get_points_coord = mesh->get_vertex_property<easy3d::vec3>("v:point");
		std::size_t quant_samples_num = (std::size_t)samples_num;
		// generate points
		for (unsigned int j = 0; j < quant_samples_num; j++)
		{
			// compute barycentric coords
			double s = sqrt((double)rand() / (double)RAND_MAX);
			double t = (double)rand() / (double)RAND_MAX;
			double c[3];

			c[0] = 1.0 - s;
			c[1] = s * (1.0 - t);
			c[2] = s * t;

			easy3d::vec3 p;

			std::size_t i = 0;
			for (auto vd : mesh->vertices(fd))
			{
				p = p + c[i] * get_points_coord[vd];
				++i;
			}

			sampled_cloud->add_vertex(p);
		}
	}

	void perform_uniform_sampling(easy3d::PointCloud* cloud, const int fix_sampling_pcl_num)
	{
		unsigned int expected_number = fix_sampling_pcl_num;
		std::vector<easy3d::PointCloud::Vertex> points_to_remove = easy3d::PointCloudSimplification::uniform_simplification(cloud, expected_number);
		auto old_num = cloud->n_vertices();
		for (auto v : points_to_remove)
			cloud->delete_vertex(v);
		cloud->garbage_collection();
		if (!points_to_remove.empty())
		{
			points_to_remove.clear();
			auto new_num = cloud->n_vertices();
			std::cout << old_num - new_num << " points removed. " << new_num << " points remain";
		}
	}

	void mesh_random_sampling
	(
		SFMesh* mesh,
		easy3d::PointCloud* sampled_cloud,
		const int mesh_sampling_points_number
	)
	{
		int expected_number = sampling_points_number;
		if (mesh_sampling_points_number > 0)
			expected_number = mesh_sampling_points_number;
		int new_sampling_point_number = std::ceil(float(expected_number) * (1.0f + 10.0f * 0.02));
		for (auto& fd : mesh->faces())
		{
			int fd_sampling_points_number = new_sampling_point_number * (mesh->get_face_area[fd] / mesh->mesh_area);
			fd_sampling_points_number = fd_sampling_points_number < 1 ? 1 : fd_sampling_points_number;
			face_random_sampling(mesh, fd, sampled_cloud, fd_sampling_points_number);
		}

		perform_uniform_sampling(sampled_cloud, mesh_sampling_points_number);
	}

	void poission_sampling_with_fixed_number
	(
		SFMesh* mesh,
		easy3d::PointCloud* possion_cloud,
		const int mesh_sampling_points_number,
		const float tolerance,
		const int bestSamplePoolSize,
		const int MontecarloRate
	)
	{
		int expected_number = sampling_points_number;
		if (mesh_sampling_points_number > 0)
			expected_number = mesh_sampling_points_number;
		int new_sampling_point_number = std::ceil(float(expected_number) * (1.0f + 10.0f * tolerance));

		float diskRadius = ComputePoissonDiskRadius(mesh->mesh_area, new_sampling_point_number);
		easy3d::PointCloud* montecarlo_cloud = new easy3d::PointCloud;
		montecarlo_sampling(mesh, montecarlo_cloud, new_sampling_point_number * MontecarloRate);
		PoissonDiskPruningByNumber(possion_cloud, montecarlo_cloud, mesh, new_sampling_point_number, diskRadius,
			tolerance, bestSamplePoolSize, MontecarloRate);
		delete montecarlo_cloud;

		perform_uniform_sampling(possion_cloud, mesh_sampling_points_number);
	}
}