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
		sampling_points_number = int(mesh_all_area * point_density);
		sampling_points_number = sampling_points_number < 1 ? 1 : sampling_points_number;

		std::cout << "sampling_points_number = " << sampling_points_number << std::endl;
		float diskRadius = ComputePoissonDiskRadius(mesh_all_area, sampling_points_number);
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
}