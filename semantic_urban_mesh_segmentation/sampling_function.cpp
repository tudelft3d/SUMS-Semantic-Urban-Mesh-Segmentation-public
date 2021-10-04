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

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	void sampling_pointcloud_on_mesh
	(
		easy3d::PointCloud* possion_cloud,
		SFMesh *smesh_out,
		const float point_density
	)
	{
		sampling_points_number = int(mesh_all_area * point_density);
		sampling_points_number = sampling_points_number < 1 ? 1 : sampling_points_number;

		easy3d::PointCloud* cloud_temp = new easy3d::PointCloud;
		montecarlo_sampling(smesh_out, cloud_temp);

		float radius_temp = ComputePoissonDiskRadius(mesh_all_area, sampling_points_number);
		PoissonDiskPruning(possion_cloud, cloud_temp, radius_temp);
		delete cloud_temp;
	}
}