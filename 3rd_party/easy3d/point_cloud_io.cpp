/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of Easy3D: software for processing and rendering
*   meshes and point clouds.
*
*	Easy3D is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	Easy3D is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include <easy3d/point_cloud_io.h>

#include <clocale>
#include <fstream>

#include <easy3d/point_cloud.h>
#include <easy3d/file.h>

namespace easy3d {


	PointCloud* PointCloudIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

		std::string ext = file::extension(file_name, true);

		PointCloud* cloud = new PointCloud;

		bool success = false;
		if (ext == "ply")
			success = io::load_ply(file_name, cloud);
		else if (ext == "bin")
			success = io::load_bin(file_name, cloud);
		else if (ext == "xyz")
			success = io::load_xyz(file_name, cloud);
		else if (ext == "bxyz")
			success = io::load_bxyz(file_name, cloud);
		else {
			std::cerr << "unknown file format: " << ext << std::endl;
			success = false;
		}

		if (!success || cloud->n_vertices() == 0) {
			std::cerr << "reading file failed (no data exist)" << std::endl;
			delete cloud;
			return nullptr;
		}

#ifndef NDEBUG
		std::cout << "vertex properties on point cloud " << file::base_name(file_name) << std::endl;
		const auto& vnames = cloud->vertex_properties();
		for (const auto& n : vnames)
			std::cout << "\t" << n << std::endl;
#endif

		return cloud;
	}


	bool PointCloudIO::save(const std::string& file_name, const PointCloud* cloud, const bool use_binary) {
		if (!cloud) {
			std::cerr << "Point cloud is null" << std::endl;
			return false;
		}

		std::string ext = file::extension(file_name, true);
		bool success = false;
		if (ext == "ply")
			success = io::save_ply(file_name, cloud, use_binary);
		else if (ext == "bin")
			success = io::save_bin(file_name, cloud);
		else if (ext == "xyz")
			success = io::save_xyz(file_name, cloud);
		else if (ext == "bxyz")
			success = io::save_bxyz(file_name, cloud);
		else {
			std::cerr << "unknown file format: " << ext << std::endl;
			success = false;
		}

		return success;
	}

} // namespace easy3d
