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


#ifndef EASY3D_FILEIO_MESH_IO_H
#define EASY3D_FILEIO_MESH_IO_H


#include <string>
#include <vector>

namespace easy3d {

	class SurfaceMesh;

	class MeshIO
	{
	public:
		// return nullptr if failed.
		static SurfaceMesh* load(const std::string& file_name);

		// save the mesh to a file. return false if failed.
		static bool	save(const std::string& file_name, const SurfaceMesh* mesh, const bool);
	
		static bool	save(const std::string& file_name, const SurfaceMesh* mesh, const std::vector<std::string> &comment);
        //
        static bool load(const std::string& file_name, SurfaceMesh* mesh);
    };

	namespace io {

		bool load_ply(const std::string& file_name, SurfaceMesh* mesh);
		bool save_ply(const std::string& file_name, const SurfaceMesh* mesh, bool binary = true);
		bool save_ply(const std::string& file_name, const SurfaceMesh* mesh, const std::vector<std::string> &comment, bool binary = true);

		bool load_off(const std::string& file_name, SurfaceMesh* mesh);
		bool save_off(const std::string& file_name, const SurfaceMesh* mesh);

		bool load_obj(const std::string& file_name, SurfaceMesh* mesh);
        bool save_obj(const std::string& file_name, const SurfaceMesh* mesh);

	} // namespace io

} // namespace easy3d

#endif // EASY3D_FILEIO_MESH_IO_H
