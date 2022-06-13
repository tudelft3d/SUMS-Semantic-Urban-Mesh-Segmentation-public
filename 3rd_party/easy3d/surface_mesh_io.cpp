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


#include <easy3d/surface_mesh_io.h>

#include <clocale>

#include <easy3d/surface_mesh.h>
#include <easy3d/file.h>


namespace easy3d {


	SurfaceMesh* MeshIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

		std::string ext = file::extension(file_name, true);

		SurfaceMesh* mesh = new SurfaceMesh;

		bool success = false;
		if (ext == "ply")
			success = io::load_ply(file_name, mesh);
		else if (ext == "obj")
			success = io::load_obj(file_name, mesh);
		else if (ext == "off")
			success = io::load_off(file_name, mesh);
		else {
			std::cerr << "unknown file format: " << ext << std::endl;
			success = false;
		}

		if (!success || mesh->n_faces() == 0) {
			std::cerr << "reading file failed (no data exist)" << std::endl;
			delete mesh;
			return nullptr;
		}


#ifndef NDEBUG
		std::cout << "vertex properties on mesh " << file::base_name(file_name) << std::endl;
		const auto& vnames = mesh->vertex_properties();
		for (const auto& n : vnames)
			std::cout << "\t" << n << std::endl;

		std::cout << "face properties on mesh " << file::base_name(file_name) << std::endl;
		const auto& fnames = mesh->face_properties();
		for (const auto& n : fnames)
			std::cout << "\t" << n << std::endl;

		std::cout << "edge properties on mesh " << file::base_name(file_name) << std::endl;
		const auto& enames = mesh->edge_properties();
		for (const auto& n : enames)
			std::cout << "\t" << n << std::endl;
#endif

		return mesh;
	}

	//------------------------------------------- Weixiao Update --------------------------------------------//
	bool MeshIO::save(const std::string& file_name, const SurfaceMesh* mesh, const std::vector<std::string> &comment)
	{
		if (!mesh) {
			std::cerr << "surface mesh is null" << std::endl;
			return false;
		}

		std::string ext = file::extension(file_name, true);

		bool success = false;
		if (ext == "ply")
			success = io::save_ply(file_name, mesh, comment, false);
		else
		{
			std::cerr << "cannot save this file in ply format: " << ext << std::endl;
			success = false;
		}

		return success;
	}
	//-------------------------------------------------------------------------------------------------------//

	bool MeshIO::save(const std::string& file_name, const SurfaceMesh* mesh, const bool use_binary)
	{
		if (!mesh) {
			std::cerr << "surface mesh is null" << std::endl;
			return false;
		}

		std::string ext = file::extension(file_name, true);

		bool success = false;
		if (ext == "ply")
			success = io::save_ply(file_name, mesh, use_binary);
		else if (ext == "obj")
			success = io::save_obj(file_name, mesh);
		else if (ext == "off")
			success = io::save_off(file_name, mesh);
		else {
			std::cerr << "unknown file format: " << ext << std::endl;
			success = false;
		}

		return success;
	}


    //Weixiao Update*********************************
    bool MeshIO::load(const std::string& file_name, SurfaceMesh* mesh)
    {
        std::setlocale(LC_NUMERIC, "C");

        std::string ext = file::extension(file_name, true);

        bool success = false;

        if (ext == "ply")
            success = io::load_ply(file_name, mesh);
        else if (ext == "obj")
            success = io::load_obj(file_name, mesh);
        else if (ext == "off")
            success = io::load_off(file_name, mesh);
        else {
            std::cerr << "unknown file format: " << ext << std::endl;
            success = false;
        }

        if (!success || mesh->n_faces() == 0) {
            std::cerr << "reading file failed (no data exist)" << std::endl;
            delete mesh;
            return nullptr;
        }


#ifndef NDEBUG
        std::cout << "vertex properties on mesh " << file::base_name(file_name) << std::endl;
        const auto& vnames = mesh->vertex_properties();
        for (const auto& n : vnames)
            std::cout << "\t" << n << std::endl;

        std::cout << "face properties on mesh " << file::base_name(file_name) << std::endl;
        const auto& fnames = mesh->face_properties();
        for (const auto& n : fnames)
            std::cout << "\t" << n << std::endl;

        std::cout << "edge properties on mesh " << file::base_name(file_name) << std::endl;
        const auto& enames = mesh->edge_properties();
        for (const auto& n : enames)
            std::cout << "\t" << n << std::endl;
#endif

        return success;
    }



} // namespace easy3d
