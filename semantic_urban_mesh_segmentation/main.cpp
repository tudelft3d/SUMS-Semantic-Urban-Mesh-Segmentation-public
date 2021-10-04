/*
*   Name        : main.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : main entrance
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

#include "operation_function.hpp" 
using namespace semantic_mesh_segmentation;

int main(int argc, char** argv)
{
	std::string path;
	// check if data paths were defined
	if (argc != 2)
	{
		std::cerr << "Usage: SSTM <path to config.txt>" << std::endl;
		std::cerr << "Press enter to continue..." << std::endl;
		std::cin.get();
		return -1;
	}
	else
	{
		path = argv[1];
	}

	read_txt_config(path);
	run(current_mode);
	std::cerr << "Press enter to continue..." << std::endl;
	std::cin.get();
	return EXIT_SUCCESS;
}