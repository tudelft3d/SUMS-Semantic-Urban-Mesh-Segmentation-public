/*
*   Name        : CGAL_Define.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : definition for CGAL types and classes
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

#pragma once
#ifndef semantic_mesh_segmentation__Define_HPP
#define semantic_mesh_segmentation__Define_HPP


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/property_map.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/Diagonalize_traits.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

#include <boost/property_map/property_map.hpp>
#include <boost/filesystem.hpp>

namespace semantic_mesh_segmentation
{
	//define CGAL classification types
#ifdef CGAL_LINKED_WITH_TBB
	typedef CGAL::Parallel_tag   Concurrency_tag;
#else
	typedef CGAL::Sequential_tag Concurrency_tag;
#endif

	typedef CGAL::Simple_cartesian<double>					  Kernel;
	typedef Kernel::FT                                        FT;
	typedef Kernel::Point_3                                   Point_3;
	typedef CGAL::Point_set_3<Point_3>						  Point_set;
	typedef Kernel::Line_2									  Line_2;
	typedef Kernel::Line_3									  Line;
	typedef Kernel::Plane_3									  Plane;
	typedef Kernel::Point_2									  Point_2;
	typedef Kernel::Vector_3								  Vector_3;
	typedef Point_set::Point_map							  Pmap;

	//Classification package
	typedef CGAL::Classification::Label_handle				  Label_handle;
	typedef CGAL::Classification::Feature_handle              Feature_handle;
	typedef CGAL::Classification::Label_set                   Label_set;
	typedef CGAL::Classification::Feature_set                 Feature_set;
	typedef CGAL::Classification::Cluster<Point_set, Pmap>    Cluster_point;
	typedef CGAL::internal::Alpha_expansion_graph_cut_boykov_kolmogorov CGAL_alpha_expansion;

	//define CGAL basic classes for PCA
	typedef Kernel::Triangle_3                                Triangle;
	typedef CGAL::Surface_mesh<Point_3>                       Mesh;
	typedef CGAL::Identity_property_map<Mesh>                 Fmap;
	typedef CGAL::Diagonalize_traits<double, 3>               DiagonalizeTraits;

	//For CGAL region growing: mesh
	typedef Mesh::Face_range								                                                                          Face_range;
	typedef CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Mesh>                                                        Neighbor_query_mesh;
	typedef CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Mesh>                                         region_type_mesh;	      
	typedef CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Mesh, Neighbor_query_mesh, Face_range>       sorting_mesh;
	typedef std::vector<std::size_t>                                                                                                  region_for;
	typedef std::vector<region_for>                                                                                                   regions_for;
	typedef region_type_mesh::Vertex_to_point_map                                                                                     Vertex_to_point_map_mesh;
	typedef CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query_mesh, region_type_mesh, typename sorting_mesh::Seed_map> Region_growing_mesh;

	//For CGAL region growing: pcl
	typedef CGAL::Point_set_3<Point_3>                                                                                                 Point_range;
	typedef Point_range::Point_map                                                                                                     Point_3_map;
	typedef Point_range::Vector_map                                                                                                    Normal_3_map;
	typedef CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Point_range, Point_3_map>                                       Neighbor_query_pcl;
	typedef CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Point_range, Point_3_map, Normal_3_map>           region_type_pcl;
	typedef CGAL::Shape_detection::Region_growing<Point_range, Neighbor_query_pcl, region_type_pcl>                                    Region_growing_pcl;

	//define CGAL feature types
	class segment_features : public CGAL::Classification::Feature_base
	{
		const std::vector< std::vector<float>> &basic_feas;
		const int fea_i;
	public:
		segment_features
		(
			const std::vector< std::vector<float>> &basic_feas_,
			const int fea_i_
		) :
			basic_feas(basic_feas_),
			fea_i(fea_i_)
		{}

		float value(std::size_t segment_index)
		{
			return basic_feas[segment_index][fea_i];
		}

		const std::type_info& type_id() const
		{
			return typeid(segment_features);
		}
	};
}

#endif