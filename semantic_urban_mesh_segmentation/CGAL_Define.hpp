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
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Projection_traits_xz_3.h>
#include <CGAL/Projection_traits_yz_3.h>

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
	typedef Kernel::Line_3									  Line_3;
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

	//For CGAL delaunay triangulation
	typedef CGAL::Delaunay_triangulation_3<Kernel> DT3;
	typedef DT3::Finite_vertices_iterator Finite_vertices_iterator;


	//For CGAL Alpha shape
	typedef CGAL::Exact_predicates_inexact_constructions_kernel              Kernel_Exact;
	typedef Kernel_Exact::FT                                                 FT_Exact;
	typedef Kernel_Exact::Point_2                                            Point_2_Exact;
	typedef Kernel_Exact::Point_3                                            Point_3_Exact;
	typedef Kernel_Exact::Segment_2                                          Segment_2_Exact;
	typedef Kernel_Exact::Segment_2                                          Segment_Exact;

	//For alpha projection on xy plane
	typedef CGAL::Projection_traits_xy_3<Kernel_Exact>				         Gt_Exact_xy;
	typedef CGAL::Alpha_shape_vertex_base_2<Gt_Exact_xy>                     Vb_Exact_xy;
	typedef CGAL::Alpha_shape_face_base_2<Gt_Exact_xy>                       Fb_Exact_xy;
	typedef CGAL::Triangulation_data_structure_2<Vb_Exact_xy, Fb_Exact_xy>   Tds_Exact_xy;
	typedef CGAL::Delaunay_triangulation_2<Gt_Exact_xy, Tds_Exact_xy>        Triangulation_2_Exact_xy;
	typedef CGAL::Alpha_shape_2<Triangulation_2_Exact_xy>                    Alpha_shape_2_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Alpha_shape_vertices_iterator            Alpha_shape_vertices_iterator_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Alpha_shape_edges_iterator               Alpha_shape_edges_iterator_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Vertex_handle                            Vertex_handle_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Edge                                     Edge_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Face_handle                              Face_handle_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Vertex_circulator                        Vertex_circulator_Exact_xy;
	typedef Alpha_shape_2_Exact_xy::Edge_circulator                          Edge_circulator_Exact_xy;

	//For alpha projection on xz plane
	typedef CGAL::Projection_traits_xz_3<Kernel_Exact>				         Gt_Exact_xz;
	typedef CGAL::Alpha_shape_vertex_base_2<Gt_Exact_xz>                     Vb_Exact_xz;
	typedef CGAL::Alpha_shape_face_base_2<Gt_Exact_xz>                       Fb_Exact_xz;
	typedef CGAL::Triangulation_data_structure_2<Vb_Exact_xz, Fb_Exact_xz>   Tds_Exact_xz;
	typedef CGAL::Delaunay_triangulation_2<Gt_Exact_xz, Tds_Exact_xz>        Triangulation_2_Exact_xz;
	typedef CGAL::Alpha_shape_2<Triangulation_2_Exact_xz>                    Alpha_shape_2_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Alpha_shape_vertices_iterator            Alpha_shape_vertices_iterator_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Alpha_shape_edges_iterator               Alpha_shape_edges_iterator_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Vertex_handle                            Vertex_handle_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Edge                                     Edge_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Face_handle                              Face_handle_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Vertex_circulator                        Vertex_circulator_Exact_xz;
	typedef Alpha_shape_2_Exact_xz::Edge_circulator                          Edge_circulator_Exact_xz;

	//For alpha projection on yz plane
	typedef CGAL::Projection_traits_yz_3<Kernel_Exact>				         Gt_Exact_yz;
	typedef CGAL::Alpha_shape_vertex_base_2<Gt_Exact_yz>                     Vb_Exact_yz;
	typedef CGAL::Alpha_shape_face_base_2<Gt_Exact_yz>                       Fb_Exact_yz;
	typedef CGAL::Triangulation_data_structure_2<Vb_Exact_yz, Fb_Exact_yz>   Tds_Exact_yz;
	typedef CGAL::Delaunay_triangulation_2<Gt_Exact_yz, Tds_Exact_yz>        Triangulation_2_Exact_yz;
	typedef CGAL::Alpha_shape_2<Triangulation_2_Exact_yz>                    Alpha_shape_2_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Alpha_shape_vertices_iterator            Alpha_shape_vertices_iterator_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Alpha_shape_edges_iterator               Alpha_shape_edges_iterator_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Vertex_handle                            Vertex_handle_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Edge                                     Edge_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Face_handle                              Face_handle_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Vertex_circulator                        Vertex_circulator_Exact_yz;
	typedef Alpha_shape_2_Exact_yz::Edge_circulator                          Edge_circulator_Exact_yz;

	template<class Alpha_shape_2_Exact, class Face_handle_Exact, class Vertex_handle_Exact>
	class AlphaShapeRegionGrower
	{
		Alpha_shape_2_Exact &A;
		enum Mode
		{
			ALPHA, // stop at alpha boundary
			EXTERIOR // stop at faces labels as exterior
		};
		int label_cnt; // label==-1 means exterior, -2 mean never visiter, 0+ means a regular region

	public:
		std::unordered_map<Face_handle_Exact, int> face_map;
		std::unordered_map<int, Vertex_handle_Exact> region_map; //label: (boundary vertex)
		AlphaShapeRegionGrower(Alpha_shape_2_Exact& as) : A(as), label_cnt(0) {};

		void grow()
		{
			std::stack<Face_handle_Exact> seeds;
			for (auto fh = A.all_faces_begin(); fh != A.all_faces_end(); ++fh)
			{
				seeds.push(fh);
				face_map[fh] = -2;
			}
			auto inf_face = A.infinite_face();
			face_map[inf_face] = -1;
			grow_region(inf_face, ALPHA); // this sets label of exterior faces to -1
			while (!seeds.empty())
			{
				auto fh = seeds.top();
				seeds.pop();
				if (face_map[fh] == -2)
				{
					face_map[fh] = label_cnt;
					grow_region(fh, EXTERIOR);
					++label_cnt;
				}
			}
		}

		void grow_region(Face_handle_Exact face_handle, Mode mode)
		{
			std::stack<Face_handle_Exact> candidates;
			candidates.push(face_handle);

			while (candidates.size() > 0)
			{
				auto fh = candidates.top(); candidates.pop();
				// check the 3 neighbors of this face
				for (int i = 0; i < 3; ++i) {
					auto e = std::make_pair(fh, i);
					auto neighbor = fh->neighbor(i);

					if (mode == ALPHA)
					{
						// add neighbor if it is not on the other side of alpha boundary
						// check if this neighbor hasn't been visited before
						if (face_map[neighbor] == -2)
						{
							auto edge_class = A.classify(e);
							if (!(edge_class == Alpha_shape_2_Exact::REGULAR || edge_class == Alpha_shape_2_Exact::SINGULAR))
							{
								face_map[neighbor] = -1;
								candidates.push(neighbor);
							}
						}
					}
					else if (mode == EXTERIOR)
					{
						// check if this neighbor hasn't been visited before and is not exterior
						if (face_map[neighbor] == -2)
						{
							face_map[neighbor] = label_cnt;
							candidates.push(neighbor);
							// if it is exterior, we store this boundary edge
						}
						else if (face_map[neighbor] == -1)
						{
							if (region_map.find(label_cnt) == region_map.end())
							{
								region_map[label_cnt] = fh->vertex(A.cw(i));
							}
						}
					}

				}
			}
		}
	};

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