/*
*   Name        : over_segmentation.cpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh over-segmentation
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

#include "over_segmentation.hpp"

using namespace easy3d;
namespace semantic_mesh_segmentation
{
	//finding segment adjacent neighbor
	void finding_segments_neighbor
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &segment_out //for all segment only
	)
	{
		//Note: Please clean previous temporary data first
		// loop over all edges
		for (auto ei : smesh_out->edges())
		{
			int id_1 = -1, id_2 = -1, index_1 = -1, index_2 = -1;
			SFMesh::Halfedge h0 = smesh_out->halfedge(ei, 0);
			if (!smesh_out->is_boundary(h0))
			{
				id_1 = smesh_out->get_face_segment_id[smesh_out->face(h0)];
				index_1 = superfacet_id_index_map[id_1];
			}

			SFMesh::Halfedge h1 = smesh_out->halfedge(ei, 1);
			if (!smesh_out->is_boundary(h1))
			{
				id_2 = smesh_out->get_face_segment_id[smesh_out->face(h1)];
				index_2 = superfacet_id_index_map[id_2];
			}

			if (id_1 != -1 && id_2 != -1)
			{
				if (id_1 != id_2)
				{
					bool neighbor_visited_flag = false;
					for (int si = 0; si < segment_out[index_1].neighbor_ids.size(); ++si)
					{
						if (id_2 == segment_out[index_1].neighbor_ids[si])
						{
							neighbor_visited_flag = true;
							break;
						}
					}
					for (int si2 = 0; si2 < segment_out[index_2].neighbor_ids.size(); ++si2)
					{
						if (id_1 == segment_out[index_2].neighbor_ids[si2])
						{
							neighbor_visited_flag = true;
							break;
						}
					}

					if (!neighbor_visited_flag)
					{
						segment_out[index_1].neighbor_ids.push_back(id_2);
						segment_out[index_2].neighbor_ids.push_back(id_1);
					}
				}
			}
		}
	}

	//Compute the set of connected components of a given mesh
	int separate_connected_components
	(
		SFMesh *smesh_out,
		std::vector<superfacets>& spf_others,
		int &component_num
	)
	{
		std::vector<superfacets> spf_temp;
		std::vector< std::pair<int, SFMesh::Face>> CCV;
		CCV.clear();
		std::stack<SFMesh::Face> sf;
		SFMesh::Face fpt = *smesh_out->faces().begin();

		for (auto f : smesh_out->faces())
		{
			if (!smesh_out->get_face_component_visit[f])
			{
				smesh_out->get_face_component_visit[f] = true;
				CCV.push_back(std::make_pair(0, f));

				superfacets current_temp;
				spf_temp.push_back(current_temp);
				spf_temp[CCV.size() - 1].sum_area = 0;

				sf.push(f);
				while (!sf.empty())
				{
					fpt = sf.top();
					++CCV.back().first;
					sf.pop();

					smesh_out->get_face_component_id[fpt] = CCV.size();
					for (auto vpt : smesh_out->vertices(fpt))
						smesh_out->get_vert_component_id[vpt] = smesh_out->get_face_component_id[fpt];

					spf_temp[CCV.size() - 1].face_vec.push_back(fpt);
					spf_temp[CCV.size() - 1].sum_area += smesh_out->get_face_area[fpt];

					SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_out->halfedges(fpt);
					SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
					do
					{
						SFMesh::Halfedge ho = smesh_out->opposite_halfedge(*h_fit);
						if (smesh_out->is_boundary(ho) == false)
						{
							SFMesh::Edge eo = smesh_out->edge(ho);
							SFMesh::Face l = smesh_out->face(ho);
							if (!smesh_out->get_face_component_visit[l])
							{
								smesh_out->get_face_component_visit[l] = true;
								sf.push(l);
							}
						}
						++h_fit;
					} while (h_fit != h_end);
				}
			}
		}

		component_num = CCV.size();
		sort(spf_temp.begin(), spf_temp.end(), larger_segments_area);
		if (spf_temp.size() != 1)
			spf_others.insert(spf_others.end(), spf_temp.begin() + 1, spf_temp.end());

		return smesh_out->get_face_component_id[spf_temp[0].face_vec[0]];
	}



	//find seed vertex with highest planarity for region growing
	void initial_vertex_planarity
	(
		SFMesh *smesh_out,
		PTCloud* cloud_pt_3d,
		std::vector<vertex_planarity> &current_mesh_planarity
	)
	{
		std::cout << "Compute initial vertex planarity for vertex" << std::endl;
		const double t_total = omp_get_wtime();
		//for compute vertex planarity
		easy3d::KdTree *cloud_pt_3d_tree = new easy3d::KdTree;
		Build_kdtree(cloud_pt_3d, cloud_pt_3d_tree);
		auto cloud_point_coord = cloud_pt_3d->get_vertex_property<vec3>("v:point");
		auto vertex_point_coord = smesh_out->get_vertex_property<vec3>("v:point");
		float sqr_range = short_range_radius_default * short_range_radius_default;

		for (auto vi : smesh_out->vertices())
		{
			//for compute vertex planarity
			std::vector<int> neighbor_indices;
			std::vector<Point_3> point_within_radius;
			cloud_pt_3d_tree->find_points_in_radius(vertex_point_coord[vi], sqr_range, neighbor_indices);
			for (auto ni : neighbor_indices)
			{
				PTCloud::Vertex vtx(ni);
				point_within_radius.emplace_back(cloud_point_coord[vtx].x, cloud_point_coord[vtx].y, cloud_point_coord[vtx].z);
			}

			Plane plcgal;
			smesh_out->get_vert_planarity[vi] = CGAL::linear_least_squares_fitting_3(point_within_radius.begin(), point_within_radius.end(), plcgal, CGAL::Dimension_tag<0>());
			vertex_planarity vp_temp = std::make_pair(vi, smesh_out->get_vert_planarity[vi]);
			current_mesh_planarity.emplace_back(vp_temp);
		}

		//for compute spherical neighbor and face color 
		float sqr_spherical_range = mr_facet_neg_sphericial_radius * mr_facet_neg_sphericial_radius;
		for (auto fi : smesh_out->faces())
		{
			for (auto vi : smesh_out->vertices(fi))
				smesh_out->get_face_center[fi] += smesh_out->get_points_coord[vi];
			smesh_out->get_face_center[fi] /= 3.0f;

			std::vector<int>  neighbor_indices;
			cloud_pt_3d_tree->find_points_in_radius(smesh_out->get_face_center[fi], sqr_spherical_range, neighbor_indices);
			for (auto ni : neighbor_indices)
			{
				PTCloud::Vertex vtx(ni);
				smesh_out->get_face_spherical_neg_points[fi].push_back(cloud_point_coord[vtx]);
			}
		}

		delete cloud_pt_3d_tree;
		std::cout << " Done in (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
	}

	//--- mohaverdi region growing conditions ---
	bool region_growing_conditions
	(
		SFMesh* smesh_out,
		vec3 &non_vert,
		SFMesh::Face &fdx,
		SFMesh::Face &f_neg,
		vec4 &current_plane_param,
		vec3 &current_avg_color,
		float &region_area
	)
	{
		//limit distances
		float dis_plane = dist2plane(current_plane_param, non_vert);

		//angle condition
		vec3 n_current(current_plane_param.x, current_plane_param.y, current_plane_param.z);
		std::set<Point_3> Neg_ExactVec_CGAL;
		for (auto v : smesh_out->vertices(f_neg))
		{
			vec3 pti = smesh_out->get_points_coord[v];
			Neg_ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
		}
		for (auto pti : smesh_out->get_face_spherical_neg_points[f_neg])
		{
			Neg_ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
		}
		vec3 n_neg;
		Plane plcgal_neg;
		CGAL::linear_least_squares_fitting_3(Neg_ExactVec_CGAL.begin(), Neg_ExactVec_CGAL.end(), plcgal_neg, CGAL::Dimension_tag<0>());
		n_neg.x = plcgal_neg.a();
		n_neg.y = plcgal_neg.b();
		n_neg.z = plcgal_neg.c();
		float angle_current = vector3D_angle(n_current, n_neg);
		if (angle_current >= 180.0f)
			angle_current = angle_current - 180.0f;
		if (angle_current > 90.0f)
			angle_current = 180.0f - angle_current;
		
		//L1 color distance
		vec3 f_c = smesh_out->get_face_color[f_neg];
		float color_L1_dis = std::abs(current_avg_color.x - f_c.x) + std::abs(current_avg_color.y - f_c.y) + std::abs(current_avg_color.z - f_c.z);
	
		//If add neight the new area is 
		float new_area = region_area + smesh_out->get_face_area[f_neg];

		//condition check
		if (dis_plane <= mr_limit_distance
			&& angle_current <= mr_angle_thres
			&& color_L1_dis <= mr_L1_color_dis
			&& new_area <= mr_max_sp_area)
			return true;
		else
			return false;
	}

	//--- Mohammad Rouhani and Yannick Verdie "Mesh" region growing ---//
	void mohaverdi_region_growing_on_mesh
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp,
		std::vector<vertex_planarity> &mesh_vertex_planarity,
		const int &current_ccv_id,
		int &id
	)
	{
		for (auto v : mesh_vertex_planarity)
		{
			if (smesh_out->get_vert_component_id[v.first] != current_ccv_id)
				continue;

			std::vector<SFMesh::Face> current_region, current_seeds;
			std::set<Point_3> ExactVec_CGAL;

			vec3 center_point, current_accum_color;
			vec4 current_plane_param;
			float sum_area = 0.0f;
			for (auto f : smesh_out->faces(v.first))//first ring
			{
				SFMesh::Face fdx = f;
				if (smesh_out->get_face_check[f]
					|| smesh_out->get_face_component_id[f] != current_ccv_id)
					continue;

				current_seeds.emplace_back(f);
				current_region.emplace_back(f);
				smesh_out->get_face_check[f] = true;

				for (auto v : smesh_out->vertices(f))
				{
					vec3 pti = smesh_out->get_points_coord[v];
					ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
				}

				current_accum_color = smesh_out->get_face_color[f];
				sum_area += smesh_out->get_face_area[f];

				for (auto pti : smesh_out->get_face_spherical_neg_points[f])
				{
					ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
				}
				Plane plcgal;
				CGAL::linear_least_squares_fitting_3(ExactVec_CGAL.begin(), ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>());
				current_plane_param.x = plcgal.a();
				current_plane_param.y = plcgal.b();
				current_plane_param.z = plcgal.c();
				current_plane_param.w = -plcgal.d();

				for (int i = 0; i < current_seeds.size(); ++i)
				{
					if (sum_area > mr_max_sp_area)
						break;
					SFMesh::Face f_cu = current_seeds[i];
					// loop over all incident faces around the face
					SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_out->halfedges(current_seeds[i]);
					SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
					do {
						SFMesh::Halfedge ho = smesh_out->opposite_halfedge(*h_fit);
						if (smesh_out->is_boundary(ho) == false)
						{
							SFMesh::Edge eo = smesh_out->edge(ho);
							SFMesh::Vertex vs = smesh_out->vertex(eo, 0);
							SFMesh::Vertex vt = smesh_out->vertex(eo, 1);
							SFMesh::Vertex non_vert;

							SFMesh::Face f_neg = smesh_out->face(ho);
							if (smesh_out->get_face_check[f_neg])
							{
								++h_fit;
								continue;
							}

							//find non-attached vertex
							for (auto current_v : smesh_out->vertices(f_neg))
							{
								if (current_v != vs && current_v != vt)
								{
									non_vert = current_v;
									break;
								}
							}

							bool merge_or_not = false;
							vec3 current_avg_color = current_accum_color / float(current_region.size());
							vec3 non_touch_vertex = smesh_out->get_points_coord[non_vert];
							merge_or_not = region_growing_conditions(smesh_out, non_touch_vertex, fdx, f_neg, current_plane_param, current_avg_color, sum_area);

							if (merge_or_not == true)
							{
								smesh_out->get_face_check[f_neg] = true;

								//update region info
								current_region.emplace_back(f_neg);
								current_seeds.emplace_back(f_neg);
								current_accum_color += smesh_out->get_face_color[f_neg];
								sum_area += smesh_out->get_face_area[f_neg];

								for (auto v : smesh_out->vertices(f_neg))
								{
									vec3 pti = smesh_out->get_points_coord[v];
									ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
								}
								for (auto pti : smesh_out->get_face_spherical_neg_points[f_neg])
								{
									ExactVec_CGAL.insert(Point_3(pti.x, pti.y, pti.z));
								}
								Plane plcgal;
								CGAL::linear_least_squares_fitting_3(ExactVec_CGAL.begin(), ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>());
								current_plane_param.x = plcgal.a();
								current_plane_param.y = plcgal.b();
								current_plane_param.z = plcgal.c();
								current_plane_param.w = -plcgal.d();

								if (!smesh_out->get_vert_check[non_vert])
									smesh_out->get_vert_check[non_vert] = true;
							}
						}
						++h_fit;
					} while (h_fit != h_end);
				}
			}

			if (!current_region.empty())
			{
				superfacets temp(id, current_region, current_plane_param);
				//temp.ExactVec_CGAL = ExactVec_CGAL;

				spf_temp.emplace_back(temp);
				++id;
			}
		}
	}

	//Mohammad Rouhani and Yannick Verdie Over-segmentation pipeline
	void perform_mohaverdi_oversegmentation
	(
		SFMesh *smesh_out,
		std::vector<superfacets>& spf_result,
		std::vector<vertex_planarity> &mesh_vertex_planarity,
		std::map<int, int> &faceid_segid_map,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		//1. Extract small connected components
		std::vector<superfacets> spf_others;
		int component_num = 0;
		int largest_seg_id = separate_connected_components(smesh_out, spf_others, component_num);

		//2. Region growing
		int segment_id = 0;
		std::cout << "  Region growing, use ";
		const double t_total = omp_get_wtime();
		sort(mesh_vertex_planarity.begin(), mesh_vertex_planarity.end(), larger_planarity<vertex_planarity>);
		for (int ccv_id = 0; ccv_id <= component_num; ++ccv_id)
			mohaverdi_region_growing_on_mesh(smesh_out, spf_result, mesh_vertex_planarity, ccv_id, segment_id);

		for (int spf_i = 0; spf_i < spf_result.size(); ++spf_i)
		{
			spf_result[spf_i].id = spf_i;
			seg_neighbors[spf_i] = std::vector<int>(spf_result.size(), -1);
			for (auto fd : spf_result[spf_i].face_vec)
			{
				smesh_out->get_face_segment_id[fd] = spf_i;
				faceid_segid_map[fd.idx()] = spf_i;
			}
		}

		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
		std::cout << "The number of segments are " << spf_result.size() << std::endl;
	}

	//--- point cloud region growing clustering ---
	void cgal_regiongrowing
	(
		PTCloud *pcl_in,
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_tmp,
		std::map<int, int> &faceid_segid_map,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		const bool with_normal_map = true;
		Point_range cpcl_out(with_normal_map);
		parsing_to_pcl(pcl_in, cpcl_out);

		// Default parameter values for the data file polygon_mesh.off.
		const std::size_t k = pcl_k_nn;
		const FT          max_distance_to_plane = FT(pcl_distance_to_plane);
		const FT          max_accepted_angle = FT(pcl_accepted_angle);
		const std::size_t min_region_size = pcl_minimum_region_size;

		// Create instances of the classes Neighbor_query and Region_type.
		Neighbor_query_pcl neighbor_query(cpcl_out, k, cpcl_out.point_map());

		region_type_pcl region_type(
			cpcl_out,
			max_distance_to_plane, max_accepted_angle, min_region_size,
			cpcl_out.point_map(), cpcl_out.normal_map());

		// Create an instance of the region growing class.
		Region_growing_pcl region_growing(
			cpcl_out, neighbor_query, region_type);

		// Run the algorithm.
		regions_for regions;
		region_growing.detect(std::back_inserter(regions));

		//parsing regions to point cloud
		std::map<int, int> ptx_ri;
		smesh_in->add_face_property<std::map<int, int>>("f:face_rg", std::map<int, int>());
		auto face_rg_majority = smesh_in->get_face_property<std::map<int, int>>("f:face_rg");
		int rg_ind = 0;
		for (auto rg : regions)
		{
			for (auto ri : rg)
			{
				PTCloud::Vertex vtx(ri);
				ptx_ri[ri] = rg_ind;
				int f_id = pcl_in->get_points_face_belong_id[vtx];
				SFMesh::Face fd(f_id);
				auto it = face_rg_majority[fd].find(rg_ind);
				if (it == face_rg_majority[fd].end())
					face_rg_majority[fd][rg_ind] = 1;
				else
					++face_rg_majority[fd][rg_ind];
			}
			++rg_ind;
		}

		//get marjority region of facets
		std::vector<std::vector< SFMesh::Face>> rg_tmp;
		rg_tmp.resize(regions.size());
		for (auto fd : smesh_in->faces())
		{
			if (face_rg_majority[fd].empty())
			{
				int ptxi = smesh_in->get_face_sampled_points[fd][0];
				rg_tmp[ptx_ri[ptxi]].push_back(fd);
			}
			else
			{
				std::multimap<int, int> dst = flip_map(face_rg_majority[fd]);
				//smesh_in->get_face_segment_id[fd] = dst.begin()->second;
				//std::cout << "total: " << regions.size() << "; fd = " << fd <<"; current = (" << dst.begin()->first<< ", "<< dst.begin()->second <<") "<< std::endl;
				rg_tmp[dst.begin()->second].push_back(fd);
			}
		}

		//paring region to mesh in continuous order
		rg_ind = 0;
		for (auto rg : rg_tmp)
		{
			if (!rg.empty())
			{
				superfacets spf_temp;
				seg_neighbors[rg_ind] = std::vector<int>(rg_tmp.size(), -1);
				spf_temp.id = rg_ind;
				for (auto fi : rg)
				{
					SFMesh::Face fdx(fi);
					smesh_in->get_face_segment_id[fdx] = rg_ind;
					faceid_segid_map[fdx.idx()] = rg_ind;
					spf_temp.face_vec.emplace_back(fdx);
					spf_temp.sum_area += smesh_in->get_face_area[fdx];
				}
				++rg_ind;
				segment_tmp.emplace_back(spf_temp);
			}
		}

		smesh_in->remove_face_property(face_rg_majority);
	}

	//--- mesh region growing clustering ---
	void cgal_regiongrowing
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_tmp,
		std::map<int, int> &faceid_segid_map,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		Mesh cmesh_out;
		parsing_to_mesh(smesh_in, cmesh_out);

		const Face_range face_range = faces(cmesh_out);

		// Default parameter values for the data file polygon_mesh.off.
		const FT          max_distance_to_plane = FT(mesh_distance_to_plane);
		const FT          max_accepted_angle = FT(mesh_accepted_angle);
		const std::size_t min_region_size = mesh_minimum_region_size;

		// Create instances of the classes Neighbor_query and Region_type.
		Neighbor_query_mesh neighbor_query(cmesh_out);
		const Vertex_to_point_map_mesh vertex_to_point_map(get(CGAL::vertex_point, cmesh_out));

		region_type_mesh region_type(
			cmesh_out,
			max_distance_to_plane, max_accepted_angle, min_region_size,
			vertex_to_point_map);

		// Sort face indices.
		sorting_mesh sorting(
			cmesh_out, neighbor_query,
			vertex_to_point_map);
		sorting.sort();

		// Create an instance of the region growing class.
		Region_growing_mesh region_growing(
			face_range, neighbor_query, region_type,
			sorting.seed_map());

		// Run the algorithm.
		regions_for regions;
		region_growing.detect(std::back_inserter(regions));

		//paring region to mesh
		int rg_ind = 0;
		for (auto rg : regions)
		{
			if (!rg.empty())
			{
				superfacets spf_temp;
				seg_neighbors[rg_ind] = std::vector<int>(regions.size(), -1);
				spf_temp.id = rg_ind;
				for (auto fi : rg)
				{
					SFMesh::Face fdx(fi);
					smesh_in->get_face_segment_id[fdx] = rg_ind;
					faceid_segid_map[fdx.idx()] = rg_ind;
					spf_temp.face_vec.emplace_back(fdx);
					spf_temp.sum_area += smesh_in->get_face_area[fdx];
				}
				++rg_ind;
				segment_tmp.emplace_back(spf_temp);
			}
		}
	}

	void finding_adjacent_segments
	(
		SFMesh *smesh_in,
		PTCloud *face_center_cloud_in,
		std::vector<superfacets> &segment_in,
		std::map<int, int> &faceid_segid_map,
		std::map<int, int> &ptidx_faceid_map,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		easy3d::KdTree *face_center_tree = new easy3d::KdTree;
		Build_kdtree(face_center_cloud_in, face_center_tree);
		float sqr_range = adjacent_radius * adjacent_radius;
#pragma omp parallel for schedule(dynamic)
		for (int vi = 0; vi < face_center_cloud_in->vertices_size(); ++vi)
		{
			PTCloud::Vertex ptx(vi);
			int fi_tmp = face_center_cloud_in->get_points_face_belong_id[ptx];
			SFMesh::Face fdx(fi_tmp);
			int seg_ind = smesh_in->get_face_segment_id[fdx];

			face_center_tree->find_points_in_radius(
				face_center_cloud_in->get_points_coord[ptx],
				sqr_range,
				ptidx_faceid_map,
				faceid_segid_map,
				seg_neighbors,
				seg_ind);
		}

#pragma omp parallel for schedule(dynamic)
		for (int seg_i = 0; seg_i < segment_in.size(); ++seg_i)
		{
			sort(seg_neighbors[seg_i].begin(), seg_neighbors[seg_i].end());
			seg_neighbors[seg_i].erase(unique(seg_neighbors[seg_i].begin(), seg_neighbors[seg_i].end()), seg_neighbors[seg_i].end());
			seg_neighbors[seg_i].erase(seg_neighbors[seg_i].begin());
			segment_in[seg_i].neighbor_ids = seg_neighbors[seg_i];
		}
		delete face_center_tree;
	}

	void segment_region_growing
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_in,
		std::vector<superfacets> &segment_merged
	)
	{
		//get_current segment information
#pragma omp parallel for schedule(dynamic)
		for (int spf_i = 0; spf_i < segment_in.size(); ++spf_i)
		{
			std::map<SFMesh::Vertex, bool> visited_vert;
			for (int fi = 0; fi < segment_in[spf_i].face_vec.size(); ++fi)
			{
				for (auto vdx : smesh_in->vertices(segment_in[spf_i].face_vec[fi]))
				{
					if (visited_vert.find(vdx) == visited_vert.end())
					{
						visited_vert[vdx] = false;
						vec3 p_temp = smesh_in->get_points_coord[vdx];
						segment_in[spf_i].ExactVec_CGAL.emplace_back(p_temp.x, p_temp.y, p_temp.z);
					}
				}
				segment_in[spf_i].sum_area += smesh_in->get_face_area[segment_in[spf_i].face_vec[fi]];
			}

			Plane plane_c;
			CGAL::linear_least_squares_fitting_3(segment_in[spf_i].ExactVec_CGAL.begin(), segment_in[spf_i].ExactVec_CGAL.end(), plane_c, CGAL::Dimension_tag<0>());
			easy3d::vec4 plane_param(plane_c.a(), plane_c.b(), plane_c.c(), -plane_c.d());
			segment_in[spf_i].plane_parameter = plane_param;
		}


		std::vector<superfacets> segment_sorted;
		segment_sorted.insert(segment_sorted.end(), segment_in.begin(), segment_in.end());
		sort(segment_sorted.begin(), segment_sorted.end(), larger_segments_area);

		//segment region growing
		int prev_percent = -1;
		for (int spf_soi = 0; spf_soi < segment_sorted.size(); ++spf_soi)
		{
			int spf_i = segment_sorted[spf_soi].id;
			int progress = int(100.0f * (spf_soi + 1) / float(segment_sorted.size()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			if (!segment_in[spf_i].seg_visited)
			{
				segment_in[spf_i].seg_visited = true;
				segment_merged.push_back(segment_in[spf_i]);
				std::vector<superfacets> current_seeds;
				current_seeds.push_back(segment_in[spf_i]);

				for (int spf_ci = 0; spf_ci < current_seeds.size(); ++spf_ci)
				{
					vec3 cur_n(current_seeds[spf_ci].plane_parameter.x, current_seeds[spf_ci].plane_parameter.y, current_seeds[spf_ci].plane_parameter.z);
					for (int spf_ni = 0; spf_ni < current_seeds[spf_ci].neighbor_ids.size(); ++spf_ni)
					{
						int neg_ind = current_seeds[spf_ci].neighbor_ids[spf_ni];
						if (!segment_in[neg_ind].seg_visited)
						{
							//distance only
							float dis2plane_avg = 0.0f;
#pragma omp parallel for reduction(+:dis2plane_avg)
							for (int pi = 0; pi < segment_in[neg_ind].ExactVec_CGAL.size(); ++pi)
							{
								dis2plane_avg += dist2plane(segment_merged[segment_merged.size() - 1].plane_parameter, segment_in[neg_ind].ExactVec_CGAL[pi]);
							}
							dis2plane_avg /= segment_in[neg_ind].ExactVec_CGAL.size();

							//angle
							vec3 neg_n(segment_in[neg_ind].plane_parameter.x, segment_in[neg_ind].plane_parameter.y, segment_in[neg_ind].plane_parameter.z);
							float ang_tmp = vector3D_angle(cur_n, neg_n);
							ang_tmp = ang_tmp > 90.0f ? 180.0f - ang_tmp : ang_tmp;


							if (dis2plane_avg <= adjacent_pt2plane_distance && ang_tmp <= adjacent_seg_angle)
							{
								segment_in[neg_ind].seg_visited = true;
								current_seeds.push_back(segment_in[neg_ind]);
								segment_merged[segment_merged.size() - 1].face_vec.insert
								(
									segment_merged[segment_merged.size() - 1].face_vec.end(),
									segment_in[neg_ind].face_vec.begin(),
									segment_in[neg_ind].face_vec.end()
								);

								segment_merged[segment_merged.size() - 1].ExactVec_CGAL.insert
								(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(),
									segment_in[neg_ind].ExactVec_CGAL.begin(),
									segment_in[neg_ind].ExactVec_CGAL.end()
								);

								//update plane parameters
								Plane plcgal;
								CGAL::linear_least_squares_fitting_3(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.begin(),
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>()
								);
								segment_merged[segment_merged.size() - 1].plane_parameter.x = plcgal.a();
								segment_merged[segment_merged.size() - 1].plane_parameter.y = plcgal.b();
								segment_merged[segment_merged.size() - 1].plane_parameter.z = plcgal.c();
								segment_merged[segment_merged.size() - 1].plane_parameter.w = -plcgal.d();
							}
						}
					}
				}
			}
		}

		//assign the new segment to face 
#pragma omp parallel for schedule(dynamic)
		for (int spf_i = 0; spf_i < segment_merged.size(); ++spf_i)
		{
			for (int fi = 0; fi < segment_merged[spf_i].face_vec.size(); ++fi)
			{
				smesh_in->get_face_segment_id[segment_merged[spf_i].face_vec[fi]] = spf_i;
			}
		}
	}

	void merge_segments
	(
		SFMesh *smesh_in,
		PTCloud *face_center_cloud_in,
		std::vector<superfacets> &segment_in,
		std::vector<superfacets> &segment_merged,
		std::map<int, int> &faceid_segid_map,
		std::map<int, int> &ptidx_faceid_map,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		finding_adjacent_segments
		(
			smesh_in,
			face_center_cloud_in,
			segment_in,
			faceid_segid_map,
			ptidx_faceid_map,
			seg_neighbors
		);

		segment_region_growing(smesh_in, segment_in, segment_merged);
	}


	void get_segments_color
	(
		SFMesh* tmp_mesh,
		std::vector<superfacets> &spf_current
	)
	{
		tmp_mesh->get_face_color = tmp_mesh->get_face_property<vec3>("f:color");
		std::srand(std::time(0));
		int R = rand() % 256;
		int G = rand() % 256;
		int B = rand() % 256;
		for (int spi = 0; spi < spf_current.size(); ++spi)
		{
			R = rand() % 256;
			G = rand() % 256;
			B = rand() % 256;
			for (int j = 0; j < spf_current[spi].face_vec.size(); ++j)
			{
				tmp_mesh->get_face_color[spf_current[spi].face_vec[j]] = vec3(R / 255.0f, G / 255.0f, B / 255.0f);
			}
		}
	}

	//------ PSSNet over-segmentation -------
	void plane_fitting_unary_term
	(
		SFMesh* smesh_out,
		SFMesh::Vertex &non_vert,
		SFMesh::Face &f_neg,
		vec4 &current_plane_param,
		std::pair<int, float> &current_label_prob,
		float &unary
	)
	{
		float dis2plane = dist2plane(current_plane_param, smesh_out->get_points_coord[non_vert]);
		unary = dis2plane * mrf_lambda_d;// > 1.0f ? 1.0f : dis_plane;

		int current_label_f_neg = -1;
		float face_predict_prob_f_neg = 1.0f;
		if (train_test_predict_val != 0)
		{
			current_label_f_neg = smesh_out->get_face_predict_label[f_neg] - 1;
			face_predict_prob_f_neg = smesh_out->get_face_predict_prob[f_neg];
		}
		else
		{
			current_label_f_neg = smesh_out->get_face_truth_label[f_neg] - 1;
		}

		float mrf_lambda_g_tmp = mrf_lambda_g;
		if (train_test_predict_val == 0)
			mrf_lambda_g_tmp = 1.0f;

		if (current_label_prob.first > -1 && current_label_f_neg > -1)
		{
			std::vector<std::string> current_label_names = get_input_names_with_delim("_", labels_name_pnp[current_label_prob.first]);
			std::vector<std::string> f_neg_label_names = get_input_names_with_delim("_", labels_name_pnp[current_label_f_neg]);

			if (current_label_names[0] == "non" &&
				f_neg_label_names[0] == "non")
			{
				if (current_label_names.size() == 2
					|| (current_label_names.size() > 2 && current_label_names[2] == f_neg_label_names[2]))
				{
					float prob = 1.0f - mrf_lambda_g_tmp * face_predict_prob_f_neg;
					unary = std::min(dis2plane, prob);
				}
				else
				{
					unary = mrf_energy_amplify;
				}
			}
			else if (labels_name_pnp.size() > 2
				&& current_label_names[0] != "non"
				&& f_neg_label_names[0] != "non")
			{
				if (labels_name_pnp[current_label_prob.first] != labels_name_pnp[current_label_f_neg]
					&& current_label_prob.second != 1.0f && face_predict_prob_f_neg != 1.0f)
				{

					float prob_fdx = mrf_lambda_g_tmp * current_label_prob.second;
					float prob_f_neg = mrf_lambda_g_tmp * face_predict_prob_f_neg;

					unary = prob_fdx * mrf_lambda_g_tmp > prob_f_neg ? prob_f_neg : mrf_energy_amplify;
				}
				else if (labels_name_pnp[current_label_prob.first] != labels_name_pnp[current_label_f_neg])
				{
					unary = mrf_energy_amplify;
				}
			}
			//else
			//{
			//  //We put the following before the condition, the results are same
			//	float prob = FLT_MAX;
			//	unary = std::min(dis2plane, prob) * mrf_lambda_d;
			//}
		}
	}

	//Plane fitting based region growing
	void region_growing_on_mesh_with_all_neighbors
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp,
		std::vector<vertex_planarity> &mesh_vertex_planarity,
		const int &current_ccv_id,
		int &id
	)
	{
		for (auto v : mesh_vertex_planarity)
		{
			if (smesh_out->get_vert_component_id[v.first] != current_ccv_id)
				continue;

			std::vector<SFMesh::Face> current_region, current_seeds;
			std::vector<Point_3> ExactVec_CGAL;

			vec3 center_point;
			vec4 current_plane_param;
			float sum_area = 0.0f;
			std::pair<std::vector<int>, float> current_label_prob(std::vector<int>(labels_name_pnp.size(), 0), 0.0f);

			for (auto f : smesh_out->faces(v.first))//first ring
			{
				SFMesh::Face fdx = f;
				if (smesh_out->get_face_check[f]
					|| smesh_out->get_face_component_id[f] != current_ccv_id)
					continue;

				current_seeds.push_back(f);
				current_region.push_back(f);

				smesh_out->get_face_check[f] = true;
				sum_area += smesh_out->get_face_area[f];

				if (current_region.size() == 1)
				{
					if (train_test_predict_val != 0)
					{
						current_label_prob.first[smesh_out->get_face_predict_label[current_region[0]] - 1] += 1;
						current_label_prob.second += smesh_out->get_face_predict_prob[current_region[0]] * smesh_out->get_face_area[current_region[0]];
					}
					else
					{
						if (smesh_out->get_face_truth_label[current_region[0]] > 0)
						{
							current_label_prob.first[smesh_out->get_face_truth_label[current_region[0]] - 1] += 1;
							current_label_prob.second += smesh_out->get_face_area[current_region[0]];
						}
					}
				}

				for (auto v : smesh_out->vertices(f))
				{
					center_point += smesh_out->get_points_coord[v];
					ExactVec_CGAL.push_back(Point_3(smesh_out->get_points_coord[v].x, smesh_out->get_points_coord[v].y, smesh_out->get_points_coord[v].z));
				}
				center_point /= 3;

				current_plane_param = smesh_out->get_face_normals[f];
				double nfd = (smesh_out->get_face_normals[f].x*center_point.x + smesh_out->get_face_normals[f].y * center_point.y + smesh_out->get_face_normals[f].z * center_point.z);
				current_plane_param.w = nfd;

				for (int i = 0; i < current_seeds.size(); ++i)
				{
					SFMesh::Face f_cu = current_seeds[i];

					//build local graph of the current seed
					std::map<int, int> local_old_new;
					std::map<int, bool> local_face_visited;
					std::vector<float> unary_terms;
					std::vector<std::pair<int, int>> pairwise_neighbors;
					std::map<std::pair<int, int>, bool> pairwise_checked;
					std::vector<float> pairwise_terms;
					std::vector<int> collected_faces, label_out;
					std::vector<SFMesh::Vertex> collected_non_vertices;
					collected_faces.push_back(f_cu.idx());
					unary_terms.push_back(0.0f);
					local_face_visited[f_cu.idx()] = true;
					local_old_new[f_cu.idx()] = collected_faces.size() - 1;

					// loop over all incident faces around the face
					SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_out->halfedges(current_seeds[i]);
					SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
					do
					{
						SFMesh::Halfedge ho = smesh_out->opposite_halfedge(*h_fit);
						if (smesh_out->is_boundary(ho) == false)
						{
							SFMesh::Edge eo = smesh_out->edge(ho);
							SFMesh::Vertex vs = smesh_out->vertex(eo, 0);
							SFMesh::Vertex vt = smesh_out->vertex(eo, 1);
							SFMesh::Vertex non_vert;

							SFMesh::Face f_neg = smesh_out->face(ho);
							collected_faces.push_back(f_neg.idx());
							local_face_visited[f_neg.idx()] = false;
							local_old_new[f_neg.idx()] = collected_faces.size() - 1;

							//find non-attached vertex
							for (auto current_v : smesh_out->vertices(f_neg))
							{
								if (current_v != vs && current_v != vt)
								{
									non_vert = current_v;
									break;
								}
							}
							collected_non_vertices.push_back(non_vert);

							float unary = 0.0f;
							int current_seg_label = std::distance(current_label_prob.first.begin(), std::max_element(current_label_prob.first.begin(), current_label_prob.first.end()));
							float seg_prob = current_label_prob.second / sum_area;

							plane_fitting_unary_term(smesh_out, non_vert, f_neg, current_plane_param, std::make_pair(current_seg_label, seg_prob), unary);
							unary_terms.push_back(unary);

							if (pairwise_checked[std::make_pair(f_cu.idx(), f_neg.idx())] == false
								&& pairwise_checked[std::make_pair(f_neg.idx(), f_cu.idx())] == false)
							{
								pairwise_checked[std::make_pair(f_cu.idx(), f_neg.idx())] = true;
								pairwise_checked[std::make_pair(f_neg.idx(), f_cu.idx())] = true;
								pairwise_neighbors.push_back(std::make_pair(local_old_new[f_cu.idx()], local_old_new[f_neg.idx()]));

								vec3 n_current(current_plane_param.x, current_plane_param.y, current_plane_param.z);
								float angle_current = vector3D_angle(n_current, smesh_out->get_face_normals[f_neg]);

								if (angle_current >= 180.0f)
									angle_current = angle_current - 180.0f;
								if (angle_current > 90.0f)
									angle_current = 180.0f - angle_current;
								angle_current = 1.0f - angle_current / 90.0f;
								float pairwise = angle_current;
								pairwise_terms.push_back(pairwise);
							}
						}
						++h_fit;
					} while (h_fit != h_end);

					MRF_oversegmentation(unary_terms, pairwise_neighbors, pairwise_terms, label_out);

					for (int li = 1; li < label_out.size(); ++li)
					{
						if (label_out[0] == label_out[li])
						{
							SFMesh::Face f_r_ng(collected_faces[li]);
							if (smesh_out->get_face_check[f_r_ng])
							{
								continue;
							}

							smesh_out->get_face_check[f_r_ng] = true;
							current_region.push_back(f_r_ng);
							current_seeds.push_back(f_r_ng);
							sum_area += smesh_out->get_face_area[f_r_ng];

							if (train_test_predict_val != 0)
							{
								current_label_prob.first[smesh_out->get_face_predict_label[current_region[0]] - 1] += 1;
								current_label_prob.second += smesh_out->get_face_predict_prob[current_region[0]] * smesh_out->get_face_area[current_region[0]];
							}
							else
							{
								if (smesh_out->get_face_truth_label[current_region[0]] > 0)
								{
									current_label_prob.first[smesh_out->get_face_truth_label[current_region[0]] - 1] += 1;
									current_label_prob.second += smesh_out->get_face_area[current_region[0]];
								}
							}

							if (!smesh_out->get_vert_check[collected_non_vertices[li - 1]])
							{
								smesh_out->get_vert_check[collected_non_vertices[li - 1]] = true;
								ExactVec_CGAL.push_back(Point_3(smesh_out->get_points_coord[collected_non_vertices[li - 1]].x, smesh_out->get_points_coord[collected_non_vertices[li - 1]].y, smesh_out->get_points_coord[collected_non_vertices[li - 1]].z));
								Plane plcgal;
								// fit plane to whole triangles
								CGAL::linear_least_squares_fitting_3(ExactVec_CGAL.begin(), ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>());
								current_plane_param.x = plcgal.a();
								current_plane_param.y = plcgal.b();
								current_plane_param.z = plcgal.c();
								current_plane_param.w = -plcgal.d();
							}
						}
					}
				}
			}

			if (!current_region.empty())
			{

				superfacets temp(id, current_region, current_plane_param);
				temp.ExactVec_CGAL = ExactVec_CGAL;
				temp.label = std::distance(current_label_prob.first.begin(), std::max_element(current_label_prob.first.begin(), current_label_prob.first.end()));
				spf_temp.push_back(temp);
				++id;
			}
		}
	}

	//get 1-ring neighbor
	void get_one_ring_neighbor
	(
		SFMesh *smesh_out,
		SFMesh::Vertex &v,
		SFMesh::Face &f
	)
	{
		for (auto f_neg : smesh_out->faces(v))
		{
			if (!f_neg.is_valid())
				continue;
			if (f_neg.idx() == f.idx())
				continue;
			bool temp_check = false;
			for (auto f_neg_tup : smesh_out->get_face_1ring_neighbor[f])
			{
				SFMesh::Face fd_neg_tup(get<0>(f_neg_tup));
				if (!fd_neg_tup.is_valid())
					continue;
				if (get<0>(f_neg_tup) == f_neg.idx())
				{
					temp_check = true;
					break;
				}
			}
			if (temp_check)
				continue;

			//ring1_neg_check[f_neg] = true;
			int f_in_f_neg_ind = 0;
			//check current face neighbor
			//check in current face can also be found in neighbor face neighbor vectors or not
			int f_neg_ind = 0;
			bool found_f_in_neg = false;
			for (auto f_f_neg_tup : smesh_out->get_face_1ring_neighbor[f_neg])
			{
				if (get<0>(f_f_neg_tup) == f.idx())
				{
					f_in_f_neg_ind = f_neg_ind;
					found_f_in_neg = true;
					break;
				}
				++f_neg_ind;
			}

			if (found_f_in_neg)
				smesh_out->get_face_1ring_neighbor[f].emplace_back(f_neg.idx(), f_in_f_neg_ind, false);
			else
			{
				smesh_out->get_face_1ring_neighbor[f].emplace_back(f_neg.idx(), smesh_out->get_face_1ring_neighbor[f_neg].size(), false);
				smesh_out->get_face_1ring_neighbor[f_neg].emplace_back(f.idx(), smesh_out->get_face_1ring_neighbor[f].size() - 1, false);
			}
		}
	}

	//global smooth small segments
	void face_global_smooth_on_mesh(SFMesh *smesh_out)
	{
		//set up 1-ring neighbors for faces
		//int i_count = 0;
		for (auto f : smesh_out->faces())
		{
			for (auto v : smesh_out->vertices(f))
			{
				if (v.is_valid())
					get_one_ring_neighbor(smesh_out, v, f);
			}
			//++i_count;
		}

		//Iterate MRF 1-ring smooth
		int count = 0, current_changes = 0, previous_changes;
		do
		{
			previous_changes = current_changes;
			for (auto f : smesh_out->faces())
			{
				int num_segments = smesh_out->get_face_1ring_neighbor[f].size() + 1;
				std::vector<int> labels_temp;
				bool has_non_planar = false;
				for (auto f_neg_tup : smesh_out->get_face_1ring_neighbor[f])
				{
					SFMesh::Face fdx(get<0>(f_neg_tup));
					int temp_wsid = smesh_out->get_face_smoothed_seg_id[fdx];
					labels_temp.push_back(temp_wsid);
				}

				labels_temp.push_back(smesh_out->get_face_smoothed_seg_id[f]);
				sort(labels_temp.begin(), labels_temp.end());
				labels_temp.erase(unique(labels_temp.begin(), labels_temp.end()), labels_temp.end());
				if (labels_temp.size() == 1)
					continue;

				if (has_non_planar)
					continue;

				MRF_oversegmentation(smesh_out, f, labels_temp, num_segments, current_changes);
			}
			++count;
		} while (current_changes != previous_changes);
		std::cout << "	Global smoothed " << current_changes << " faces." << std::endl;
	}

	void rearrange_segments
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp,
		std::vector<superfacets> &spf_planar_seg
	)
	{
		int new_id = 0;//for spf_out index, same as tri_spid_update
		for (int i = 0; i < spf_temp.size(); ++i)
		{
			std::vector<SFMesh::Face> current_region;
			std::vector<Point_3> ExactVec_CGAL;
			std::map<SFMesh::Vertex, bool> vert_visited;

			for (auto fi : spf_temp[i].face_vec)
			{
				//Check if the face has been assign to more than one segment
				for (auto vi : smesh_out->vertices(fi))
				{
					auto it = vert_visited.find(vi);
					if (it == vert_visited.end())
					{
						SFMesh::Vertex vtx(vi);
						ExactVec_CGAL.emplace_back(smesh_out->get_points_coord[vtx].x, smesh_out->get_points_coord[vtx].y, smesh_out->get_points_coord[vtx].z);
						vert_visited[vtx] = true;
					}
				}

				current_region.emplace_back(fi);
				smesh_out->get_face_spid_update[fi] = new_id;
				smesh_out->get_face_smoothed_seg_id[fi] = smesh_out->get_face_spid_update[fi];
			}

			//plane fitting
			Plane plcgal;
			if (!current_region.empty())
			{
				float fitting_score = CGAL::linear_least_squares_fitting_3(ExactVec_CGAL.begin(), ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>());
				vec4 current_plane_param(plcgal.a(), plcgal.b(), plcgal.c(), -plcgal.d());
				superfacets temp(spf_temp[i].id, current_region, current_plane_param, ExactVec_CGAL);
				temp.plane_cgal = plcgal;
				temp.id = new_id;
				temp.index = new_id;
				temp.label = spf_temp[i].label;
				temp.ExactVec_CGAL = ExactVec_CGAL;

				for (int fi = 0; fi < current_region.size(); ++fi)
				{
					smesh_out->get_face_planar_segment_plane_normal[current_region[fi]] = vec3(current_plane_param.x, current_plane_param.y, current_plane_param.z);
				}

				spf_planar_seg.emplace_back(temp);
				new_id++;
			}
		}
	}

	void parsing_with_planar_region
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp_vec,
		const int wshed_id
	)
	{
		//parsing to new segments
		std::vector<superfacets> spf_temp = std::vector<superfacets>(wshed_id, superfacets());
		for (auto f : smesh_out->faces())
		{
			spf_temp[smesh_out->get_face_smoothed_seg_id[f]].face_vec.emplace_back(f);
		}

		int ind = 0;
		for (auto spf : spf_temp)
		{
			if (!spf.face_vec.empty())
			{
				spf.index = ind;
				spf_temp_vec.emplace_back(spf);
				++ind;
			}
		}

		//Update face segment id
#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < spf_temp_vec.size(); ++i)
		{
			for (auto fi : spf_temp_vec[i].face_vec)
			{
				smesh_out->get_face_smoothed_seg_id[fi] = spf_temp_vec[i].index;
				smesh_out->get_face_segment_id[fi] = spf_temp_vec[i].index;
			}
		}
	}

	void planar_segment_merge_preprocessing
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_in,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		for (int spf_i = 0; spf_i < spf_in.size(); ++spf_i)
		{
			spf_in[spf_i].id = spf_i;
			seg_neighbors[spf_i] = std::vector<int>(spf_in.size(), -1);
			spf_in[spf_i].neighbor_ids.clear();
			spf_in[spf_i].sum_area = 0.0f;

			std::vector<int> majority_labels(labels_name_pnp.size(), 0);
			int count_unclassified = 0;
			for (auto fd : spf_in[spf_i].face_vec)
			{
				spf_in[spf_i].sum_area += smesh_out->get_face_area[fd];
				smesh_out->get_face_segment_id[fd] = spf_i;
				if (labels_name_pnp.size() > 2)
				{
					if (train_test_predict_val == 0)
					{
						if (smesh_out->get_face_truth_label[fd] > 0)
						{
							++majority_labels[smesh_out->get_face_truth_label[fd] - 1];
						}
						else
						{
							++count_unclassified;
						}
					}
					else
					{
						if (smesh_out->get_face_predict_label[fd] != -1)
						{
							++majority_labels[smesh_out->get_face_predict_label[fd] - 1];
						}
						else
						{
							++count_unclassified;
						}
					}
				}
			}

			if (labels_name_pnp.size() > 2)
			{
				int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
				if (train_test_predict_val == 0 && majority_labels[maxElementIndex] < count_unclassified)
				{
					spf_in[spf_i].ground_truth = -1;
					spf_in[spf_i].predict = -1;
					spf_in[spf_i].label = -1;
				}
				else
				{
					spf_in[spf_i].label = maxElementIndex;
					if (train_test_predict_val == 0)
					{
						spf_in[spf_i].ground_truth = maxElementIndex;
					}
					else
					{
						spf_in[spf_i].predict = maxElementIndex;
					}
				}
			}
		}
	}

	void finding_adjacent_segments_based_on_connectivity
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_in,
		std::map<int, std::vector<int>> &seg_neighbors
	)
	{
		for (int seg_i = 0; seg_i < segment_in.size(); ++seg_i)
		{
			for (auto &fd : segment_in[seg_i].face_vec)
			{
				SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_in->halfedges(fd);
				SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
				do {
					SFMesh::Halfedge ho = smesh_in->opposite_halfedge(*h_fit);
					if (smesh_in->is_boundary(ho) == false)
					{
						SFMesh::Face f_neg = smesh_in->face(ho);
						int neg_seg_id = smesh_in->get_face_segment_id[f_neg];
						if (neg_seg_id != segment_in[seg_i].id)
						{
							seg_neighbors[segment_in[seg_i].id][neg_seg_id] = neg_seg_id;
						}
					}
					++h_fit;
				} while (h_fit != h_end);
			}
		}

#pragma omp parallel for schedule(dynamic)
		for (int seg_i = 0; seg_i < segment_in.size(); ++seg_i)
		{
			sort(seg_neighbors[seg_i].begin(), seg_neighbors[seg_i].end());
			seg_neighbors[seg_i].erase(unique(seg_neighbors[seg_i].begin(), seg_neighbors[seg_i].end()), seg_neighbors[seg_i].end());
			seg_neighbors[seg_i].erase(seg_neighbors[seg_i].begin());
			segment_in[seg_i].neighbor_ids = seg_neighbors[seg_i];
			segment_in[seg_i].seg_visited = false;
		}
	}

	void segment_region_growing_pssnet
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_in,
		std::vector<superfacets> &segment_merged
	)
	{
		//get_current segment information
#pragma omp parallel for schedule(dynamic)
		for (int spf_i = 0; spf_i < segment_in.size(); ++spf_i)
		{
			segment_in[spf_i].sum_area = 0.0f;
			segment_in[spf_i].seg_visited = false;
			std::map<SFMesh::Vertex, bool> visited_vert;
			bool has_face_has_color_boder = false;
			for (int fi = 0; fi < segment_in[spf_i].face_vec.size(); ++fi)
			{
				for (auto vdx : smesh_in->vertices(segment_in[spf_i].face_vec[fi]))
				{
					if (visited_vert.find(vdx) == visited_vert.end())
					{
						visited_vert[vdx] = false;
						vec3 p_temp = smesh_in->get_points_coord[vdx];
						segment_in[spf_i].ExactVec_CGAL.emplace_back(p_temp.x, p_temp.y, p_temp.z);
					}
				}
				segment_in[spf_i].sum_area += smesh_in->get_face_area[segment_in[spf_i].face_vec[fi]];
			}

			Plane plane_c;
			CGAL::linear_least_squares_fitting_3(segment_in[spf_i].ExactVec_CGAL.begin(), segment_in[spf_i].ExactVec_CGAL.end(), plane_c, CGAL::Dimension_tag<0>());
			easy3d::vec4 plane_param(plane_c.a(), plane_c.b(), plane_c.c(), -plane_c.d());
			segment_in[spf_i].plane_parameter = plane_param;
		}

		std::vector<superfacets> segment_sorted;
		segment_sorted.insert(segment_sorted.end(), segment_in.begin(), segment_in.end());
		sort(segment_sorted.begin(), segment_sorted.end(), smaller_segments_area);

		//segment region growing
		int prev_percent = -1;
		for (int spf_soi = 0; spf_soi < segment_sorted.size(); ++spf_soi)
		{
			int spf_i = segment_sorted[spf_soi].id;
			int progress = int(100.0f * (spf_soi + 1) / float(segment_sorted.size()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			if (!segment_in[spf_i].seg_visited)
			{
				segment_in[spf_i].seg_visited = true;
				segment_merged.push_back(segment_in[spf_i]);
				std::vector<superfacets> current_seeds;
				current_seeds.push_back(segment_in[spf_i]);

				std::vector<int> majority_labels(labels_name_pnp.size(), 0);
				segment_merged[segment_merged.size() - 1].label = segment_in[spf_i].label;
				if (segment_in[spf_i].label > -1)
					++majority_labels[segment_in[spf_i].label];
				if (train_test_predict_val == 0)
					segment_merged[segment_merged.size() - 1].ground_truth = segment_in[spf_i].ground_truth;
				else
					segment_merged[segment_merged.size() - 1].predict = segment_in[spf_i].predict;

				std::vector<std::string> spf_i_label_names;
				for (int spf_ci = 0; spf_ci < current_seeds.size(); ++spf_ci)
				{
					vec3 cur_n(current_seeds[spf_ci].plane_parameter.x, current_seeds[spf_ci].plane_parameter.y, current_seeds[spf_ci].plane_parameter.z);
					for (int spf_ni = 0; spf_ni < current_seeds[spf_ci].neighbor_ids.size(); ++spf_ni)
					{
						int neg_ind = current_seeds[spf_ci].neighbor_ids[spf_ni];

						if (!segment_in[neg_ind].seg_visited)
						{
							//distance only
							float dis2plane_avg = 0.0f;
#pragma omp parallel for reduction(+:dis2plane_avg)
							for (int pi = 0; pi < segment_in[neg_ind].ExactVec_CGAL.size(); ++pi)
							{
								dis2plane_avg += dist2plane(segment_merged[segment_merged.size() - 1].plane_parameter, segment_in[neg_ind].ExactVec_CGAL[pi]);
							}
							dis2plane_avg /= segment_in[neg_ind].ExactVec_CGAL.size();

							//angle
							vec3 neg_n(segment_in[neg_ind].plane_parameter.x, segment_in[neg_ind].plane_parameter.y, segment_in[neg_ind].plane_parameter.z);
							float ang_tmp = vector3D_angle(cur_n, neg_n);
							ang_tmp = ang_tmp > 90.0f ? 180.0f - ang_tmp : ang_tmp;

							float cmp_area = segment_in[neg_ind].sum_area;

							bool is_ok = false;
							if (dis2plane_avg <= adjacent_pt2plane_distance
								&& ang_tmp <= adjacent_seg_angle)
								is_ok = true;

							if (labels_name_pnp.size() > 2)
							{
								if (current_seeds[spf_ci].label != segment_in[neg_ind].label)
									is_ok = false;
							}

							if (is_ok)
							{
								segment_in[neg_ind].seg_visited = true;
								current_seeds.push_back(segment_in[neg_ind]);
								segment_merged[segment_merged.size() - 1].face_vec.insert
								(
									segment_merged[segment_merged.size() - 1].face_vec.end(),
									segment_in[neg_ind].face_vec.begin(),
									segment_in[neg_ind].face_vec.end()
								);

								segment_merged[segment_merged.size() - 1].ExactVec_CGAL.insert
								(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(),
									segment_in[neg_ind].ExactVec_CGAL.begin(),
									segment_in[neg_ind].ExactVec_CGAL.end()
								);
								segment_merged[segment_merged.size() - 1].sum_area += segment_in[neg_ind].sum_area;
								//update plane parameters
								Plane plcgal;
								CGAL::linear_least_squares_fitting_3(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.begin(),
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>()
								);
								segment_merged[segment_merged.size() - 1].plane_parameter.x = plcgal.a();
								segment_merged[segment_merged.size() - 1].plane_parameter.y = plcgal.b();
								segment_merged[segment_merged.size() - 1].plane_parameter.z = plcgal.c();
								segment_merged[segment_merged.size() - 1].plane_parameter.w = -plcgal.d();
								if (segment_in[neg_ind].label > -1)
									++majority_labels[segment_in[neg_ind].label];
							}
						}
					}
				}
				segment_merged[segment_merged.size() - 1].label = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			}
		}

		//assign the new segment to face 
#pragma omp parallel for schedule(dynamic)
		for (int spf_i = 0; spf_i < segment_merged.size(); ++spf_i)
		{
			for (int fi = 0; fi < segment_merged[spf_i].face_vec.size(); ++fi)
			{
				smesh_in->get_face_segment_id[segment_merged[spf_i].face_vec[fi]] = spf_i;
			}
		}
	}

	void merge_single_segments
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_in,
		std::vector<superfacets> &segment_merged
	)
	{
		std::vector<superfacets> segment_sorted;
		segment_sorted.insert(segment_sorted.end(), segment_in.begin(), segment_in.end());
		sort(segment_sorted.begin(), segment_sorted.end(), larger_segments_area);

		int prev_percent = -1;
		for (int spf_soi = 0; spf_soi < segment_sorted.size(); ++spf_soi)
		{
			int spf_i = segment_sorted[spf_soi].id;
			int progress = int(100.0f * (spf_soi + 1) / float(segment_sorted.size()));
			if (progress != prev_percent)
			{
				printf("%3d%%\b\b\b\b", progress);
				prev_percent = progress;
			}

			if (!segment_in[spf_i].seg_visited)
			{
				segment_in[spf_i].seg_visited = true;
				segment_merged.push_back(segment_in[spf_i]);
				std::vector<superfacets> current_seeds;
				current_seeds.push_back(segment_in[spf_i]);

				std::vector<int> majority_labels(labels_name_pnp.size(), 0);
				segment_merged[segment_merged.size() - 1].label = segment_in[spf_i].label;
				++majority_labels[segment_in[spf_i].label];

				for (int spf_ci = 0; spf_ci < current_seeds.size(); ++spf_ci)
				{
					vec3 cur_n(current_seeds[spf_ci].plane_parameter.x, current_seeds[spf_ci].plane_parameter.y, current_seeds[spf_ci].plane_parameter.z);
					for (int spf_ni = 0; spf_ni < current_seeds[spf_ci].neighbor_ids.size(); ++spf_ni)
					{
						int neg_ind = current_seeds[spf_ci].neighbor_ids[spf_ni];
						if (segment_in[neg_ind].label == 1)
							continue;

						if (!segment_in[neg_ind].seg_visited)
						{
							bool is_ok = false;
							if (segment_in[neg_ind].neighbor_ids.size() <= 1
								|| segment_in[neg_ind].face_vec.size() == 1)
								is_ok = true;

							if (labels_name_pnp.size() > 2)
							{
								if (current_seeds[spf_ci].label != segment_in[neg_ind].label)
									is_ok = false;
							}

							if (is_ok)
							{
								segment_in[neg_ind].seg_visited = true;
								current_seeds.push_back(segment_in[neg_ind]);
								segment_merged[segment_merged.size() - 1].face_vec.insert
								(
									segment_merged[segment_merged.size() - 1].face_vec.end(),
									segment_in[neg_ind].face_vec.begin(),
									segment_in[neg_ind].face_vec.end()
								);

								segment_merged[segment_merged.size() - 1].ExactVec_CGAL.insert
								(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(),
									segment_in[neg_ind].ExactVec_CGAL.begin(),
									segment_in[neg_ind].ExactVec_CGAL.end()
								);
								segment_merged[segment_merged.size() - 1].sum_area += segment_in[neg_ind].sum_area;
								//update plane parameters
								Plane plcgal;
								CGAL::linear_least_squares_fitting_3(
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.begin(),
									segment_merged[segment_merged.size() - 1].ExactVec_CGAL.end(), plcgal, CGAL::Dimension_tag<0>()
								);
								segment_merged[segment_merged.size() - 1].plane_parameter.x = plcgal.a();
								segment_merged[segment_merged.size() - 1].plane_parameter.y = plcgal.b();
								segment_merged[segment_merged.size() - 1].plane_parameter.z = plcgal.c();
								segment_merged[segment_merged.size() - 1].plane_parameter.w = -plcgal.d();

								++majority_labels[segment_in[neg_ind].label];
							}
						}
					}
				}

				segment_merged[segment_merged.size() - 1].label = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			}
		}

		//assign the new segment to face 
		for (int spf_i = 0; spf_i < segment_merged.size(); ++spf_i)
		{
			segment_merged[spf_i].id = spf_i;
			segment_merged[spf_i].neighbor_ids.clear();
			segment_merged[spf_i].sum_area = 0.0f;
			for (int fi = 0; fi < segment_merged[spf_i].face_vec.size(); ++fi)
			{
				segment_merged[spf_i].sum_area += smesh_in->get_face_area[segment_merged[spf_i].face_vec[fi]];
				smesh_in->get_face_segment_id[segment_merged[spf_i].face_vec[fi]] = spf_i;
			}
		}
	}

	void merge_single_triangles
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp_vec
	)
	{
		for (auto spf : spf_temp_vec)
		{
			for (auto fd : spf.face_vec)
			{
				int current_seg_id = smesh_out->get_face_segment_id[fd];
				std::map<int, int> segid_count;

				SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_out->halfedges(fd);
				SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
				do {
					SFMesh::Halfedge ho = smesh_out->opposite_halfedge(*h_fit);
					if (smesh_out->is_boundary(ho) == false)
					{
						SFMesh::Edge eo = smesh_out->edge(ho);
						SFMesh::Vertex vs = smesh_out->vertex(eo, 0);
						SFMesh::Vertex vt = smesh_out->vertex(eo, 1);
						SFMesh::Vertex non_vert;

						SFMesh::Face f_neg = smesh_out->face(ho);

						int neg_seg_id = smesh_out->get_face_segment_id[f_neg];
						if (neg_seg_id != current_seg_id)
						{
							if (segid_count.find(neg_seg_id) == segid_count.end())
							{
								segid_count[neg_seg_id] = 1;
							}
							else
							{
								segid_count[neg_seg_id] += 1;
							}
						}
					}
					++h_fit;
				} while (h_fit != h_end);

				std::pair<int, int> maxseg_count(-1, -INT_MAX);
				for (auto seg_neg : segid_count)
				{
					if (seg_neg.second > maxseg_count.second)
					{
						maxseg_count.first = seg_neg.first;
						maxseg_count.second = seg_neg.second;
					}
				}
				if (maxseg_count.second >= 2)
				{
					smesh_out->get_face_segment_id[fd] = maxseg_count.first;
				}

			}
		}

		//re-assign the new superfaces
		std::vector<superfacets> new_spf_temp_vec;
		std::map<int, bool> spf_build;
		std::map<int, int> spf_old_new;
		for (auto fd : smesh_out->faces())
		{
			smesh_out->get_face_check[fd] = false;
			int seg_id = smesh_out->get_face_segment_id[fd];
			if (spf_build.find(seg_id) == spf_build.end())
			{
				spf_build[seg_id] = true;
				superfacets spf_new;
				spf_new.label = spf_temp_vec[seg_id].label;
				spf_new.face_vec.push_back(fd);
				new_spf_temp_vec.push_back(spf_new);
				spf_old_new[seg_id] = new_spf_temp_vec.size() - 1;
			}
			else
			{
				new_spf_temp_vec[spf_old_new[seg_id]].face_vec.push_back(fd);
			}
		}
		spf_temp_vec.clear();
		spf_temp_vec = new_spf_temp_vec;
	}

	void rearrange_connected_segments
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_out
	)
	{
		for (auto &fd : smesh_in->faces())
		{
			if (smesh_in->get_face_check[fd])
				continue;
			smesh_in->get_face_check[fd] = true;
			std::vector<SFMesh::Face> current_region, current_seeds;
			current_seeds.emplace_back(fd);
			current_region.emplace_back(fd);
			std::vector<int> majority_labels(labels_name_pnp.size(), 0);
			if (train_test_predict_val == 0)
			{
				if (smesh_in->get_face_truth_label[fd] > 0)
				{
					++majority_labels[smesh_in->get_face_truth_label[fd] - 1];
				}
			}
			else
			{
				if (smesh_in->get_face_predict_label[fd] != -1)
				{
					++majority_labels[smesh_in->get_face_predict_label[fd] - 1];
				}
			}

			for (int ci = 0; ci < current_seeds.size(); ++ci)
			{
				SFMesh::HalfedgeAroundFaceCirculator h_fit = smesh_in->halfedges(current_seeds[ci]);
				SFMesh::HalfedgeAroundFaceCirculator h_end = h_fit;
				do
				{
					SFMesh::Halfedge ho = smesh_in->opposite_halfedge(*h_fit);
					if (smesh_in->is_boundary(ho) == false)
					{
						//SFMesh::Edge eo = smesh_in->edge(ho);
						SFMesh::Face f_neg = smesh_in->face(ho);
						if (smesh_in->get_face_check[f_neg])
						{
							++h_fit;
							continue;
						}

						if (smesh_in->get_face_segment_id[current_seeds[ci]] == smesh_in->get_face_segment_id[f_neg])
						{
							current_seeds.emplace_back(f_neg);
							current_region.emplace_back(f_neg);
							smesh_in->get_face_check[f_neg] = true;

							if (train_test_predict_val == 0)
							{
								if (smesh_in->get_face_truth_label[f_neg] > 0)
								{
									++majority_labels[smesh_in->get_face_truth_label[f_neg] - 1];
								}
							}
							else
							{
								if (smesh_in->get_face_predict_label[f_neg] != -1)
								{
									++majority_labels[smesh_in->get_face_predict_label[f_neg] - 1];
								}
							}
						}
					}
					++h_fit;
				} while (h_fit != h_end);
			}

			superfacets temp;
			temp.face_vec.insert(temp.face_vec.end(), current_region.begin(), current_region.end());
			int maxElementIndex = std::max_element(majority_labels.begin(), majority_labels.end()) - majority_labels.begin();
			temp.label = maxElementIndex;
			segment_out.emplace_back(temp);

		}

		for (int spf_i = 0; spf_i < segment_out.size(); ++spf_i)
		{
			segment_out[spf_i].id = spf_i;
			segment_out[spf_i].neighbor_ids.clear();
			segment_out[spf_i].sum_area = 0.0f;
			for (int fi = 0; fi < segment_out[spf_i].face_vec.size(); ++fi)
			{
				segment_out[spf_i].sum_area += smesh_in->get_face_area[segment_out[spf_i].face_vec[fi]];
				smesh_in->get_face_segment_id[segment_out[spf_i].face_vec[fi]] = spf_i;
			}
		}
	}

	void merge_planar_segments
	(
		SFMesh *smesh_in,
		std::vector<superfacets> &segment_in,
		std::vector<superfacets> &segment_merged
	)
	{
		std::map<int, std::vector<int>> seg_neighbors;
		std::vector<superfacets> segment_intermediate;
		planar_segment_merge_preprocessing(smesh_in, segment_in, seg_neighbors);

		finding_adjacent_segments_based_on_connectivity
		(
			smesh_in,
			segment_in,
			seg_neighbors
		);

		segment_region_growing_pssnet(smesh_in, segment_in, segment_intermediate);

		//for new segments
		const int previous_seg_num = segment_intermediate.size();
		int count = 0, current_changes = 0, previous_changes;
		do
		{
			previous_changes = current_changes;

			if (!segment_merged.empty())
			{
				segment_intermediate.clear();
				segment_intermediate.insert(segment_intermediate.end(), segment_merged.begin(), segment_merged.end());
			}

			seg_neighbors.clear();
			segment_merged.clear();
			planar_segment_merge_preprocessing(smesh_in, segment_intermediate, seg_neighbors);

			finding_adjacent_segments_based_on_connectivity
			(
				smesh_in,
				segment_intermediate,
				seg_neighbors
			);

			//merge isolated segments
			merge_single_segments(smesh_in, segment_intermediate, segment_merged);

			//merge single triangles
			merge_single_triangles(smesh_in, segment_merged);

			//separate connected components of segments
			segment_merged.clear();
			rearrange_connected_segments(smesh_in, segment_merged);

			const int tmp_size = segment_merged.size();
			current_changes = tmp_size;
			++count;
		} while (current_changes != previous_changes);
	}

	//assign initial planar segments reassignments
	void assign_initial_segments_on_mesh
	(
		SFMesh *smesh_out,
		std::vector<superfacets> &spf_temp,
		std::vector<superfacets> &spf_planar_seg
	)
	{
		std::cout << "  assign_initial_segments_on_mesh, use ";
		const double t_total = omp_get_wtime();

		//rearrange segments with new id and planar param
		rearrange_segments(smesh_out, spf_temp, spf_planar_seg);

		int new_id = spf_planar_seg.size();
		//smooth faces by using pairwise term
		face_global_smooth_on_mesh(smesh_out);

		//parsing to new water shed
		std::vector<superfacets> spf_temp_vec;
		parsing_with_planar_region(smesh_out, spf_temp_vec, new_id);
		spf_planar_seg.clear();

		//detect connected segments
		merge_planar_segments(smesh_out, spf_temp_vec, spf_planar_seg);

		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
		std::cout << " After region growing, the segment number is " << spf_planar_seg.size() << std::endl;
	}

	//Over-segmentation pipeline
	void perform_pnpmrf_oversegmentation_on_mesh
	(
		SFMesh *smesh_out,
		std::vector<superfacets>& spf_result,
		std::vector<vertex_planarity> &mesh_vertex_planarity
	)
	{
		std::vector<superfacets> spf_temp, spf_others;
		//1. Extract small connected components
		int component_num = 0;
		int largest_seg_id = separate_connected_components(smesh_out, spf_others, component_num);

		//2. MRF Plane fitting based region growing
		int segment_id = 0;
		std::cout << "  Region growing, use ";
		const double t_total = omp_get_wtime();
		sort(mesh_vertex_planarity.begin(), mesh_vertex_planarity.end(), larger_planarity<vertex_planarity>);
		//for (int ccv_id = 0; ccv_id <= component_num; ++ccv_id)
		//	region_growing_on_mesh(smesh_out, spf_temp, mesh_vertex_planarity, ccv_id, segment_id);
		for (int ccv_id = 0; ccv_id <= component_num; ++ccv_id)
			region_growing_on_mesh_with_all_neighbors(smesh_out, spf_temp, mesh_vertex_planarity, ccv_id, segment_id);
		std::cout << " (s): " << omp_get_wtime() - t_total << '\n' << std::endl;
		std::cout << "The number of intermediate segments are " << spf_temp.size() << std::endl;

		//3. Assign initial planar segments
		assign_initial_segments_on_mesh(smesh_out, spf_temp, spf_result);
		std::cout << "The number of segments are " << spf_result.size() << std::endl;
	}
}
