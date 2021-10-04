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
}
