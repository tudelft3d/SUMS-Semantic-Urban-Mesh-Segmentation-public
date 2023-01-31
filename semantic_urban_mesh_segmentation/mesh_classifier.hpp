/*
*   Name        : mesh_classifier.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : classification of input features, including random forest classifier and MRF, and results evaluation.
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
#ifndef semantic_mesh_segmentation__MESH_CLASSIFIER_HPP
#define semantic_mesh_segmentation__MESH_CLASSIFIER_HPP

#define COMPILER_MSVC
#define NOMINMAX

#include <omp.h>
#include <typeinfo>
#include <easy3d/surface_mesh.h>
#include <easy3d/kdtree.h>
#include <easy3d/point_cloud.h>
#include <3rd_party/gco/GCoptimization.h>
#include "mesh_io.hpp"
#include "parameters.hpp"
#include "CGAL_Define.hpp"

namespace semantic_mesh_segmentation
{
	struct all_eval
	{
		std::vector<float> label_asa = std::vector<float>(labels_name_pnp.size(), 0.0f);
		std::vector<float> label_ue = std::vector<float>(labels_name_pnp.size(), 0.0f);
		std::vector<float> label_sumarea = std::vector<float>(labels_name_pnp.size(), 0.0f);
		std::vector<float> asa_out = std::vector<float>(labels_name_pnp.size() + 1, 0.0f);
		std::vector<float> segment_count_gctc = std::vector<float>(3, 0.0f);
		std::vector<float> boundary_evaluation = std::vector<float>(5, 0.0f);
		all_eval() {};
	};

	inline void compute_asa
	(
		all_eval *all_seg_evaluation = nullptr,
		std::vector<float> &asa_out = std::vector<float>(),
		std::map<int, float> &label_asa = std::map<int, float>(),
		std::map<int, float> &label_ue = std::map<int, float>(),
		std::map<int, float> &label_sumarea = std::map<int, float>()
	)
	{
		float avg_asa = 0.0f, avg_ue = 0.0f;
		for (int li = 0; li < labels_name_pnp.size(); ++li)
		{
			if (!asa_out.empty())
			{
				//accumulate all
				if (all_seg_evaluation != nullptr)
				{
					all_seg_evaluation->label_asa[li] += label_asa[li];
					all_seg_evaluation->label_ue[li] += label_ue[li];
					all_seg_evaluation->label_sumarea[li] += label_sumarea[li];
				}

				//compute individual
				if (label_sumarea[li] != 0)
				{
					label_asa[li] /= label_sumarea[li];
					label_ue[li] /= label_sumarea[li];
				}
				else
				{
					label_asa[li] = 0.0f;
					label_ue[li] = 0.0f;
				}
				avg_asa += label_asa[li];
				avg_ue += label_ue[li];
				std::cout << "asa(" << labels_name_pnp[li] << ") = " << label_asa[li]
					//<< ";\tue(" << labels_name_pnp[li] << ") = " << label_ue[li]
					/*<< ";\ttruth area(" << labels_name_pnp[li] << ") = " << label_sumarea[li]*/ << std::endl;
				asa_out[li] = label_asa[li];
			}
			else
			{
				if (all_seg_evaluation->label_sumarea[li] != 0)
				{
					all_seg_evaluation->label_asa[li] /= all_seg_evaluation->label_sumarea[li];
					all_seg_evaluation->label_ue[li] /= all_seg_evaluation->label_sumarea[li];
				}
				else
				{
					all_seg_evaluation->label_asa[li] = 0.0f;
					all_seg_evaluation->label_ue[li] = 0.0f;
				}
				avg_asa += all_seg_evaluation->label_asa[li];
				avg_ue += all_seg_evaluation->label_ue[li];
				std::cout << "asa(" << labels_name_pnp[li] << ") = " << all_seg_evaluation->label_asa[li]
					//<< ";\tue(" << labels_name_pnp[li] << ") = " << label_ue[li]
					/*<< ";\ttruth area(" << labels_name_pnp[li] << ") = " << label_sumarea[li]*/ << std::endl;
				all_seg_evaluation->asa_out[li] = all_seg_evaluation->label_asa[li];

			}
		}
		if (!asa_out.empty())
		{
			asa_out[labels_name_pnp.size()] = avg_asa / labels_name_pnp.size();
			std::cout << "avg_ASA = " << avg_asa / labels_name_pnp.size() << std::endl;
			//std::cout << "avg_ue = " << avg_ue / labels_name_pnp.size() << std::endl;
		}
		else
		{
			all_seg_evaluation->asa_out[labels_name_pnp.size()] = avg_asa / labels_name_pnp.size();
			std::cout << "avg_ASA = " << avg_asa / labels_name_pnp.size() << std::endl;
			//std::cout << "avg_ue = " << avg_ue / labels_name_pnp.size() << std::endl;
		}
	}

	inline void compute_gc_tc
	(
		int &test_count,
		int &ground_count,
		std::vector<float> &gc_tc_out = std::vector<float>(),
		all_eval *all_seg_evaluation = nullptr
	)
	{
		if (!gc_tc_out.empty())
		{
			gc_tc_out[0] = test_count;
			gc_tc_out[1] = ground_count;
			gc_tc_out[2] = float(ground_count) / float(test_count);
		}

		//accumulate all
		if (all_seg_evaluation != nullptr)
		{
			all_seg_evaluation->segment_count_gctc[0] += test_count;
			all_seg_evaluation->segment_count_gctc[1] += ground_count;
			all_seg_evaluation->segment_count_gctc[2] = float(all_seg_evaluation->segment_count_gctc[0]) / float(all_seg_evaluation->segment_count_gctc[1]);
		}

		std::cout << "number of segments(test) = " << test_count << std::endl <<
			"number of segments(ground truth) = " << ground_count << std::endl <<
			std::fixed << std::showpoint << std::setprecision(6) <<
			"g_c / t_c = " << float(ground_count) / float(test_count) << std::endl << std::endl;
	}


	inline void compute_br
	(
		float &boundary_num_g,
		float &boundary_num_t,
		float &intersect_edges,
		std::vector<float> &br_out = std::vector<float>(),
		all_eval *all_seg_evaluation = nullptr
	)
	{
		if (!br_out.empty())
		{
			br_out[0] = boundary_num_g;
			br_out[1] = boundary_num_t;
			br_out[2] = intersect_edges;
			br_out[3] = float(intersect_edges) / float(boundary_num_g);
			br_out[4] = float(intersect_edges) / float(boundary_num_t);
		}

		//accumulate all
		if (all_seg_evaluation != nullptr)
		{
			all_seg_evaluation->boundary_evaluation[0] += boundary_num_g;
			all_seg_evaluation->boundary_evaluation[1] += boundary_num_t;
			all_seg_evaluation->boundary_evaluation[2] += intersect_edges;
			all_seg_evaluation->boundary_evaluation[3] = float(all_seg_evaluation->boundary_evaluation[2]) / float(all_seg_evaluation->boundary_evaluation[0]);
			all_seg_evaluation->boundary_evaluation[4] = float(all_seg_evaluation->boundary_evaluation[2]) / float(all_seg_evaluation->boundary_evaluation[1]);

			std::cout << "Ground truth edges = " << boundary_num_g << std::endl <<
				";Predicted edges = " << boundary_num_t << std::endl <<
				";Intersected edges = " << intersect_edges << std::endl <<
				"BR = " << float(intersect_edges) / float(boundary_num_g) <<
				";\tBP = " << float(intersect_edges) / float(boundary_num_t) << std::endl;
		}
	}

	//--- bool comparision ---
	inline bool feature_importance_descending
	(
		std::pair<int, int> &fea_1,
		std::pair<int, int> &fea_2
	)
	{
		if (fea_1.second >= fea_2.second)
			return true;
		else
			return false;
	}

	inline void unary_accum
	(
		std::vector<int> &predict_label,
		std::vector<std::vector<float>> &predict_prob_all,
		std::map<int, std::tuple<int, int, std::vector<float>>> &unary_label_prob,
		const int seg_join_i,
		const int ind
	)
	{
		if (unary_label_prob.find(ind) == unary_label_prob.end())
			unary_label_prob[ind] = std::make_tuple(predict_label[seg_join_i], 1, std::vector<float>(labels_name.size(), 0.0f));
		else
			get<1>(unary_label_prob[ind]) += 1;

		for (int li = 0; li < labels_name.size(); ++li)
			get<2>(unary_label_prob[ind])[li] += predict_prob_all[seg_join_i][li];
	}

	//--- compute variances ---
	inline float compute_variance_of_input
	(
		std::vector< float > &fea
	)
	{
		std::pair<float, float> mean_sd(0.0f, 0.0f);
		for (int i = 0; i < fea.size(); ++i)
			mean_sd.first += fea[i];

		mean_sd.first /= float(fea.size());
		if (!std::isnormal(mean_sd.first))
			mean_sd.first = default_feature_value_minmax.first;

		for (int i = 0; i < fea.size(); ++i)
			mean_sd.second += std::pow((fea[i] - mean_sd.first), 2.0f);

		mean_sd.second = mean_sd.second / float(fea.size());
		if (!std::isnormal(mean_sd.second))
			mean_sd.second = default_feature_value_minmax.first;

		return mean_sd.second;
	}

	//--- function declaration ---
	void add_labels
	(
		Label_set &labels,
		bool enable_double_lables = false
	);

	void add_feature_names
	(
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		Feature_set &,
		int &,
		std::vector<std::string> &
	);

	void add_joint_feature_names
	(
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		Feature_set &,
		int &,
		std::vector<std::string> &
	);

	void ETH_RF_Train_Base
	(
		std::vector<Cluster_point> &,
		std::vector<int> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		Label_set &,
		Feature_set &,
		CGAL::Classification::ETHZ::Random_forest_classifier &
	);

	void ETH_RF_Test_Base
	(
		easy3d::PointCloud *,
		std::vector<Cluster_point> &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		Label_set &,
		Feature_set &,
		CGAL::Classification::ETHZ::Random_forest_classifier &,
		std::vector<int> &,
		std::vector<float> &,
		std::vector<std::vector<float>> &
	);

	void ETH_RF_Test_with_Trained_Model
	(
		easy3d::PointCloud *,
		std::vector<Cluster_point> &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector< std::vector<float>> &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		Label_set &,
		std::vector<int> &,
		std::vector<float> &,
		std::vector<std::vector<float>> &,
		Feature_set &,
		CGAL::Classification::ETHZ::Random_forest_classifier &
	);

	void ETH_RF_SavingTest_and_or_Evaluation
	(
		SFMesh *,
		Label_set &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector<float> &,
		const int m
	);

	void evaluation_all_test_data
	(
		Label_set &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector<float> &,
		const int m = -1
	);

	void joint_labeling_energy
	(
		std::map<std::pair<int, int>, std::pair<float, bool>> &,
		std::vector<int> &,
		std::vector<float> &,
		std::vector<std::vector<float>> &,
		std::map<int, std::tuple<int, int, std::vector<float>>> &,
		std::map<std::pair<int, int>, float> &
	);

	void MRF_joint_labeling
	(
		SFMesh*,
		std::vector< std::vector<int>> &,
		std::map<int, std::tuple<int, int, std::vector<float>>> &,//sites, (count joint labels, accum labels_probs)
		std::map<std::pair<int, int>, float> &, //neighbors, prob
		const int
	);

	float feature_variances_computation
	(
		std::vector<int> &,
		std::vector< std::vector<float>> & ,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &,
		std::vector< std::vector<float> > &
	);

	void parsing_properties_from_classification
	(
		SFMesh *,
		std::vector< std::vector<int>> &,
		std::vector<int> &,
		std::vector<int> &,
		std::vector<float> &,
		std::vector<int> &,
		std::vector<float> &,
		std::vector<std::vector<float>> &
	);

	//--- PSSNet graph-cut module ---
	void MRF_oversegmentation
	(
		std::vector<float> &,
		std::vector<std::pair<int, int>> &,
		std::vector<float> &,
		std::vector<int> &
	);

	void MRF_oversegmentation
	(
		SFMesh *,
		SFMesh::Face &,
		std::vector<int> &,
		const int ,
		int &
	);

	void segment_purity_evaluation(SFMesh *, SFMesh *, std::map<int, float> &, std::map<int, float> &, std::map<int, float> &, std::vector<int> &, std::vector<int> &);

	void boundary_evaluation(SFMesh *, SFMesh *, float &, float &, float &, int &);

	void oversegmentation_evaluation(SFMesh *, SFMesh *, const int, all_eval *);
}
#endif//semantic_mesh_segmentation__MESH_CLASSIFIER_HPP
