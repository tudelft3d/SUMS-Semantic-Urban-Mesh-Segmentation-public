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
}
#endif//semantic_mesh_segmentation__MESH_CLASSIFIER_HPP
