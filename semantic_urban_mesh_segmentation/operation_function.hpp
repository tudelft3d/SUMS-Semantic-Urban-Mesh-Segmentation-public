/*
*   Name        : operation_function.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for run different operation modes
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
#ifndef semantic_mesh_segmentation__OPERATION_FUNCTION_HPP
#define semantic_mesh_segmentation__OPERATION_FUNCTION_HPP

#include <omp.h>
#include <easy3d/point_cloud_io.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include "mesh_processor.hpp"

namespace semantic_mesh_segmentation
{
	inline void GCN_feature_write_configurations()
	{
		processing_mode = 0;
		use_existing_mesh_segments = true;
		is_pointclouds_exist = true;
		use_GCN_features = true;

		use_feas[0] = true;
		use_feas[1] = true;
		use_feas[2] = true;
		use_feas[3] = false;

		use_basic_features[0] = false;
		use_basic_features[2] = false;
		use_basic_features[3] = false;

		use_mulsc_eles[0] = false;
		use_mulsc_eles[1] = false;
		use_mulsc_eles[2] = false;

		use_basic_features[0] = false;  //avg_center_z
		use_basic_features[1] = true;  //interior_mat_radius
		use_basic_features[2] = false;  //sum_area
		use_basic_features[3] = false;  //relative_elevation
		use_basic_features[4] = true;  //triangle density (only useful when input is adaptive mesh)
		use_basic_features[5] = true;  //vertex_count
		use_basic_features[6] = true;  //points_to_plane_dist_mean
		use_basic_features[7] = true;  //compactness
		use_basic_features[8] = true;  //shape_index
		use_basic_features[9] = true;  //shape_descriptor

		use_eigen_features[0] = false;  //eigen_1
		use_eigen_features[1] = false;  //eigen_2
		use_eigen_features[2] = false;  //eigen_3
		use_eigen_features[3] = true;   //verticality
		use_eigen_features[4] = true;   //linearity
		use_eigen_features[5] = true;  //planarity
		use_eigen_features[6] = true;   //sphericity
		use_eigen_features[7] = false;  //anisotropy
		use_eigen_features[8] = false;  //eigenentropy
		use_eigen_features[9] = false;  //omnivariance
		use_eigen_features[10] = false; //sumeigenvals
		use_eigen_features[11] = true;  //curvature
		use_eigen_features[12] = false; //verticality_eig1
		use_eigen_features[13] = false; //verticality_eig3
		use_eigen_features[14] = false; //surface
		use_eigen_features[15] = false; //volume
		use_eigen_features[16] = false; //absolute_eigvec_1_moment_1st
		use_eigen_features[17] = false; //absolute_eigvec_2_moment_1st
		use_eigen_features[18] = false; //absolute_eigvec_3_moment_1st
		use_eigen_features[19] = false; //absolute_eigvec_1_moment_2nd
		use_eigen_features[20] = false; //absolute_eigvec_2_moment_2nd
		use_eigen_features[21] = false; //absolute_eigvec_3_moment_2nd
		use_eigen_features[22] = false; //vertical_moment_1st
		use_eigen_features[23] = false; //vertical_moment_2nd
		use_eigen_features[24] = false; //uniformity

		use_color_features[0] = true;//red
		use_color_features[1] = true;//green
		use_color_features[2] = true;//blue
		use_color_features[3] = true;//hue
		use_color_features[4] = true;//sat
		use_color_features[5] = true;//val
		use_color_features[6] = true;//greenness
		use_color_features[7] = false;//red_var
		use_color_features[8] = false;//green_var
		use_color_features[9] = false;//blue_var
		use_color_features[10] = true;//hue_var
		use_color_features[11] = true;//sat_var
		use_color_features[12] = true;//val_var
		use_color_features[13] = false;//greenness_var
		use_color_features[14] = true;//hue_bin_0
		use_color_features[15] = true;//hue_bin_1
		use_color_features[16] = true;//hue_bin_2
		use_color_features[17] = true;//hue_bin_3
		use_color_features[18] = true;//hue_bin_4
		use_color_features[19] = true;//hue_bin_5
		use_color_features[20] = true;//hue_bin_6
		use_color_features[21] = true;//hue_bin_7
		use_color_features[22] = true;//hue_bin_8
		use_color_features[23] = true;//hue_bin_9
		use_color_features[24] = true;//hue_bin_10
		use_color_features[25] = true;//hue_bin_11
		use_color_features[26] = true;//hue_bin_12
		use_color_features[27] = true;//hue_bin_13
		use_color_features[28] = true;//hue_bin_14
		use_color_features[29] = true;//sat_bin_0
		use_color_features[30] = true;//sat_bin_1
		use_color_features[31] = true;//sat_bin_2
		use_color_features[32] = true;//sat_bin_3
		use_color_features[33] = true;//sat_bin_4
		use_color_features[34] = true;//val_bin_0
		use_color_features[35] = true;//val_bin_1
		use_color_features[36] = true;//val_bin_2
		use_color_features[37] = true;//val_bin_3
		use_color_features[38] = true;//val_bin_4
	}

    void run(const operating_mode&);
	void changing_to_test_or_predict(const int);
}
#endif