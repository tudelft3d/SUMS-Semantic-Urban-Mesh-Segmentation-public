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


#ifndef EASY3D_KD_TREE_H
#define EASY3D_KD_TREE_H

#include <easy3d/types.h>
#include <easy3d/point_cloud.h>
#include <vector>
#include <map>
#include <set>
#include "semantic_urban_mesh_segmentation/PTCloud.hpp"

namespace easy3d {

    class PointCloud;

    class KdTree  {
    public:
        KdTree();
        virtual ~KdTree();

        //______________ tree construction __________________________

        // call the following functions to build a kd-tree of a point cloud.
        virtual void begin() ;
        virtual void add_point_cloud(PointCloud* cloud) ;
        virtual void end() ;    // now your kd-tree is ready.

        //________________ closest point ____________________________

        // find the closest point of p in the point cloud.
        // return the index of the found point.
        virtual int find_closest_point(const vec3& p) const;

        // the same as the previous one, but it also returns its squared_distance to the query point.
        virtual int find_closest_point(const vec3& p, float& squared_distance) const;


        //_________________ K-nearest neighbors ____________________

        // find closest K points of p in the point cloud.
        // return the indices of the found points in 'neighbors'.
        virtual void find_closest_K_points(
            const vec3& p, int k,
            std::vector<int>& neighbors
            ) const ;

        // the same as the previous one, but it also returns their squared_distances to the query point.
        virtual void find_closest_K_points(
            const vec3& p, int k,
            std::vector<int>& neighbors, std::vector<float>& squared_distances
            ) const ;


        //___________________ fixed-radius search ___________________________

        // search for all points within the 'radius' range.
        // return the indices of the found points in 'neighbors'.
        virtual void find_points_in_radius(
                const vec3& p, float radius,
                std::vector<int>& neighbors
                ) const;

        // the same as the previous one, but it also returns their squared_distances to the query point.
        virtual void find_points_in_radius(
                const vec3& p, float radius,
                std::vector<int>& neighbors,
                std::vector<float>& squared_distances
                ) const;

		//**********************Weixiao Update***************************//
		// search for all points within the 'radius' range.
		// return the indices of the found points in 'neighbors'.
		virtual void find_points_in_radius(
			const vec3& p,
			float,
			std::vector<int>&,
			semantic_mesh_segmentation::PTCloud*,
			std::pair<float, float> &
		) const;


		virtual void find_points_in_radius(
			const vec3& p,
			float,
			std::vector<int>&,
			PointCloud*,
			std::vector<semantic_mesh_segmentation::ptx_z> &
		) const;

		virtual void find_points_in_radius_minmax(
			const vec3& p,
			float,
			std::vector<int>&,
			PointCloud*,
			semantic_mesh_segmentation::ptx_minmax &
		) const;

		virtual bool find_points_in_radius(
			const vec3&,
			float,
			std::vector<int>&,
			std::map<int, int> &,
			const int
		) const;

		virtual void find_points_in_radius(
			const vec3& ,
			float ,
			std::map<int, int> &,
			std::map<int, int> &,
			std::map<int, std::vector<int>> &,
			const int 
		) const;
		//***************************************************************//

    protected:
        std::vector<vec3>*	points_; // reference of the original point cloud data
        void*				tree_;
    } ;

} // namespace easy3d

#endif  // EASY3D_KD_TREE_H


