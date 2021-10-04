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


#ifndef EASY3D_TYPES_H
#define EASY3D_TYPES_H

#include <cstdint>

#include <easy3d/vec.h>
#include <easy3d/mat.h>
#include <easy3d/box.h>
#include <easy3d/quat.h>


/**
* Gathers different basic types for geometric operations.
* Types defined here are points/vectors, lines, segments,
* planes, boxes and matrices in 2D and 3D.
*/

namespace easy3d {

    //____________________ default types___________________

    typedef Vec<2, float>				vec2;
    typedef Vec<3, float>				vec3;
    typedef Vec<4, float>				vec4;

    typedef Mat2<float>					mat2;
    typedef Mat3<float>					mat3;
    typedef Mat4<float>					mat4;
    typedef Mat<3, 4, float>			mat34;
    typedef Mat<4, 3, float>			mat43;

    typedef Quat<float>					quat;

    typedef GenericBox2<float>			Box2;
    typedef GenericBox3<float>			Box3;

namespace geom {

    /* Returns a vector orthogonal to v. Its norm() depends on v, but is zero only for a null v.*/
    inline vec3 orthogonal(const vec3& v) {
        float absx = std::fabs(v.x);
        float absy = std::fabs(v.y);
        float absz = std::fabs(v.z);
        // Find smallest component. Keep equal case for null values.
        if ((absy >= absx) && (absz >= absx))
            return vec3(0.0, -v.z, v.y);
        else
            if ((absx >= absy) && (absz >= absy))
                return vec3(-v.z, 0.0f, v.x);
            else
                return vec3(-v.y, v.x, 0.0f);
    }

    // bounding box for 3D/2D point set
    template < typename InputIterator >
    inline Box3 bounding_box_3(InputIterator first, InputIterator last) {
        mpl_debug_assert(first != last);
        Box3 result;
        for (InputIterator it = first; it != last; ++it) {
            result.add_point(*it);
        }
        return result;
    }

    template < typename InputIterator >
    inline Box2 bounding_box_2(InputIterator first, InputIterator last) {
        mpl_debug_assert(first != last);
        Box2 result;
        for (InputIterator it = first; it != last; ++it) {
            result.add_point(*it);
        }
        return result;
    }

    // centroid for 3D/2D point set
    template < typename InputIterator >
    inline vec3 centroid_3(InputIterator begin, InputIterator end) {
        mpl_debug_assert(begin != end);
        vec3 v(0, 0, 0);
        unsigned int nb_pts = 0;
        for (InputIterator it = begin; it != end; ++it) {
            v += (*it);
            ++nb_pts;
        }
        return v / nb_pts;
    }

    template < typename InputIterator >
    inline vec2 centroid_2(InputIterator begin, InputIterator end) {
        mpl_debug_assert(begin != end);
        vec2 v(0, 0);
        unsigned int nb_pts = 0;
        for (InputIterator it = begin; it != end; ++it) {
            v += (*it);
            ++nb_pts;
        }
        return v / nb_pts;
    }

    inline vec3 barycenter(const vec3& p1, const vec3& p2) {
        return vec3(
            0.5f * (p1.x + p2.x),
            0.5f * (p1.y + p2.y),
            0.5f * (p1.z + p2.z)
            );
    }

    inline vec2 barycenter(const vec2& p1, const vec2& p2) {
        return vec2(
            0.5f * (p1.x + p2.x),
            0.5f * (p1.y + p2.y)
            );
    }

    inline vec3 barycenter(
        const vec3& p1, const vec3& p2, const vec3& p3
        ) {
        return vec3(
            (p1.x + p2.x + p3.x) / 3.0f,
            (p1.y + p2.y + p3.y) / 3.0f,
            (p1.z + p2.z + p3.z) / 3.0f
            );
    }

    inline vec2 barycenter(
        const vec2& p1, const vec2& p2, const vec2& p3
        ) {
        return vec2(
            (p1.x + p2.x + p3.x) / 3.0f,
            (p1.y + p2.y + p3.y) / 3.0f
            );
    }

    inline vec3 barycenter(
        const vec3& p1, const vec3& p2, const vec3& p3, const vec3& p4
        ) {
        return vec3(
            0.25f * (p1.x + p2.x + p3.x + p4.x),
            0.25f * (p1.y + p2.y + p3.y + p4.y),
            0.25f * (p1.z + p2.z + p3.z + p4.z)
            );
    }


    inline float cos_angle(const vec3& a, const vec3& b) {
        float na2 = length2(a);
        float nb2 = length2(b);
        return dot(a, b) / ::sqrt(na2 * nb2);
    }

    inline float angle(const vec3& a, const vec3& b) {
        return std::acos(cos_angle(a, b));
    }


    inline float cos_angle(const vec2& a, const vec2& b) {
        float na2 = length2(a);
        float nb2 = length2(b);
        return dot(a, b) / std::sqrt(na2 * nb2);
    }

    inline float det(const vec2& a, const vec2& b) {
        return a.x*b.y - a.y*b.x;
    }


    /* returns the angle in the interval [-pi .. pi] */
    inline float angle(const vec2& a, const vec2& b) {
        return det(a, b) > 0 ?
            std::acos(cos_angle(a, b)) :
            -std::acos(cos_angle(a, b));
    }

    // round the given floating point number v to be num_digits.
    // TODO: this function should not be in this file.
    template <class T>
    T truncate_digits(const T& v, int num_digits) {
        T tmp = std::pow(10.0f, num_digits);
        long long des = static_cast<long long>((v < 0) ? (v * tmp - 0.5f) : (v * tmp + 0.5f));
        T result = T(des) / tmp;
        return result;
    }

    // radians
    template <typename T>
    inline T to_radians(T degrees) {
        return degrees * static_cast<T>(0.01745329251994329576923690768489);
    }


    // degrees
    template <typename T>
    inline T to_degrees(T radians) {
        return radians * static_cast<T>(57.295779513082320876798154814105);
    }


    inline float triangle_area(
        const vec3& p1, const vec3& p2, const vec3& p3
        ) {
        return 0.5f * length(cross(p2 - p1, p3 - p1));
    }

    inline float triangle_signed_area(
        const vec2& p1, const vec2& p2, const vec2& p3
        ) {
        return 0.5f * det(p2 - p1, p3 - p1);
    }

    inline vec3 triangle_normal(
        const vec3& p1, const vec3& p2, const vec3& p3
        ) {
        vec3 n = cross(p2 - p1, p3 - p2);
        return normalize(n);
    }

} // namespace Geom

}

#endif  // EASY3D_TYPES_H


