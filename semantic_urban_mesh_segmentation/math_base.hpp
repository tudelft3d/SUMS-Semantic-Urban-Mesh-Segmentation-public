/*
*   Name        : math_base.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : basic math functions for sampling mesh point clouds  
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
#ifndef semantic_mesh_segmentation__MATH_BASE_HPP
#define semantic_mesh_segmentation__MATH_BASE_HPP

#include <float.h>
#include <math.h>
#include <assert.h>
#include <cmath>
#include <limits>
#include <algorithm>

#ifdef __BORLANDC__
float sqrtf(float v) { return sqrt(v); }
float fabsf(float v) { return fabs(v); }
float cosf(float v) { return cos(v); }
float sinf(float v) { return sin(v); }
float acosf(float v) { return acos(v); }
float asinf(float v) { return asin(v); }
float atan2f(float v0, float v1) { return atan2(v0, v1); }
#endif

namespace semantic_mesh_segmentation {
	namespace math {
		template <class SCALAR>

		class MagnitudoComparer
		{
		public:
			inline bool operator() (const SCALAR a, const SCALAR b) { return fabs(a) > fabs(b); }
		};
			   
		inline float Sqrt(const short v) { return sqrtf(v); }
		inline float Sqrt(const int v) { return sqrtf((float)v); }

		inline float Sqrt(const float v) { return sqrtf(v); }

		inline float Abs(const float v) { return fabsf(v); }

		inline float Cos(const float v) { return cosf(v); }

		inline float Sin(const float v) { return sinf(v); }

		inline float Acos(const float v) { return acosf(v); }

		inline float Asin(const float v) { return asinf(v); }

		inline float Atan2(const float v0, const float v1) { return atan2f(v0, v1); }

		inline double Sqrt(const double v) { return sqrt(v); }

		inline double Abs(const double v) { return fabs(v); }

		inline double Cos(const double v) { return cos(v); }

		inline double Sin(const double v) { return sin(v); }

		inline double Acos(const double v) { return acos(v); }

		inline double Asin(const double v) { return asin(v); }

		inline double Atan2(const double v0, const double v1) { return atan2(v0, v1); }

		template <typename T> inline static T Sqr(T a) { return a * a; }

		template<class T> inline const T & Min(const T &a, const T &b, const T &c)
		{

			if (a < b)
			{
				if (a < c) 
					return a;
				else 
					return c;
			}
			else 
			{
				if (b < c) 
					return b;
				else 
					return c;
			}
		}

		template<class T> inline const T & Max(const T &a, const T &b, const T &c) 
		{
			if (a > b) 
			{
				if (a > c) 
					return a;
				else
					return c; // if c<a then c is smaller than b...

			}
			else 
			{

				if (b > c) 
					return b;
				else 
					return c;
			}
		}



		template<class T> inline void Sort(T &a, T &b) 
		{
			if (a > b) std::swap(a, b);
		}

		template<class T> inline void Sort(T &a, T &b, T &c) 
		{

			if (a > b) std::swap(a, b);
			if (b > c) 
			{ 
				std::swap(b, c); 
				if (a > b) 
					std::swap(a, b); 
			}

		}

		/* Some <math.h> files do not define M_PI... */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef SQRT_TWO
#define SQRT_TWO 1.4142135623730950488
#endif

		template <class SCALAR>
		inline SCALAR  Clamp(const SCALAR & val, const SCALAR& minval, const SCALAR& maxval)
		{
			if (val < minval) return minval;
			if (val > maxval) return maxval;
			return val;
		}

		inline float   ToDeg(const float &a) { return a * 180.0f / float(M_PI); }
		inline float   ToRad(const float &a) { return float(M_PI)*a / 180.0f; }
		inline double  ToDeg(const double &a) { return a * 180.0 / M_PI; }
		inline double  ToRad(const double &a) { return M_PI * a / 180.0; }

		template <typename T>
		int Sgn(T val) {
			return (T(0) < val) - (val < T(0));
		}

#if defined(_MSC_VER) // Microsoft Visual C++

		template<class T> int IsNAN(T t) { return _isnan(t) || (!_finite(t)); }

#elif defined(__MINGW32__) // GCC

		template<class T> int IsNAN(T t) { return std::isnan(t) || std::isinf(t); }

#elif defined(__GNUC__) // GCC

		template<class T> int IsNAN(T t) { return std::isnan(t) || std::isinf(t); }

#else // generic
		template<class T> int IsNAN(T t)
		{
			if (std::numeric_limits<T>::has_infinity)
				return !(t <= std::numeric_limits<T>::infinity());
			else
				return t != t;
		}
#endif
}	// End math namespace

/// a type that stands for "void". Useful for Parameter type of a point.

	class VoidType 
	{
	public:
		VoidType() {};
	};
}	// End semantic_mesh_segmentation namespace

#endif//semantic_mesh_segmentation__MATH_BASE
