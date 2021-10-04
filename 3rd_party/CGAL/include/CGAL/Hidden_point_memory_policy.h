// Copyright (c) 2017  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/STL_Extension/include/CGAL/Hidden_point_memory_policy.h $
// $Id: Hidden_point_memory_policy.h 8cdfad0 %aI Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0+
//
// Author(s)     : Mael Rouxel-Labbé

#ifndef CGAL_HIDDEN_POINT_MEMORY_POLICY_H
#define CGAL_HIDDEN_POINT_MEMORY_POLICY_H

#include <CGAL/tags.h>

namespace CGAL {

// A policy to select whether hidden points should be cached in cells or discarded
// during the construction of a regular triangulation.

template < typename Tag >
struct Hidden_points_memory_policy : public Tag { };

typedef Hidden_points_memory_policy<Tag_true> Keep_hidden_points;
typedef Hidden_points_memory_policy<Tag_false> Discard_hidden_points;

} // namespace CGAL

#endif // CGAL_HIDDEN_POINT_MEMORY_POLICY_H
