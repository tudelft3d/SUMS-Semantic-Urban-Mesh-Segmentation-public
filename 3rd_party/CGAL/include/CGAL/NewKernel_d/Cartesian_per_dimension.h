// Copyright (c) 2014
// INRIA Saclay-Ile de France (France)
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/NewKernel_d/include/CGAL/NewKernel_d/Cartesian_per_dimension.h $
// $Id: Cartesian_per_dimension.h 0698f79 %aI Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0+
//
// Author(s)     : Marc Glisse

#ifndef CGAL_KD_CARTESIAN_PER_DIM_H
#define CGAL_KD_CARTESIAN_PER_DIM_H
#include <CGAL/NewKernel_d/functor_tags.h>
#include <CGAL/Dimension.h>
#include <CGAL/predicates/sign_of_determinant.h>

// Should probably disappear.

namespace CGAL {
template <class Dim_, class R_, class Derived_>
struct Cartesian_per_dimension : public R_ {};
}

#endif
