// Copyright (c) 2011  INRIA Sophia-Antipolis (France).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Spatial_sorting/include/CGAL/Hilbert_sort_middle_base.h $
// $Id: Hilbert_sort_middle_base.h 0698f79 %aI Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0+
//
// Author(s)     :  Olivier Devillers

#ifndef CGAL_HILBERT_SORT_MIDDLE_BASE_H
#define CGAL_HILBERT_SORT_MIDDLE_BASE_H

#include <CGAL/config.h>
#include <algorithm>

namespace CGAL {

namespace internal {

    template <class RandomAccessIterator, class Cmp>
    RandomAccessIterator
    fixed_hilbert_split (RandomAccessIterator begin, RandomAccessIterator end,
                   Cmp cmp = Cmp ())
    {
        if (begin >= end) return begin;

        return std::partition (begin, end, cmp);
    }
}

} // namespace CGAL

#endif//CGAL_HILBERT_SORT_MIDDLE_BASE_H
