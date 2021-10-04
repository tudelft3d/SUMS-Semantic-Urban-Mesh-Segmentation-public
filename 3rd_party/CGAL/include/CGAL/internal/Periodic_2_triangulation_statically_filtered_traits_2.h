// Copyright (c) 1997-2013, 2017 INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Periodic_2_triangulation_2/include/CGAL/internal/Periodic_2_triangulation_statically_filtered_traits_2.h $
// $Id: Periodic_2_triangulation_statically_filtered_traits_2.h cbde108 %aI Mael Rouxel-Labbé
// SPDX-License-Identifier: GPL-3.0+
//
// Author(s)     : Nico Kruithof <Nico@nghk.nl>,
//                 Mael Rouxel-Labbé

// This class gathers optimized predicates written by hand, using
// a few steps of filtering.  It should work if the initial traits has
// cartesian coordinates which fit exactly in doubles.
//
// Purely static filters code has been removed, since it requires additional
// logic and is not plug'n play (requires users providing bounds).
// If it should be provided again, it should probably be separate.

#ifndef CGAL_PERIODIC_2_TRIANGULATION_STATICALLY_FILTERED_TRAITS_2_H
#define CGAL_PERIODIC_2_TRIANGULATION_STATICALLY_FILTERED_TRAITS_2_H

#include <CGAL/license/Periodic_2_triangulation_2.h>

#include <CGAL/internal/Periodic_2_triangulation_filtered_traits_2.h>
#include <CGAL/internal/Static_filters/Periodic_2_orientation_2.h>

namespace CGAL {

template<class K_,
         class Off_ = typename CGAL::Periodic_2_offset_2>
class Periodic_2_triangulation_statically_filtered_traits_2
  : public Periodic_2_triangulation_filtered_traits_base_2<K_, Off_>
{
  typedef Periodic_2_triangulation_statically_filtered_traits_2<K_, Off_>   Self;
  typedef Periodic_2_triangulation_filtered_traits_base_2<K_, Off_>         Base;

public:
  typedef K_                                                                Kernel;
  typedef typename Kernel::Iso_rectangle_2                                  Iso_rectangle_2;

  Periodic_2_triangulation_statically_filtered_traits_2(const Iso_rectangle_2& domain,
                                                        const Kernel& k)
    : Base(domain, k)
  { }

  typedef internal::Static_filters_predicates::Periodic_2_orientation_2<
            Self, typename Base::Orientation_2> Orientation_2;

  Orientation_2 orientation_2_object() const
  {
    return Orientation_2(&this->_domain, this->Base::orientation_2_object());
  }
};

} // namespace CGAL

#endif // CGAL_PERIODIC_2_TRIANGULATION_STATICALLY_FILTERED_TRAITS_2_H
