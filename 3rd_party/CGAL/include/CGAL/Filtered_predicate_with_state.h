// Copyright (c) 2001-2005  INRIA Sophia-Antipolis (France).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Filtered_kernel/include/CGAL/Filtered_predicate_with_state.h $
// $Id: Filtered_predicate_with_state.h 82c13e6 %aI Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0+
// 
//
// Author(s)     : Sylvain Pion, Andreas Fabri, Sebastien Loriot

#ifndef CGAL_FILTERED_PREDICATE_WITH_STATE_H
#define CGAL_FILTERED_PREDICATE_WITH_STATE_H

#include <string>
#include <CGAL/config.h>
#include <CGAL/Interval_nt.h>
#include <CGAL/Uncertain.h>
#include <CGAL/Profile_counter.h>
#include <boost/optional.hpp>

namespace CGAL {

// This template class is a wrapper that implements the filtering for any
// predicate with state (dynamic filters with IA).

  template <class EP, class AP, class C2E, class C2A, class O1, bool Protection = true>
class Filtered_predicate_with_state
{
  C2E c2e;
  C2A c2a;
  O1  o1;
  mutable boost::optional<EP>  oep;
  AP  ap;
  typedef typename AP::result_type  Ares;

public:

  typedef AP    Approximate_predicate;
  typedef EP    Exact_predicate;
  typedef C2E   To_exact_converter;
  typedef C2A   To_approximate_converter;

  typedef typename EP::result_type  result_type;
  // AP::result_type must be convertible to EP::result_type.

  Filtered_predicate_with_state(const O1 &o1)
    : c2e(), c2a(), o1(o1), oep(), ap(c2a(o1))
  {}

  template <typename... Args>
  result_type
  operator()(const Args&... args) const;
};

template <class EP, class AP, class C2E, class C2A, class O1, bool Protection>
  template <typename... Args>
typename Filtered_predicate_with_state<EP,AP,C2E,C2A,O1,Protection>::result_type
Filtered_predicate_with_state<EP,AP,C2E,C2A,O1,Protection>::
  operator()(const Args&... args) const
{
    CGAL_BRANCH_PROFILER(std::string(" failures/calls to   : ") + std::string(CGAL_PRETTY_FUNCTION), tmp);
    // Protection is outside the try block as VC8 has the CGAL_CFG_FPU_ROUNDING_MODE_UNWINDING_VC_BUG
    {
      Protect_FPU_rounding<Protection> p;
      try
	{
	  Ares res = ap(c2a(args)...);
	  if (is_certain(res))
	    return get_certain(res);
	}
      catch (Uncertain_conversion_exception&) {}
    }
    CGAL_BRANCH_PROFILER_BRANCH(tmp);
    Protect_FPU_rounding<!Protection> p(CGAL_FE_TONEAREST);
    if(! oep){
      #if BOOST_VERSION < 105600
      oep = EP(c2e(o1));
      #else
      oep.emplace(c2e(o1));
      #endif
    }
    return (*oep)(c2e(args)...);
}

} //namespace CGAL

#endif // CGAL_FILTERED_PREDICATE_WITH_STATE_H
