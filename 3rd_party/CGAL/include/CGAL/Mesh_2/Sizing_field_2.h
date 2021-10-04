// Copyright (c) 2013 INRIA Sophia-Antipolis (France),
//               2014-2015 GeometryFactory (France).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Mesh_2/include/CGAL/Mesh_2/Sizing_field_2.h $
// $Id: Sizing_field_2.h ee57fc2 %aI Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0+
// 
//
// Author(s) : Jane Tournois, Pierre Alliez
//

#ifndef CGAL_SIZING_FIELD_2_H
#define CGAL_SIZING_FIELD_2_H

#include <CGAL/license/Mesh_2.h>


#include <list>

#include <CGAL/basic.h>

namespace CGAL
{

template <typename Tr>
class Sizing_field_2 // pure virtual class
{    
public:
  typedef typename Tr::Geom_traits::Point_2 Point_2;
  typedef typename Tr::Geom_traits::FT      FT;

public:
  Sizing_field_2()
  {
  }
  Sizing_field_2(Tr& )
  {
  }

  virtual ~Sizing_field_2()
  {
  }

  virtual FT operator()(const Point_2& p) const = 0;
};

}//namespace CGAL

#endif
