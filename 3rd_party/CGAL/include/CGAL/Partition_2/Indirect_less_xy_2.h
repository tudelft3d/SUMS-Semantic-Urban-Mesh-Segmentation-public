// Copyright (c) 2000  Max-Planck-Institute Saarbruecken (Germany).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Partition_2/include/CGAL/Partition_2/Indirect_less_xy_2.h $
// $Id: Indirect_less_xy_2.h ca161c5 %aI Andreas Fabri
// SPDX-License-Identifier: GPL-3.0+
// 
//
// Author(s)     : Susan Hert <hert@mpi-sb.mpg.de>

#ifndef CGAL_INDIRECT_LESS_XY_2_H
#define CGAL_INDIRECT_LESS_XY_2_H

#include <CGAL/license/Partition_2.h>


namespace CGAL {

template <class Traits>
class Indirect_less_xy_2 
{
   public:
     typedef typename Traits::Less_xy_2     Less_xy_2;

     Indirect_less_xy_2(const Traits& traits) : _less_xy_2(traits.less_xy_2_object()) 
     { }
     
     template <class Iterator>
     bool 
     operator()(Iterator p, Iterator q) const
     { 
        return _less_xy_2(*p, *q);
     }

   private:
     Less_xy_2 _less_xy_2;
};

}

#endif // CGAL_INDIRECT_LESS_XY_2_H
