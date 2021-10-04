// Copyright (c) 2012  Tel-Aviv University (Israel).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Arrangement_on_surface_2/demo/Arrangement_on_surface_2/ISnappable.h $
// $Id: ISnappable.h ee57fc2 %aI Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0+
//
// Author(s)     : Alex Tsui <alextsui05@gmail.com>

#ifndef ISNAPPABLE_H
#define ISNAPPABLE_H

class ISnappable
{
public:
  virtual ~ISnappable( ) { }
  virtual void setSnappingEnabled( bool b ) = 0;
  virtual void setSnapToGridEnabled( bool b ) = 0;
}; // class ISnappable


#endif // SNAPPABLE_H
