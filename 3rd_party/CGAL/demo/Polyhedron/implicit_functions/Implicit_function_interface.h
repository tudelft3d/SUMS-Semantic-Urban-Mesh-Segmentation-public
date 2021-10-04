// Copyright (c) 2010 INRIA Sophia-Antipolis (France).
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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Polyhedron/demo/Polyhedron/implicit_functions/Implicit_function_interface.h $
// $Id: Implicit_function_interface.h ee57fc2 %aI Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0+
//
//
// Author(s)     : Stephane Tayeb
//
//******************************************************************************
// File Description : 
//******************************************************************************

#ifndef CGAL_MESH_3_IMPLICIT_FUNCTION_INTERFACE_H
#define CGAL_MESH_3_IMPLICIT_FUNCTION_INTERFACE_H

#include <QObject>
#include <QString>
#include <CGAL/Three/Scene_interface.h>

class Implicit_function_interface 
{
public:
  typedef CGAL::Three::Scene_interface::Bbox Bbox;
  
  virtual ~Implicit_function_interface() {}
  
  virtual QString name() const = 0;
  virtual double operator()(double x, double y, double z) const = 0;
  virtual Bbox bbox() const = 0;

  /// Returns the minimum and maximum of the function.
  ///
  /// The method returns false if it cannot know efficiently its extrema.
  /// In that case, the function will be scanned over the bounding box to
  /// compute approximation of the extrema. If the method is not
  /// overloaded by derived classes, the default is to return false.
  virtual bool get_min_max(double&, double&) {
    return false;
  }
};

Q_DECLARE_INTERFACE(Implicit_function_interface,
                    "com.geometryfactory.Mesh3Demo.Implicit_function_interface/1.0")


#endif // CGAL_MESH_3_IMPLICIT_FUNCTION_INTERFACE_H
