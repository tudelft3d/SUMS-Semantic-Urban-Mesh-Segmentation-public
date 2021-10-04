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
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0-beta1/Arrangement_on_surface_2/demo/Arrangement_on_surface_2/DeleteCurveMode.h $
// $Id: DeleteCurveMode.h ee57fc2 %aI Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0+
//
// Author(s)     : Alex Tsui <alextsui05@gmail.com>

#ifndef DELETE_CURVE_MODE_H
#define DELETE_CURVE_MODE_H

#include <QMetaType>

class QString;

/**
An attribute describing the policy for deleting curves from the arrangement.
*/
class DeleteCurveMode
{
public:
  enum Mode {
    DELETE_CURVE,
    DELETE_EDGE
  };

  DeleteCurveMode( );
  DeleteCurveMode( const DeleteCurveMode& dcm );
  DeleteCurveMode( Mode mode );
  ~DeleteCurveMode( );

  Mode mode( ) const;
  void setMode( Mode mode );

  static QString ToString( const DeleteCurveMode& mode );

protected:
  Mode m_mode;
};

Q_DECLARE_METATYPE( DeleteCurveMode )

#endif // DELETE_CURVE_MODE_H
