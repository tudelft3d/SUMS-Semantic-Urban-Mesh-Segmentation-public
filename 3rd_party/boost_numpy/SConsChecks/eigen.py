# -*- python -*-
#
# Copyright Christoph Lassner 2014.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)

from __future__ import print_function
import sys
import os

from ._tools import _checkLibs, _setupPaths

from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs

_check_dict = {}
_eigen_option_dict = {'--eigen-dir':
                      {'dest':"eigen_prefix",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'default':os.environ.get("EIGEN_ROOT"),
                      'help':"prefix for Eigen library; should have 'Eigen' and 'unsupported' subdirectories"}}

def CheckEigen(context):
    eigen_source_file = r"""
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  Matrix2d n;
  n <<  3,  -1,
       2.5,  1.5;
  return 1 - (m==n);
}
"""
    context.Message('Check building with Eigen... ')
    include_dir = context.env.GetOption("eigen_prefix")
    try:
      ex_include_dir = context.env.GetOption("eigen_include")
      if not ex_include_dir is None:
        include_dir = ex_include_dir
    except:
      pass
    _setupPaths(context.env,
                prefix = None,
                include = include_dir,
                lib = None
                )
    result = (context.checkLibs([], eigen_source_file))
    if not result:
        context.Result(0)
        print("Cannot build with Eigen.")
        return False
    result, output = context.TryRun(eigen_source_file,'.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built with Eigen.")
        return False
    context.Result(1)
    return True
_check_dict['eigen'] = {'options': _eigen_option_dict,
                        'checks': [CheckEigen]}
