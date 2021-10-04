# -*- python -*-
#
# Copyright Christoph Lassner 2015.
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
_openblas_option_dict = {'--openblas-dir':
                       {'dest':"openblas_prefix",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'default':os.environ.get("OPENBLAS_ROOT"),
                       'help':"prefix for OpenBlas; should contain the 'bin', 'include', and 'lib' folders."},
                       '--openblas-inc-dir':
                       {'dest':"openblas_include",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'help':"this folder should contain 'cblas.h'.",
                       'default':os.environ.get("OPENBLAS_INCLUDE_DIR")},
                       '--openblas-lib-dir':
                       {'dest':"openblas_lib",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'help':"this folder should contain 'libopenblas'.",
                       'default':os.environ.get("OPENBLAS_LIB_DIR")}}

def CheckOpenBLAS(context):
    sample_source_file = r"""
#include <cblas.h>
#include <iostream>
#include <cstdlib>

using namespace std;

int main(int argc, char** argv)
{
  int n=2;
  double* x = (double*)malloc(n*sizeof(double));
  double* upperTriangleResult = (double*)malloc(n*(n+1)*sizeof(double)/2);

  for (int j=0;j<n*(n+1)/2;j++) upperTriangleResult[j] = 0;
  x[0] = 1; x[1] = 3;

  cblas_dspr(CblasRowMajor,CblasUpper,n,1,x,1,upperTriangleResult);
  double*& A = upperTriangleResult;
  cout << A[0] << "\t" << A[1] << endl << "*\t" << A[2] << endl;

  free(upperTriangleResult); free(x);
  return EXIT_SUCCESS;
}
"""
    context.Message('Check building with OpenBLAS... ')
    ex_prefix_dir = context.env.GetOption("openblas_prefix")
    ex_lib_dir = context.env.GetOption("openblas_lib")
    ex_include_dir = context.env.GetOption("openblas_include")
    _setupPaths(context.env,
                prefix = ex_prefix_dir,
                include = ex_include_dir,
                lib = ex_lib_dir
                )
    if os.name == 'nt':
        result = (context.checkLibs(['libopenblas.dll.a'], sample_source_file))
    else:
        result = (context.checkLibs(['openblas'], sample_source_file))
    if not result:
        context.Result(0)
        print("Cannot build with OpenBLAS.")
        return False
    result, output = context.TryRun(sample_source_file,'.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built with OpenBLAS.")
        return False
    context.Result(1)
    return True
_check_dict['openblas'] = {'options': _openblas_option_dict,
                           'checks': [CheckOpenBLAS]}
