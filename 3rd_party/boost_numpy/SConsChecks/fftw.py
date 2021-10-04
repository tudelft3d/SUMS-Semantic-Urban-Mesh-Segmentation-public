# -*- python -*-
#
# Copyright Christoph Lassner 2014.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)
from __future__ import print_function
from ._tools import _checkLibs, _setupPaths
from .python import CheckPython
from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs
import os

_check_dict = {}
_fftw_option_dict = {'--fftw-dir':
                      {'dest':"fftw_prefix",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'default':os.environ.get("FFTW_ROOT"),
                      'help':"prefix for the fftw library; should contain fftw3.h and the library files."},
                      '--fftw-inc-dir':
                      {'dest':"fftw_include",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'help':"location of fftw3.h",
                      'default':os.environ.get("FFTW_INCLUDE_DIR")},
                      '--boost-lib-dir':
                      {'dest':"fftw_lib",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'help':"location of fftw libraries",
                      'default':os.environ.get("FFTW_LIB_DIR")},
                      '--fftw-lib-flavor':
                      {'dest':"fftw_flavor",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'help':"FFTW library suffix for float or long double linkage, so either '', 'f' or 'l'.",
                       'default':""},
                      '--fftw-ver-major':
                      {'dest':"fftw_ver_major",
                       'type':"int",
                       'nargs':1,
                       'action':"store",
                       'help':"FFTW library major version, e.g. 3",
                       'default':3},
                      '--fftw-ver-minor':
                      {'dest':"fftw_ver_minor",
                       'type':"int",
                       'nargs':1,
                       'action':"store",
                       'help':"FFTW library minor version, e.g. 3",
                       'default':3}}

def CheckFFTW(context):
    fftw_dtype = 'double'
    fftw_flavor = ''
    try:
      fftw_flavor = context.env.GetOption("fftw_flavor")
    except:
      pass
    if fftw_flavor == 'f':
      fftw_dtype = 'float'
    elif fftw_flavor == 'l':
      fftw_dtype = 'long double'
    elif fftw_flavor != '':
      raise Exception("Unknown fftw flavor: %s" % (fftw_flavor))
    fftw_major = 3
    try:
      fftw_major = context.env.GetOption("fftw_ver_major")
    except:
      pass
    fftw_minor = 3
    try:
      fftw_minor = context.env.GetOption("fftw_ver_minor")
    except:
      pass

    fftw_source_file = r"""
#include <numeric>
#include <cstdint>
#include <fftw3.h>
int main() {
  fftw_complex *in, *out;
  fftw_plan p;
  std::size_t N = 10;
  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  for (std::size_t i = 0; i < N; ++i) {
    in[i][0] = static_cast<%s>(i);
    in[i][1] = static_cast<%s>(i);
  }
  fftw_execute(p);
  fftw_destroy_plan(p);
  fftw_free(in);
  fftw_free(out);
  return 0;
}
""" % (fftw_dtype, fftw_dtype)
    context.Message('Check building against fftw... ')
    fftw_root = context.env.GetOption("fftw_prefix")
    fftw_ex_inc = context.env.GetOption("fftw_include")
    fftw_ex_lib = context.env.GetOption("fftw_lib")
    if fftw_ex_inc is None and not fftw_root is None:
      fftw_ex_inc = fftw_root
    if fftw_ex_lib is None and not fftw_root is None:
      fftw_ex_lib = fftw_root
    _setupPaths(context.env,
        prefix = fftw_root,
        include = fftw_ex_inc,
        lib = fftw_ex_lib
        )
    if context.env['CC'] == 'gcc':
        context.env.AppendUnique(CPPFLAGS=['-std=c++0x'])
    if os.name == 'nt':
        fftw_libname = 'libfftw%d%s-%d' % (fftw_major, fftw_flavor, fftw_minor)
    else:
        fftw_libname = 'libfftw%d%s' % (fftw_major, fftw_flavor)
    result = context.checkLibs([fftw_libname], fftw_source_file)
    if not result:
        context.Result(0)
        print("Cannot build against fftw.")
        return False
    result, output = context.TryRun(fftw_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against fftw.")
        return False
    context.Result(1)
    return True
_check_dict['fftw'] = {'options': _fftw_option_dict,
                       'checks': [CheckFFTW]}