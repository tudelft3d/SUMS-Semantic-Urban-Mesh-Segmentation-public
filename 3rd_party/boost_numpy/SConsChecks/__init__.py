# -*- python -*-
#
# These SCons tests are based on the ones of boost numpy
# (https://github.com/ndarray/Boost.NumPy/blob/master/SConscript),
# but have been altered and substantially extended.
#
# Copyright Christoph Lassner 2014.
# For the python, numpy and boost python checks, _setupPaths and _checkLibs:
# Copyright Jim Bosch 2010-2012.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)

from collections import OrderedDict

from .boost import CheckBoostNumpy, \
                   CheckBoostPP, \
                   CheckBoostPython, \
                   CheckBoostSerialization, \
                   CheckBoostTest, \
                   CheckBoostThread, \
                   CheckBoostDateTime, \
                   CheckBoostFilesystem, \
                   CheckBoostInterprocess, \
                   CheckBoostSystem, \
                   _check_dict

from .python import CheckPython, \
                    CheckNumPy, \
                    _check_dict as _python_check_dict
_check_dict = dict(_check_dict, **_python_check_dict)

from .opencv import CheckOpenCV,\
                    _check_dict as _opencv_check_dict
_check_dict = dict(_check_dict, **_opencv_check_dict)

from .eigen import CheckEigen, \
                   _check_dict as _eigen_check_dict
_check_dict = dict(_check_dict, **_eigen_check_dict)

from .swig import CheckSwig, \
                  _check_dict as _swig_check_dict
_check_dict = dict(_check_dict, **_swig_check_dict)

from .fftw import CheckFFTW, \
                  _check_dict as _fftw_check_dict
_check_dict = dict(_check_dict, **_fftw_check_dict)

from .matlab import CheckMatlab, \
                  _check_dict as _matlab_check_dict
_check_dict = dict(_check_dict, **_matlab_check_dict)

from .openblas import CheckOpenBLAS, \
                   _check_dict as _openblas_check_dict
_check_dict = dict(_check_dict, **_openblas_check_dict)

from .hdf5 import CheckHDF5, \
                   _check_dict as _hdf5_check_dict
_check_dict = dict(_check_dict, **_hdf5_check_dict)

from .cuda import CheckCUDA, \
                   _check_dict as _cuda_check_dict
_check_dict = dict(_check_dict, **_cuda_check_dict)

from .protobuf import CheckProtobuf, \
                   _check_dict as _protobuf_check_dict
_check_dict = dict(_check_dict, **_protobuf_check_dict)

def AddLibOptions(add_method, lib_names):
  r"""
  Provide the `AddOption` method for your enviromnent, and all command line
  options for the provided `lib_names` will be created automatically.

  Example:
  >>> AddLibOptions(AddOption, ['boost.test'])

  """
  options = {}
  for lib_name in lib_names:
    if not lib_name in list(_check_dict.keys()):
      raise Exception("Unknown library: %s." % (lib_name))
    for option, keywords in list(_check_dict[lib_name]['options'].items()):
      if not option in list(options.keys()):
        options[option] = keywords
  for option in list(options.keys()):
    add_method(option, **options[option])

def GetLibChecks(lib_names):
  r"""
  Returns an OrderedDict with names and methods for all setup checks for the
  provided library names.

  Example:
  >>> GetLibChecks(['boost.test'])
  {'CheckBoostTest' : CheckBoostTest}
  """
  checks = OrderedDict()
  for lib_name in lib_names:
    if not lib_name in list(_check_dict.keys()):
      raise Exception("Unknown library: %s" % (lib_name))
    for check in _check_dict[lib_name]['checks']:
      if not check.__name__ in list(checks.keys()):
        checks[check.__name__] = check
  return checks
