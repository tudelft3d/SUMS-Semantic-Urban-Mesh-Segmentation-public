# -*- python -*-
#
# These SCons tests are based on the ones of ndarray
# (https://github.com/ndarray/ndarray),
# but have been altered and substantially extended.
#
# Copyright Christoph Lassner 2014.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)

from __future__ import print_function
import os

_check_dict = {}

def CheckSwig(context):
    context.Message("Checking for SWIG...")
    context.env.PrependUnique(SWIGFLAGS = ["-python", "-c++"])
    if os.name == 'nt':
      result, swig_cmd = context.TryAction("where swig")
      swig_cmd = None
    else:
      result, swig_cmd = context.TryAction("which swig > $TARGET")
    context.Result(result)
    if result:
        if not swig_cmd is None:
          print("Using SWIG at", swig_cmd.strip())
    return result
_check_dict['swig'] = {'options': {},
                       'checks': [CheckSwig]}
