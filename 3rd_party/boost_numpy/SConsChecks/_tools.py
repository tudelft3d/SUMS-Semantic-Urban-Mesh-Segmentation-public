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

import os

def _setupPaths(env,
                prefix,
                include,
                lib,
                include_add_dir='include',
                lib_add_dir='lib'):
    if prefix is not None:
        if include is None:
            include = os.path.join(prefix, include_add_dir)
        if lib is None:
            lib = os.path.join(prefix, lib_add_dir)
    if include:
        env.PrependUnique(CPPPATH=[include])
    if lib:
        env.PrependUnique(LIBPATH=[lib])

def _checkLibs(context, try_libs, source_file, file_ending='.cpp'):
    init_libs = context.env.get('LIBS', [])
    context.env.PrependUnique(LIBS=[try_libs])
    result = context.TryLink(source_file, file_ending)
    if not result :
        context.env.Replace(LIBS=init_libs)
    return result
