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

from __future__ import print_function
import os
import sys
import platform
import subprocess

from ._tools import _checkLibs, _setupPaths
from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs

_check_dict = {}
def CheckPython(context):
    python_source_file = r"""
// If defined, enforces linking againg PythonXXd.lib, which
// is usually not included in Python environments.
#undef _DEBUG
#include "Python.h"
int main()
{
  Py_Initialize();
  Py_Finalize();
  return 0;
}
"""
    context.Message('Check building against Python... ')
    try:
        import distutils.sysconfig
    except ImportError:
        context.Result(0)
        print('Failed to import distutils.sysconfig.')
        return False
    context.env.AppendUnique(CPPPATH=[distutils.sysconfig.get_python_inc()])
    if platform.system() == 'Windows':
        libfile = os.path.join(os.path.dirname(sys.executable),
                               'libs',
                               'python27.lib')
        context.env.AppendUnique(LIBS=[libfile])
    else:
        libDir = distutils.sysconfig.get_config_var("LIBDIR")
        context.env.AppendUnique(LIBPATH=[libDir])
        libfile = distutils.sysconfig.get_config_var("LIBRARY")
        import re
        match = re.search("(python.*)\.(a|so|dylib)", libfile)
        if match:
            context.env.AppendUnique(LIBS=[match.group(1)])
            if match.group(2) == 'a':
                flags = distutils.sysconfig.get_config_var('LINKFORSHARED')
                if flags is not None:
                    context.env.AppendUnique(LINKFLAGS=flags.split())
        flags = [f for f in " ".join(distutils.sysconfig.get_config_vars("MODLIBS", "SHLIBS")).split()
                 if f != "-L"]
        context.env.MergeFlags(" ".join(flags))
    result, output = context.TryRun(python_source_file,'.cpp')
    if not result and context.env["PLATFORM"] == 'darwin':
        # Sometimes we need some extra stuff on Mac OS
        frameworkDir = libDir       # search up the libDir tree for the proper home for frameworks
        while frameworkDir and frameworkDir != "/":
            frameworkDir, d2 = os.path.split(frameworkDir)
            if d2 == "Python.framework":
                if not "Python" in os.listdir(os.path.join(frameworkDir, d2)):
                    context.Result(0)
                    print((
                        "Expected to find Python in framework directory %s, but it isn't there"
                        % frameworkDir))
                    return False
                break
        context.env.AppendUnique(LINKFLAGS="-F%s" % frameworkDir)
        result, output = context.TryRun(python_source_file,'.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built with Python.")
        return False
    if context.env["PLATFORM"] == "darwin":
        context.env["LDMODULESUFFIX"] = ".so"
    context.Result(1)
    return True
_check_dict['python'] = {'options': {},
                         'checks': [CheckPython]}

def CheckNumPy(context):
    numpy_source_file = r"""
// If defined, enforces linking againg PythonXXd.lib, which
// is usually not included in readymade Python environments.
#undef _DEBUG
#include "Python.h"
#include "numpy/arrayobject.h"
#if PY_MAJOR_VERSION == 2
void doImport() {
  import_array();
}
#else
void * doImport() {
  import_array();
  return nullptr;
}
#endif
int main()
{
  int result = 0;
  Py_Initialize();
  doImport();
  if (PyErr_Occurred()) {
    result = 1;
  } else {
    npy_intp dims = 2;
    PyObject * a = PyArray_SimpleNew(1, &dims, NPY_INT);
    if (!a) result = 1;
    Py_DECREF(a);
  }
  Py_Finalize();
  return result;
}
"""
    context.Message('Check building against NumPy... ')
    try:
        import numpy
    except ImportError:
        context.Result(0)
        print('Failed to import numpy.')
        print('Things to try:')
        print('1) Check that the command line python (with which you probably installed numpy):')
        print('   ', end=' ')
        sys.stdout.flush()
        subprocess.call('which python',shell=True)
        print('  is the same as the one used by SCons:')
        print('  ', sys.executable)
        print('   If not, then you probably need to reinstall numpy with %s.' % sys.executable)
        print('   Alternatively, you can reinstall SCons with your preferred python.')
        print('2) Check that if you open a python session from the command line,')
        print('   import numpy is successful there.')
        return False
    context.env.Append(CPPPATH=numpy.get_include())
    result = context.checkLibs([''],numpy_source_file)
    if not result:
        context.Result(0)
        print("Cannot build against NumPy.")
        return False
    result, output = context.TryRun(numpy_source_file,'.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built with NumPy.")
        return False
    context.Result(1)
    return True
_check_dict['numpy'] = {'options': {},
                        'checks': [CheckPython, CheckNumPy]}
