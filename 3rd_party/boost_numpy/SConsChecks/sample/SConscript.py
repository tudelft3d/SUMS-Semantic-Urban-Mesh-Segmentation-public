# -*- python -*-

import os
import sys
import platform
import subprocess
from SConsChecks import AddLibOptions, GetLibChecks

_libs = ['boost.numpy',
         'boost.preprocessor',
         'boost.python',
         'boost.serialization',
         'boost.test',
         'boost.thread',
         'python',
         'numpy',
         'opencv']

_checks = GetLibChecks(_libs)

def getRequiredLibs():
  req_libs = ['boost.preprocessor']
  if GetOption('serialization_enabled'):
    req_libs.append('boost.serialization')
  if GetOption('with_python'):
    req_libs.extend(['python',
                     'numpy',
                     'boost.python',
                     'boost.numpy',
                     'boost.thread',
                     'opencv'])
  if GetOption('with_tests'):
    req_libs.append('boost.test')
  return req_libs

# Setup command-line options
def setupOptions():
    AddOption("--prefix-dir", dest="prefix", type="string", nargs=1, action="store",
              metavar="DIR", default=default_prefix, help="installation prefix")
    AddOption("--install-headers-dir", dest="install_headers", type="string", nargs=1, action="store",
              metavar="DIR", help="location to install header files (overrides --prefix for headers)")
    AddOption("--install-lib-dir", dest="install_lib", type="string", nargs=1, action="store",
              metavar="DIR", help="location to install libraries (overrides --prefix for libraries)")
    AddOption("--with-serialization", dest="serialization_enabled",
              action="store_true", help="enable serialization (requires boost serialization)",
              default=False),
    AddOption("--with-python", dest="with_python",
              action="store_true", help="enables building the python library",
              default=False),
    AddOption("--with-tests", dest="with_tests",
              action="store_true", help="enables building the test suite",
              default=False),
    AddOption("--rpath", dest="custom_rpath", type="string", action="append",
              help="runtime link paths to add to libraries and executables (unix); may be passed more than once")
    # Add library configuration options.
    AddLibOptions(AddOption, _libs)
    # Default variables.
    variables = Variables()
    variables.Add("CCFLAGS", default=os.environ.get("CCFLAGS", "-O2 -g"), help="compiler flags")
    return variables

def makeEnvironment(variables):
    shellEnv = {}
    # Some of these don't make sense on Windows, but don't hurt.
    for key in ("PATH", "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH", "PYTHONPATH"):
        if key in os.environ:
            shellEnv[key] = os.environ[key]
    env = Environment(variables=variables, ENV=shellEnv)
    return env

def setupTargets(env, root="."):
    lib, headers = SConscript(os.path.join(root, "yourlibrary", "SConscript.py"),
                              exports='env',
                              variant_dir='build/yourlibrary')
    if GetOption('with_python'):
      python_module = SConscript(os.path.join(root, "pyyourlibrary", "SConscript.py"),
                                 exports='env',
                                 variant_dir='build/pyyourlibrary')
    if GetOption('with_tests'):
      tests_executable = SConscript(os.path.join(root, "yourtests", "SConscript.py"),
                                    exports='env',
                                    variant_dir='build/yourtests')
    prefix = Dir(GetOption("prefix")).abspath
    install_headers = GetOption('install_headers')
    install_lib = GetOption('install_lib')
    if not install_headers:
        install_headers = os.path.join(prefix, "include")
    if not install_lib:
        install_lib = os.path.join(prefix, "lib")
    env.Alias("install", env.Install(install_lib, lib))
    for header in headers:
        env.Alias("install", env.Install(os.path.join(install_headers, "yourlibrary"),
                                           os.path.join(root, "yourlibrary", header.name)))

Return("setupOptions",
       "makeEnvironment",
       "setupTargets",
       "_checks",
       "getRequiredLibs")
