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
from ._tools import _checkLibs, _setupPaths
from .python import CheckPython
from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs
import os

_check_dict = {}
_boost_option_dict = {'--boost-dir':
                      {'dest':"boost_prefix",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'default':os.environ.get("BOOST_ROOT"),
                      'help':"prefix for Boost libraries; should have 'include' and 'lib' subdirectories"},
                      '--boost-inc-dir':
                      {'dest':"boost_include",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'help':"location of Boost header files",
                      'default':os.environ.get("BOOST_INCLUDE_DIR")},
                      '--boost-lib-dir':
                      {'dest':"boost_lib",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"DIR",
                      'help':"location of Boost libraries",
                      'default':os.environ.get("BOOST_LIB_DIR")},
                      '--boost-compiler-str':
                      {'dest':"boost_comp",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"STR",
                      'help':"the compiler string used in the Windows Boost library names, e.g., vc110",
                      'default':os.environ.get("BOOST_COMP_STR")},
                      '--boost-version-str':
                      {'dest':"boost_ver",
                      'type':"string",
                      'nargs':1,
                      'action':"store",
                      'metavar':"VER",
                      'help':"the version string used in the Windows Boost library names, e.g., 1_54",
                      'default':os.environ.get("BOOST_VER_STR")}}
_boost_np_option_dict = { '--boost-np-dir':
                          {'dest':"boost_np_prefix",
                          'type':"string",
                          'nargs':1,
                          'action':"store",
                          'metavar':"DIR",
                          'default':os.environ.get("BOOST_ROOT"),
                          'help':"prefix for Boost numpy libraries; should have 'include' and 'lib' subdirectories"},
                          '--boost-np-inc-dir':
                          {'dest':"boost_np_include",
                          'type':"string",
                          'nargs':1,
                          'action':"store",
                          'metavar':"DIR",
                          'help':"location of Boost numpy header files",
                          'default':os.environ.get("BOOST_INCLUDE_DIR")},
                          '--boost-np-lib-dir':
                          {'dest':"boost_np_lib",
                          'type':"string",
                          'nargs':1,
                          'action':"store",
                          'metavar':"DIR",
                          'help':"location of Boost numpy libraries",
                          'default':os.environ.get("BOOST_LIB_DIR")}}
_boost_python_option_dict = { '--boost-python-lib':
                              {'dest':"boost_python_lib",
                              'type':"string",
                              'nargs':1,
                              'action':"store",
                              'metavar':"FILENAME",
                              'default':"boost_python",
                              'help':"boost python library filename"}}

def _set_boost_path(context):
    boostpre = context.env.GetOption("boost_prefix")
    boostinc = context.env.GetOption("boost_include")
    boostlib = context.env.GetOption("boost_lib")
    if os.name == 'nt':
        _setupPaths(context.env,
            prefix = boostpre,
            include = boostinc,
            lib = boostlib,
            include_add_dir='',
            lib_add_dir=os.path.join('stage', 'lib')
            )
    else:
        _setupPaths(context.env,
            prefix = boostpre,
            include = boostinc,
            lib = boostlib
            )

def CheckBoostPython(context):
    bp_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include "boost/python.hpp"
class Foo { public: Foo() {} };
int main()
{
  Py_Initialize();
  boost::python::object obj;
  boost::python::class_< Foo >("Foo", boost::python::init<>());
  Py_Finalize();
  return 0;
}
"""
    context.Message('Check building against Boost.Python... ')
    _set_boost_path(context)
    boost_python_lib = context.env.GetOption('boost_python_lib')
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], bp_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs([boost_python_lib+'-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file) or \
                     context.checkLibs([boost_python_lib+'-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file)
        result = (
            nt_try or
            context.checkLibs([''], bp_source_file) or
            context.checkLibs([boost_python_lib], bp_source_file) or
            context.checkLibs([boost_python_lib+'-mt'], bp_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Python.")
        return False
    result, output = context.TryRun(bp_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Python.")
        return False
    context.Result(1)
    return True
_check_dict['boost.python'] = {'options': dict(_boost_option_dict, **_boost_python_option_dict),
                                'checks': [CheckPython, CheckBoostPython]}

def CheckBoostNumpy(context):
    bp_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include "boost/python.hpp"
#include <boost/numpy.hpp>
namespace py = boost::python;
namespace np = boost::numpy;
class Foo { public: Foo(np::ndarray &t) {} };
int main()
{
  Py_Initialize();
  np::initialize();
  boost::python::object obj;
  boost::python::class_< Foo >("Foo", boost::python::init<np::ndarray&>());
  Py_Finalize();
  return 0;
}
"""
    context.Message('Check building against Boost.Numpy... ')
    _setupPaths(context.env,
        prefix = context.env.GetOption("boost_np_prefix"),
        include = context.env.GetOption("boost_np_include"),
        lib = context.env.GetOption("boost_np_lib")
        )
    if context.env.GetOption("debug_checks"):
      result = (context.checkLibs(['boost_numpy_d'], bp_source_file))
    else:
      result = (context.checkLibs(['boost_numpy'], bp_source_file))
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Numpy.")
        return False
    result, output = context.TryRun(bp_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Numpy.")
        return False
    context.Result(1)
    return True
_check_dict['boost.numpy'] = {'options': dict(_boost_option_dict, **_boost_np_option_dict),
                              'checks': [CheckBoostPython, CheckBoostNumpy]}

def CheckBoostSerialization(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <sstream>

int main()
{
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  int t = 5;
  oa << t;
  int t2 = 0;
  boost::archive::text_iarchive ia(ss);
  ia >> t2;
  return (t != t2);
}
"""
    context.Message('Check building against Boost.Serialization... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_serialization-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], boost_source_file) or \
                     context.checkLibs(['boost_serialization-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], boost_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_serialization'], boost_source_file) or
            context.checkLibs(['boost_serialization-mt'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Serialization.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Serialization.")
        return False
    context.Result(1)
    return True
_check_dict['boost.serialization'] = {'options': _boost_option_dict,
                                      'checks': [CheckBoostSerialization]}

def CheckBoostThread(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/thread.hpp>

void workerFunc() {
  int i = 0;
  for (int j = 0; j < 5; ++j)
    ++i;
}

int main()
{
  boost::thread workerThread(workerFunc);
  workerThread.join();
  return 0;
}
"""
    context.Message('Check building against Boost.Thread... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_thread-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver')),
                       'boost_system-%s-%s' %\
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file) or \
                     context.checkLibs(['boost_thread-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver')),
                       'boost_system-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_thread', 'boost_system'], boost_source_file) or
            context.checkLibs(['boost_thread-mt', 'boost_system-mt'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Thread.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Thread.")
        return False
    context.Result(1)
    return True
_check_dict['boost.thread'] = {'options': _boost_option_dict,
                                'checks': [CheckBoostThread]}
                                
def CheckBoostDateTime(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/date_time/gregorian/gregorian.hpp>

int
main() 
{
  using namespace boost::gregorian;
  std::string s = "2014-02-19";
  date birthday(from_simple_string(s));
  date today = day_clock::local_day();
  days days_alive = today - birthday;
  return 0;
}
"""
    context.Message('Check building against Boost.DateTime... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_date_time-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file) or \
                     context.checkLibs(['boost_date_time-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_date_time'], boost_source_file) or
            context.checkLibs(['boost_date_time-mt'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.DateTime.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.DateTime.")
        return False
    context.Result(1)
    return True
_check_dict['boost.datetime'] = {'options': _boost_option_dict,
                                'checks': [CheckBoostDateTime]}

def CheckBoostSystem(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/system/system_error.hpp>

using namespace std;

int main(int argc, char* argv[])
{
  int err_code = errno;
  boost::system::error_code ec (err_code,
    boost::system::system_category ());
  return 0;
}
"""
    context.Message('Check building against Boost.System... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_system-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file) or \
                     context.checkLibs(['boost_system-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_system'], boost_source_file) or
            context.checkLibs(['boost_system-mt'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.System.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.System.")
        return False
    context.Result(1)
    return True
_check_dict['boost.system'] = {'options': _boost_option_dict,
                               'checks': [CheckBoostSystem]}

def CheckBoostTest(context):
    boost_source_file = r"""
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "dummy module"
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( my_test )
{
    BOOST_REQUIRE( 1+1==2 );
}
"""
    context.Message('Check building against Boost.Test... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        context.env.AppendUnique(LINKFLAGS='/SUBSYSTEM:CONSOLE')
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_unit_test_framework-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], boost_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_unit_test_framework'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Test.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Test.")
        return False
    context.Result(1)
    return True
_check_dict['boost.test'] = {'options': _boost_option_dict,
                             'checks': [CheckBoostSystem, CheckBoostTest]}

def CheckBoostFilesystem(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::filesystem;

int main(int argc, char* argv[])
{
  path p (".");

  if (! (exists(p) && is_directory(p)))
      return 1;
  return 0;
}
"""
    context.Message('Check building against Boost.Filesystem... ')
    _set_boost_path(context)
    if context.env['CC'] == 'cl':
        # Use msvc's autolinking support.
        result = (context.checkLibs([''], boost_source_file))
    else:
        nt_try = False
        if os.name == 'nt':
            # Lost here, since Boost is built with compiler and version
            # suffix in that case. If it's not set, give up.
            nt_try = context.checkLibs(['boost_filesystem-%s-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file) or \
                     context.checkLibs(['boost_filesystem-%s-mt-%s' % \
                       (GetOption('boost_comp'), GetOption('boost_ver'))], bp_source_file)
        result = (
            nt_try or
            context.checkLibs([''], boost_source_file) or # icl support
            context.checkLibs(['boost_filesystem'], boost_source_file) or
            context.checkLibs(['boost_filesystem-mt'], boost_source_file)
            )
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Filesystem.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Filesystem.")
        return False
    context.Result(1)
    return True
_check_dict['boost.filesystem'] = {'options': _boost_option_dict,
                                   'checks': [CheckBoostSystem, CheckBoostFilesystem]}

def CheckBoostInterprocess(context):
    boost_source_file = r"""
// Get diagnostics in the log.
#define BOOST_LIB_DIAGNOSTIC
#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdlib> //std::system

int main (int argc, char *argv[])
{
   using namespace boost::interprocess;
      //Remove shared memory on construction and destruction
      struct shm_remove
      {
         shm_remove() {  shared_memory_object::remove("MySharedMemory"); }
         ~shm_remove(){  shared_memory_object::remove("MySharedMemory"); }
      } remover;

      //Create a managed shared memory segment
      managed_shared_memory segment(create_only, "MySharedMemory", 65536);

      //Allocate a portion of the segment (raw memory)
      managed_shared_memory::size_type free_memory = segment.get_free_memory();
      void * shptr = segment.allocate(1024/*bytes to allocate*/);

      //Check invariant
      if(free_memory <= segment.get_free_memory())
         return 1;
   return 0;
}
"""
    context.Message('Check building against Boost.Interprocess... ')
    _set_boost_path(context)
    if os.name == 'nt':
        result = (context.checkLibs([], boost_source_file))
    else:
        result = (context.checkLibs(['rt', 'pthread'], boost_source_file))
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Interprocess.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Interprocess.")
        return False
    context.Result(1)
    return True
_check_dict['boost.interprocess'] = {'options': _boost_option_dict,
                                     'checks': [CheckBoostInterprocess]}

def CheckBoostPP(context):
    boost_source_file = r"""
#include <boost/preprocessor/cat.hpp>

#define STATIC_ASSERT(EXPR)\
  enum\
  { BOOST_PP_CAT(static_check_,__LINE__) = (EXPR) ? 1 : -1\
  };\
  typedef char\
    BOOST_PP_CAT(static_assert_,__LINE__)\
    [ BOOST_PP_CAT(static_check_,__LINE__)\
    ]

int main()
{
  STATIC_ASSERT(sizeof(int) <= sizeof(long));
  return 0;
}
"""
    context.Message('Check building against Boost.Preprocessor... ')
    _set_boost_path(context)
    result = (context.checkLibs([''], boost_source_file))
    if not result:
        context.Result(0)
        print("Cannot build against Boost.Preprocessor.")
        return False
    result, output = context.TryRun(boost_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against Boost.Preprocessor.")
        return False
    context.Result(1)
    return True
_check_dict['boost.preprocessor'] = {'options': _boost_option_dict,
                                     'checks': [CheckBoostPP]}
