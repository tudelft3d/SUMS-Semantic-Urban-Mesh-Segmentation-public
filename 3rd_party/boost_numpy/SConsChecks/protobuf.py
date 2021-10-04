# -*- python -*-
#
# Copyright Christoph Lassner 2015.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)
from __future__ import print_function
from ._tools import _checkLibs, _setupPaths
from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs
import os
import subprocess

_check_dict = {}
_protobuf_option_dict = {'--protobuf-dir':
                           {'dest':"protobuf_prefix",
                           'type':"string",
                           'nargs':1,
                           'action':"store",
                           'metavar':"DIR",
                           'default':os.environ.get("PROTOBUF_ROOT"),
                           'help':"prefix for protobuf; should contain the 'src' folder."},
			'--protobuf-include-dir':
                           {'dest':"protobuf_include_dir",
                           'type':"string",
                           'nargs':1,
                           'action':"store",
                           'metavar':"DIR",
                           'default':os.environ.get("PROTOBUF_INCLUDE_DIR"),
                           'help':"the relative (to protobuf root) or absolute include path"},
			'--protobuf-lib-dir':
                           {'dest':"protobuf_lib_dir",
                           'type':"string",
                           'nargs':1,
                           'action':"store",
                           'metavar':"DIR",
                           'default':os.environ.get("PROTOBUF_LIB_DIR"),
                           'help':"the relative (to protobuf root) or absolute library path"},
                           '--protoc':
                           {'dest':"protoc",
                           'type':"string",
                           'nargs':1,
                           'action':"store",
                           'metavar':"EXEC",
                           'help':"this is the protoc executable.",
                           'default':os.environ.get("PROTOC")}}

def CheckProtobuf(context):
    context.Message('Check that protoc is available... ')
    print("env['CC']: %s, env['CXX']: %s" % (context.env['CC'], context.env['CXX']))
    if not context.env.GetOption("protoc") is None:
        result = os.path.isfile(context.env.GetOption("protoc"))
    else:
        if os.name != 'nt':
            result = not subprocess.call(['which', 'protoc'])
        else:
            result = not subprocess.call(['where', 'protoc'])
    if not result:
        context.Result(0)
        print("Cannot find protoc!")
        return False
    context.Message('Check that protobuf header files are available...')
    source_file = r"""
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
//#include <google/protobuf/generated_enum_reflection.h>
//#include <google/protobuf/unknown_field_set.h>

int main() {
  return 0;
}
"""
    ex_prefix_dir = context.env.GetOption("protobuf_prefix")
    ex_lib_dir = context.env.GetOption("protobuf_lib_dir")
    try:
        debug_build = context.env.GetOption("debug_build")
    except:
        debug_build = False
	if ex_lib_dir is None or ex_lib_dir == '':
		if debug_build:
			ex_lib_dir = os.path.join('vsprojects', 'x64', 'Debug')
		else:
			ex_lib_dir = os.path.join('vsprojects', 'x64', 'Release')
    ex_include_dir = context.env.GetOption("protobuf_include_dir")
    _setupPaths(context.env,
                prefix = ex_prefix_dir,
                include = ex_include_dir,
                lib = ex_lib_dir
                )
    result = context.checkLibs(['libprotobuf'], source_file)
    if not result:
        context.Result(0)
        print("Cannot build with PROTOBUF headers.")
        return False
    context.Result(1)
    return True
_check_dict['protobuf'] = {'options': _protobuf_option_dict,
                           'checks': [CheckProtobuf]}
