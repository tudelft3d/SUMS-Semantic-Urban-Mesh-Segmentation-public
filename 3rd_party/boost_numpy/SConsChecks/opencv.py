# -*- python -*-
#
# These SCons tests are based on the ones of boost numpy
# (https://github.com/ndarray/Boost.NumPy/blob/master/SConscript),
# but have been altered and substantially extended.
#
# Copyright Christoph Lassner 2014.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)

from __future__ import print_function
import platform
import os
import sys

from ._tools import _checkLibs, _setupPaths
from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs

_check_dict = {}
_opencv_option_dict = {'--opencv-dir':
                        {'dest':"opencv_prefix",
                        'type':"string",
                        'nargs':1,
                        'action':"store",
                        'metavar':"DIR",
                        'default':os.environ.get("OPENCV_ROOT"),
                        'help':"prefix for OpenCV libraries; should have 'include' and 'lib' subdirectories"},
                        '--opencv-inc-dir':
                        {'dest':"opencv_include",
                        'type':"string",
                        'nargs':1,
                        'action':"store",
                        'metavar':"DIR",
                        'help':"location of OpenCV header files",
                        'default':os.environ.get("OPENCV_INCLUDE_DIR")},
                        '--opencv-lib-dir':
                        {'dest':"opencv_lib",
                        'type':"string",
                        'nargs':1,
                        'action':"store",
                        'metavar':"DIR",
                        'help':"location of OpenCV libraries",
                        'default':os.environ.get("OPENCV_LIB_DIR")},
                        '--opencv-version':
                        {'dest':"opencv_version",
                        'type':"string",
                        'nargs':1,
                        'action':"store",
                        'metavar':"VERSION",
                        'help':"opencv version, without dots, e.g. 248",
                        'default':os.environ.get("OPENCV_VERSION")}}

def CheckOpenCV(context):
    opencv_source_file = r"""
#include <opencv2/opencv.hpp>

int main()
{
  cv::Mat test = cv::Mat::eye(5, 5, CV_64F);
  return 0;
}
"""
    context.Message('Check building against OpenCV... ')
    _setupPaths(context.env,
        prefix = context.env.GetOption("opencv_prefix"),
        include = context.env.GetOption("opencv_include"),
        lib = context.env.GetOption("opencv_lib")
        )
    libsuffix = ''
    try:
        if context.env.GetOption("debug_checks"):
          libsuffix = 'd'
    except:
        pass
    if platform.system() == 'Windows':
        if context.env.GetOption('opencv_version') is None:
            print("Please specify the OpenCV version using the " +\
                  "environment variable OPENCV_VERSION or using " +\
                  "the parameter --opencv-version as the number that " +\
                  "is attached to the library files, e.g., 2411 for "+\
                  "version 2.4.11.")
            context.Result(0)
            return False
        result = (context.checkLibs(['opencv_imgproc' + context.env.GetOption("opencv_version") + libsuffix,
                                     'opencv_highgui' + context.env.GetOption("opencv_version") + libsuffix,
                                     'opencv_core' + context.env.GetOption("opencv_version") + libsuffix], opencv_source_file))
    else:
        result = (context.checkLibs(['opencv_imgproc' + libsuffix,
                                     'opencv_highgui' + libsuffix,
                                     'opencv_core' + libsuffix,
                                     'tiff',
                                     'freetype'], opencv_source_file))
        if not result:
            # Its not common on Linux to have the 'd' suffix for debugging.
            result = (context.checkLibs(['opencv_imgproc',
                                         'opencv_highgui',
                                         'opencv_core',
                                         'tiff',
                                         'freetype'], opencv_source_file))
    if not result:
        context.Result(0)
        print("Cannot build against OpenCV.")
        return False
    result, output = context.TryRun(opencv_source_file, '.cpp')
    if not result:
        context.Result(0)
        print("Cannot run program built against OpenCV.")
        return False
    context.Result(1)
    return True
_check_dict['opencv'] = {'options': _opencv_option_dict,
                         'checks': [CheckOpenCV]}

