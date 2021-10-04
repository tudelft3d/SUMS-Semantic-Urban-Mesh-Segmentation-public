# -*- python -*-
#
# Copyright Christoph Lassner 2015.
#
# Distributed under the Boost Software License, Version 1.0.
#    (See http://www.boost.org/LICENSE_1_0.txt)

from __future__ import print_function
import os
import sys

from ._tools import _checkLibs, _setupPaths

from SCons.SConf import CheckContext
CheckContext.checkLibs = _checkLibs

_check_dict = {}
_cuda_option_dict  = {'--cuda-dir':
                       {'dest':"cuda_prefix",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'default':os.environ.get("CUDA_ROOT"),
                       'help':"prefix for cuda; should contain the   'bin', 'include', and 'lib' folders."},
                       '--cuda-inc-dir':
                       {'dest':"cuda_include",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'help':"this folder should contain 'cuda.h'.",
                       'default':os.environ.get("CUDA_INCLUDE_DIR")},
                       '--cuda-lib-dir':
                       {'dest':"cuda_lib",
                       'type':"string",
                       'nargs':1,
                       'action':"store",
                       'metavar':"DIR",
                       'help':"this folder should contain 'libcudart_static.a'.",
                       'default':os.environ.get("CUDA_LIB_DIR")}}

def CheckCUDA(context):
    context.Message("Checking for CUDA...")
    if os.name == 'nt':
      result, nvcc_cmd = context.TryAction("where nvcc")
      result, cl_cmd = context.TryAction("where cl")
      nvcc_cmd = None
    else:
      result, nvcc_cmd = context.TryAction("which nvcc > $TARGET")
    context.Result(result)
    if result:
        if not nvcc_cmd is None:
          print("Using nvcc at", nvcc_cmd.strip())
    context.Message("Check building with CUDA...")
    sample_source_file = r"""
#include <stdio.h>
 
__global__
void add(int *a, int *b, int *c ) {
    *c = *a + *b;
}
 
int main( void ) {
    int a, b, c;
    // host copies of a, b, c
    int *dev_a, *dev_b, *dev_c;
    // device copies of a, b, c
    int size = sizeof(int);
    // we need space for an integer
    // allocate device copies of a, b, c
    cudaMalloc( (void**)&dev_a, size );
    cudaMalloc( (void**)&dev_b, size );
    cudaMalloc( (void**)&dev_c, size );
    a = 2;
    b = 7;
    // copy inputs to device
    cudaMemcpy(dev_a, &a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(dev_b, &b, size, cudaMemcpyHostToDevice);
    // launch add() kernel on GPU, passing parameters
    add<<< 1, 1 >>>(dev_a,dev_b,dev_c);
    // copy device result back to host copy of c
    cudaMemcpy( &c,dev_c, size,cudaMemcpyDeviceToHost);
    cudaFree(dev_a);
    cudaFree(dev_b);
    cudaFree(dev_c);
    return 0;
}
"""
    ex_prefix_dir = context.env.GetOption("cuda_prefix")
    ex_lib_dir = context.env.GetOption("cuda_lib")
    ex_include_dir = context.env.GetOption("cuda_include")
    is_64bits = sys.maxsize > 2**32
    if is_64bits:
        lib_add_dir = 'lib64'
    else:
        lib_add_dir = 'lib'
    _setupPaths(context.env,
                prefix = ex_prefix_dir,
                include = ex_include_dir,
                lib = ex_lib_dir,
                lib_add_dir = lib_add_dir
                )
    defines_saved = context.env['CPPDEFINES']
    context.env['CPPDEFINES'] = ''
    result_bld = (context.checkLibs(['cudart', 'cublas', 'cublas_device',
                                     'cufft', 'cufftw', 'curand'],
                                     sample_source_file,
                                     '.cu'))
    if not result_bld:
        context.Result(0)
        print("Cannot build with CUDA.")
        return False
    result, output = context.TryRun(sample_source_file, '.cu')
    if not result:
        print("Warning: cannot run program built with CUDA. Continuing anyway.")
    context.env['CPPDEFINES'] = defines_saved
    context.Result(1)
    return True
_check_dict['cuda'] = {'options': _cuda_option_dict,
                       'checks': [CheckCUDA]}
