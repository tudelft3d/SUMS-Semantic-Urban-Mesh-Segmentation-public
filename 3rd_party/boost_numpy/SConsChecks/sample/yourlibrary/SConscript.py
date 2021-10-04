# -*- python -*-

import os
Import("env")

# Create a temporary environment to be able to modify it locally.
lib_env = env.Clone()
# Create the build file list.
file_list = Glob('*.cpp')
headers = Glob('*.h')
# The library.
lib_file = lib_env.SharedLibrary('yourlibrary', file_list)
Return("lib_file", "headers")
