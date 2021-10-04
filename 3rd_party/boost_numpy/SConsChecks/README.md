# Simple library checking with SCONS

This collection of library checks is provided as a convenience helper. It 
offers the possibility to perform a platform independent check for the library
(including compilation and execution) and automatically adds command line
configuration options for it. It works with Python 2 and 3 and is tested on Windows and Linux.

## Usage

Every library has an assigned library id string. Currently, checks are
implemented for:

- 'boost.datetime'
- 'boost.filesystem'
- 'boost.interprocess'
- 'boost.numpy',
- 'boost.preprocessor',
- 'boost.python',
- 'boost.serialization',
- 'boost.system',
- 'boost.test',
- 'boost.thread',
- 'cuda',
- 'eigen',
- 'fftw',
- 'hdf5',
- 'matlab',
- 'openblas'
- 'numpy',
- 'opencv',
- 'python',
- 'protobuf',
- 'swig'.

You can either:

```
import SConsChecks
# call checks directly
SConsChecks.CheckNumPy()
```

or:

```
# import these convenience methods,
from SConsChecks import AddLibOptions, GetLibChecks
# define a list of required checks,
_libs = ['boost.numpy',
         'boost.preprocessor',
         'boost.python',
         'boost.serialization',
         'boost.test',
         'boost.thread',
         'python',
         'numpy',
         'opencv',
         'eigen']
# and get a check (name, method) dict.
_checks = GetLibChecks(_libs)

# You can automatically add all command line options to configure the
# libraries as follows:
def setupOptions():
    # ...
    # Add library configuration command line options.
    AddLibOptions(AddOption, _libs)
```

See the attached sample project for details.


## Default library configuration

The following defaults are used to configure library xyz:

- Environment variable XYZ_ROOT (command-line --xyz-dir):
  the libraries root folder. If specified, the following include and 
  lib directories are used relative to the root directory.

- Environment variable XYZ_INCLUDE_DIR (command-line --xyz-inc-dir):
  the libraries header include folder.

- Environment variable XYZ_LIB_DIR (command-line --xyz-lib-dir):
  the libraries linking folder.

## Contributing

You are very welcome to extend the list of available checks. Please fork the
repo and send me a pull request.

## License and credits

This project is heavily inspired by the build system of boost numpy 
(see https://github.com/ndarray/Boost.NumPy/blob/master/SConscript), which
hass been created by Jim Bosch.

The source code is distributed under the Boost Software License, Version 1.0
(see http://www.boost.org/LICENSE_1_0.txt).
