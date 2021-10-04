# -*- python -*-
from SConsChecks import GetLibChecks

# Get the general build environment options and methods.
setupOptions, makeEnvironment, setupTargets, checks, getRequiredLibs = \
  SConscript("SConscript.py", variant_dir='build')

# Prepare the command line options.
variables = setupOptions()

# Create the build environment.
env = makeEnvironment(variables)

# The root path must be added, since the fertilized headers are
# referenced.
env.AppendUnique(CPPPATH="#.")

# Only take actions if neither help nor clean is specified.
if not GetOption("help") and not GetOption("clean"):
    config = env.Configure(custom_tests=checks)
    checknames = GetLibChecks(getRequiredLibs()).keys()
    if False in (config.__dict__[checkname]() for checkname in checknames):
        Exit(1)
    env = config.Finish()

# The targets must be created in any case (remember, the 'clean' option
# need to know about them).
setupTargets(env)
