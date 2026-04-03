# extra_script.py — PlatformIO build helper for madflight
#
# Fixes transitive dependency issue: SD_MMC → FS.h
# PlatformIO's LDF text-scanner picks up #include "SD_MMC.h" even inside
# #ifdef BBX_USE_MMC (which is not defined). SD_MMC.cpp then can't find FS.h
# because LDF fails to resolve SD_MMC's own dependency on the FS library.
#
# Solution: manually add the FS framework library's include path and source
# to the build environment.

Import("env")
import os

framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")

# Framework built-in libraries that are needed transitively
# SD_MMC → FS, SD → FS
builtin_libs = ["FS"]

for lib_name in builtin_libs:
    lib_src = os.path.join(framework_dir, "libraries", lib_name, "src")
    if os.path.isdir(lib_src):
        # Add include path (resolves #include "FS.h")
        env.Append(CPPPATH=[lib_src])
        # Compile the library source files (provides FS:: implementations)
        env.BuildSources(
            os.path.join("$BUILD_DIR", "fw_" + lib_name),
            lib_src
        )
