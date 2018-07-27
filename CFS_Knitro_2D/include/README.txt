====================================================
=== Important note on object-oriented interfaces ===
====================================================

Release: v1.1 for Knitro 10.2

----------------
IMPORTANT NOTICE
----------------
This is the C++ object-oriented interface for Artelys Knitro.

This C++ interface is built on top of Knitro C API and is oriented
to be primarily user firendly.

Users targetting efficiency our advised to directly use C API instead.

Also note that SIGNIFICANT CHANGES ARE EXPECTED in future Knitro versions.
Future version will not be backwards compatible.

However, since the interfaces is relying on Knitro C interface,
this version of the interfaces will still work with future releases of Knitro.

Comments and feedbacks on object-oriented interfaces are welcome and should
be addressed to support-knitro@artelys.com.


---------------------------------------
HOW TO BUILD THE C++ INTERFACE EXAMPLES
---------------------------------------
Below are  the instructions to build knitro C++ examples for each platform.
You need CMake [https://cmake.org/download/] and a C++ compiler to build these examples.

The example ExHS15_termination requires a C++0x (or C++11) compiler (remove this target
in CMakeLists.txt if your compiler does not support it).
The interface and other examples are compatible with older C++ versions.

========= Windows =========
Define the environment variable KNITRODIR as the Knitro root directory.

Create a directory "build" with the "C++" interfaces directory.

From the directory build, open a command prompt run "cmake .. -G [My Visual]"
where [My visual] is your visual studio version with target platform, e.g. :
    > cmake .. -G "Visual Studio 14 2015 Win64"

This will creater Visual Studio projects wich you can now open and build with Visual Studio.

Other supported building systems can be listed using :
    > cmake --help

========= Linux and Mac OS =========
Define the environment variable KNITRODIR as the Knitro root directory.
Add the directory containing the Knitro dynamic library to the environment variable
LD_LIBRARY_PATH (or DYLD_LIBRARY_PATH on Mac OS).

Create a directory "build" in the "C++" interfaces directory.

From the directory build, run "cmake ..".

This will create a makefile in the build folder.

Run "make" in the build folder. This will create executables for the examples.